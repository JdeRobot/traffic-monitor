#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "math.h"
#include <sys/types.h>
#include <opencv/cv.h>

#include "feature_tracking.h"
#include "road_detection.h"
#include "movement_model.h"
#include "planar_geom.h"

CvPoint2D32f* new_points = 0;
CvPoint2D32f* swap_points;
char* status = 0;
int need_to_init = 0;
int night_mode = 0;
int flags = 0;
int add_remove_pt = 0;
CvPoint pt;
int win_size = 10;

#define MIN_ANGLE 90
#define MAX_DISTANCE 10000

namespace trafficmonitor{

#define DEBUG 0
#define LOG(args...){if (DEBUG) printf(args);}

#define MAX_NEW_BLOBS 10
#define MAX_TEMPORAL_ID 1000
#define DEFAULT_MIN_SEP 10
#define MIN_DISTANCE 10
#define MAX_DENSITY_DISTANCE 30

/**
 *
 */
FeatureTracker::FeatureTracker(){blob_temporal_id=1;}

/**
 *
 */
void FeatureTracker::init(){
   blob_temporal_id=1;
   mov_direction.a = 0;
   mov_direction.b = 0;
}

/**
 *
 */
  void FeatureTracker::track_blobs(bool klt_only
                                   ,colorspaces::Image& new_frame
                                   ,timeval timestamp
                                   ,vector<Blob*>& tracked_blobs
                                   ,vector<Blob*>& tmp_blobs
                                   ,unsigned int num_blobs)
  {
    int i, bin_w, c;
    IplImage tmpImg(new_frame);
    Blob matched_blob;
    static bool init = false;

   LOG(" ===> tracking %zu blobs\n", tracked_blobs.size());

   if (!init)
   {
      /* allocate all the buffers */
      image = cvCreateImage( cvGetSize(&tmpImg), 8, 3 );
      image->origin = tmpImg.origin;
      grey = cvCreateImage( cvGetSize(&tmpImg), 8, 1 );
      prev_grey = cvCreateImage( cvGetSize(&tmpImg), 8, 1 );
      pyramid = cvCreateImage( cvGetSize(&tmpImg), 8, 1 );
      prev_pyramid = cvCreateImage( cvGetSize(&tmpImg), 8, 1 );
      new_points = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(CvPoint2D32f));
      status = (char*)cvAlloc(MAX_COUNT);
      flags = 0;
      init = true;
   }

   cvCopy( &tmpImg, image, 0 );
   cvCvtColor( image, grey, CV_BGR2GRAY );

   /** Init tracked blobs*/
   for(unsigned int i=0; i<tracked_blobs.size(); i++)
   {
      tracked_blobs[i]->set_matched(false);
   }

   for(unsigned int i=0; i<tmp_blobs.size(); i++)
   {
      tmp_blobs[i]->tracking_blobs_id.clear();
   }

   Road* road = Road::Instance();
   Tsegment tmp_finish_line;
   if (road)
      road->get_tracking_zone_finish_line(&tmp_finish_line);

   for (unsigned int i=0; i<tracked_blobs.size(); i++)
   {
      Blob* blob = tracked_blobs[i];

      if (blob->is_free())
         continue;

      Blob matched_blob;
      if (match_blob(klt_only, blob, tmp_blobs, num_blobs, &matched_blob))
      {
         /** Check if we are close to the finish line, in this case calculate the blob
          *  trajectory related stats.
          */
         if (GET_LINE_POINT_DISTANCE(tmp_finish_line, blob->get_2d_center()) < MIN_DISTANCE)
         {
            blob->end_of_tracking(timestamp);
         }
      }
      else
      {
         Tvector tmp;
         get_vector(&tmp, blob->get_first_2d_center(), blob->get_2d_center());
         add_vector(&tmp, &mov_direction);
         blob->end_of_tracking(timestamp);
      }
   }

   process_detected_blobs(grey, timestamp, tracked_blobs, tmp_blobs, num_blobs);

   CV_SWAP( prev_grey, grey, swap_temp );
   CV_SWAP( prev_pyramid, pyramid, swap_temp );
}

/**
 *
 */
CvRect FeatureTracker::get_bounding_rect(CvPoint2D32f* points, int count)
{
   CvPoint2D32f left_corner;
   CvPoint2D32f right_corner;

   left_corner.x = INT_MAX;
   left_corner.y = INT_MAX;
   right_corner.x = 0;
   right_corner.y = 0;

   for(int i=0; i<count; i++)
   {
      if (points[i].x>0 && points[i].y>0)
      {
         Tpoint2D p;
         p.x = (int)points[i].x;
         p.y = (int)points[i].y;
         if (!MovModel::Instance()->movement_on(p))
         {
            continue;
         }

         left_corner.x = min(left_corner.x, points[i].x);
         left_corner.y = min(left_corner.y, points[i].y);
         right_corner.x = max(right_corner.x,  points[i].x);
         right_corner.y = max(right_corner.y, points[i].y);
      }
   }

   CvRect result;
   result.x = (int)left_corner.x;
   result.y = (int)left_corner.y;
   result.width = (int)(right_corner.x-left_corner.x);
   result.height = (int)(right_corner.y-left_corner.y);

   return result;
}

/**
 *
 */
void FeatureTracker::process_detected_blobs(IplImage* frame,
                                            timeval timestamp,
                                            vector<Blob*>& tracked_blobs,
                                            vector<Blob*>& tmp_blobs,
                                            unsigned int num_blobs)
{
   Road* road = Road::Instance();
   unsigned char free_positions[MAX_NEW_BLOBS];
   memset(free_positions, 0, sizeof(free_positions));
   int k=0;

   LOG("Processing detected blobs ..\n");

   /** Populate free positions*/
   for (unsigned int i=0; i<tracked_blobs.size(); i++)
   {
      if (tracked_blobs[i]->is_free())
      {
         free_positions[k++ % MAX_NEW_BLOBS] = i;
      }
   }

   /*************************************************************************
    ** Once we have matched all the blobs, now we will check if the
    ** non-matched blobs are new blobs (detection zone), in this case
    ** a new blob is created so it will be tracked in the next calls
    **************************************************************************/
   k=0;
   for (unsigned int i=0; i<num_blobs and k<MAX_NEW_BLOBS; i++)
   {
      if (tmp_blobs[i]->is_free())
         continue;

      if (tmp_blobs[i]->already_matched())
      {
         LOG("tmp_blob %d have been already matched\n", i);

         if (tmp_blobs[i]->tracking_blobs_id.size() == 1)
         {
            Blob* blob = find_blob(tracked_blobs, tmp_blobs[i]->tracking_blobs_id[0]);
            if (blob)
            {
               blob->ocluded = false;
               LOG("blob %d found correctly, updating feature points\n", tmp_blobs[i]->tracking_blobs_id[0]);
               blob->track(*tmp_blobs[i]);
               get_good_features(grey,blob);
            }
            else
            {
               LOG("can't find the blob -> blob_id = %d\n", tmp_blobs[i]->tracking_blobs_id[0]);
            }
         }
         else if (tmp_blobs[i]->tracking_blobs_id.size() > 1)
         {
            LOG("\n tmp_blob %d with size(%d:%d) pos(%d:%d) has %zu followers \n"
                ,i
                ,tmp_blobs[i]->get_rect().width
                ,tmp_blobs[i]->get_rect().height
                ,tmp_blobs[i]->get_2d_center().x
                ,tmp_blobs[i]->get_2d_center().y
                ,tmp_blobs[i]->tracking_blobs_id.size());

            for(unsigned int m=0; m<tmp_blobs[i]->tracking_blobs_id.size(); m++)
            {
               Blob* blob = find_blob(tracked_blobs, tmp_blobs[i]->tracking_blobs_id[m]);
               if (blob)
               {
                  LOG(" Followed by blob %d size(%d:%d) pos(%d:%d) (now is ocluded!)\n"
                      ,blob->get_id()
                      ,blob->get_rect().width
                      ,blob->get_rect().height
                      ,blob->get_2d_center().x
                      ,blob->get_2d_center().y);


                  blob->ocluded = true;
               }
               else
               {
                  LOG("BUG: cant find the blob -> %d\n", tmp_blobs[i]->tracking_blobs_id[m]);
               }
            }
         }
      }
      else if (road->in_detection_zone(tmp_blobs[i]->get_2d_center()))
      {
         unsigned int free_pos = free_positions[k++];
         if (free_pos <tracked_blobs.size())
         {
            //TODO: comment
            *tracked_blobs[free_pos] = *tmp_blobs[i];
            tracked_blobs[free_pos]->set_id(blob_temporal_id++ % MAX_TEMPORAL_ID);
            tracked_blobs[free_pos]->start_tracking(timestamp);

            Blob* blob = tracked_blobs[free_pos];

            get_good_features(grey,blob);

            printf(">>>> new vehicle (%d) tracked=%s stored on pos=%d np=%d per=%1.f 2dcenter(%d:%d) size(%d:%d) 3dcenter(%1.f:%1.f:%1.f) in_timestamp=%u\n\n",
                   tracked_blobs[free_pos]->get_id(),
                   tracked_blobs[free_pos]->is_being_tracked()?"true":"false",
                   free_pos,
                   tracked_blobs[free_pos]->get_num_points(),
                   tracked_blobs[free_pos]->density(),
                   tracked_blobs[free_pos]->get_2d_center().y,
                   tracked_blobs[free_pos]->get_2d_center().x,
                   tracked_blobs[free_pos]->get_rect().width,
                   tracked_blobs[free_pos]->get_rect().height,
                   tracked_blobs[free_pos]->get_current_3d_center().X,
                   tracked_blobs[free_pos]->get_current_3d_center().Y,
                   tracked_blobs[free_pos]->get_current_3d_center().Z,
                   (unsigned int)tracked_blobs[free_pos]->get_ingress_timestamp().tv_usec);
         }
         else
         {
            LOG("Trying to store the new blob in a non-valid pos %d\n",free_pos);
         }
      }
      //IF

   }//FOR

   int num_of_fusions = merge_vehicles(tracked_blobs);
   if (num_of_fusions > 0)
   {
     int antes = blob_temporal_id;
     blob_temporal_id -= num_of_fusions;
     printf("Fusion: %d --> %d\n",antes,blob_temporal_id);
   }
}

int FeatureTracker::merge_vehicles(vector<Blob*>& tracked_blobs)
{
   // Vehicle/Blob fusion:
   int num_of_fusions = 0;
   do
   {
      num_of_fusions = 0;
      for (unsigned int i=0; i<tracked_blobs.size(); i++)
      {
         if (tracked_blobs[i]->is_free())
         {
            continue;
         }

         Blob* first_blob = tracked_blobs[i];

         for (unsigned int j=0; j<tracked_blobs.size(); j++)
         {
            if ((i == j) || tracked_blobs[j]->is_free())
            {
               continue;
            }

            Blob* second_blob = tracked_blobs[j];
            float distance_3d = DISTANCE_3D_HPOINTS(second_blob->get_current_3d_center(), first_blob->get_current_3d_center());
            float distance_2d = DISTANCE_2D(second_blob->get_2d_center(), first_blob->get_2d_center());

            if ((distance_3d < 2))
            {
              printf("**** FUSION (3D) of vehiles %u (id=%d %1.f:%1.f:%1.f) and %u( id=%d %1.f:%1.f:%1.f) ***\n"
                     ,i
                     ,first_blob->get_id()
                     ,first_blob->get_current_3d_center().X
                     ,first_blob->get_current_3d_center().Y
                     ,first_blob->get_current_3d_center().Z
                     ,j
                     ,second_blob->get_id()
                     ,second_blob->get_current_3d_center().X
                     ,second_blob->get_current_3d_center().Y
                     ,second_blob->get_current_3d_center().Z);

               first_blob->join(*second_blob);
               second_blob->set_free(true);
               num_of_fusions++;
            }
            else if (distance_2d < 5)
            {
               printf("**** FUSION (2D) of vehiles %u (id=%d %1.f:%1.f:%1.f) and %u( id=%d %1.f:%1.f:%1.f) ***\n"
                      ,i
                      ,first_blob->get_id()
                      ,first_blob->get_current_3d_center().X
                      ,first_blob->get_current_3d_center().Y
                      ,first_blob->get_current_3d_center().Z
                      ,j
                      ,second_blob->get_id()
                      ,second_blob->get_current_3d_center().X
                      ,second_blob->get_current_3d_center().Y
                      ,second_blob->get_current_3d_center().Z);

               first_blob->join(*second_blob);
               second_blob->set_free(true);
               num_of_fusions++;
            }
         }
      }
   }
   while (num_of_fusions > 0);

   return num_of_fusions;
}

/**
 *
 */
Blob* FeatureTracker::find_blob(vector<Blob*>& blobs, int id)
{
   for(unsigned int i=0; i<blobs.size(); i++)
   {
      if (blobs[i]->get_id() == id)
      {
         return blobs[i];
      }
   }

   return NULL;
}

/**
 *
 */
bool FeatureTracker::vote_the_nearest_blob(vector<Blob*>& tmp_blobs, CvPoint2D32f* new_points, int num_points, Blob& tracked_blob)
{
   for (unsigned int j=0; j<tmp_blobs.size(); j++)
   {
      tmp_blobs[j]->votes = 0;
   }

   for(int i=0; i<num_points; i++)
   {
      for (unsigned int j=0; j<tmp_blobs.size(); j++)
      {
         if (tmp_blobs[j]->is_free())
         {
            continue;
         }

         if (PointInRectangle(new_points[i], tmp_blobs[j]->get_rect()))
         {
            tmp_blobs[j]->votes++;
         }
      }
   }

   int max_votes = 5;
   int idx = -1;
   for (unsigned int j=0; j<tmp_blobs.size(); j++)
   {
      if (tmp_blobs[j]->is_free())
      {
         continue;
      }

      if (tmp_blobs[j]->votes > max_votes)
      {
         max_votes = tmp_blobs[j]->votes;
         idx = j;
      }
   }

   if (idx > 0)
   {
      LOG("The blob with the highest votes is idx(%d) size(%d:%d) votes=%d\n"
          ,idx
          ,tmp_blobs[idx]->get_rect().width
          ,tmp_blobs[idx]->get_rect().height
          ,tmp_blobs[idx]->votes);

      tmp_blobs[idx]->set_matched(true);
      tmp_blobs[idx]->tracked_by(tracked_blob);
   }
}

/**
 *
 */
void FeatureTracker::get_good_features(IplImage* grey, Blob* blob)
{
   /** We are using just uniform distributed points. The CV goodFeaturesToTrack
    *  method doesn't work well. The first problem I found with this is to only
    *  get points over the vehicle. The mask we pass contains part of the road
    *  and this seems to cause some problems to the goodFeatures to track.
    */
   get_uniform_points(blob);
   return;

   // Note: now the below code is working, just give it a try

   // CvSize img_sz = cvGetSize( grey );
   // IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
   // IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
   // IplImage* mask = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);

   // CvRect rect = blob->get_rect();
   // LOG("Calculating good features [(%d:%d) (%d:%d)]..\n",rect.x, rect.y, rect.width, rect.height);
   // blob->init_points();

   // cvSetImageROI(mask, blob->get_rect());
   // cvSetImageROI(grey, blob->get_rect());
   // // cvAnd(grey, mask, grey, 0);

   // int corner_count = MAX_COUNT;
   // cvGoodFeaturesToTrack(grey,
   //                       eig_image,
   //                       tmp_image,
   //                       blob->points,
   //                       &corner_count,
   //                       0.1,
   //                       5.0,
   //                       NULL,
   //                       3,
   //                       0,
   //                       0.04);

   // LOG("-> found %d good features\n",corner_count);
   // blob->num_features = corner_count;

   // int delta_x = blob->get_rect().x;
   // int delta_y = blob->get_rect().y;
   // for(int i=0; i<blob->num_features; i++)
   // {
   //    blob->points[i].x += delta_x;
   //    blob->points[i].y += delta_y;
   //    LOG("%d -- (%.2f:%.2f)\n", i, blob->points[i].x, blob->points[i].y);
   // }

   // cvResetImageROI(grey);
   // cvResetImageROI(mask);
   // cvReleaseImage(&eig_image);
   // cvReleaseImage(&tmp_image);
}


bool FeatureTracker::match_vehicle_in_detection_zone(Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob)
{
   LOG(" > in detection zone \n");

   /****
    *** Detection zone:
    ***
    *** In this case we will use just the distance in order to match
    *** the blobs since we don't know the direction of the blob
    */
   float distance = MAX_DISTANCE;
   float tmp;
   int pos=-1;

   for (unsigned int j=0; j<num_blobs; j++)
   {
      if (not tmp_blobs[j]->already_matched())
      {
         tmp = DISTANCE_2D(blob->get_2d_center(), tmp_blobs[j]->get_2d_center());
         if (tmp<distance)
         {
            distance = tmp;
            pos = j;
         }
      }

   }//FOR

   if (pos>=0)
   {
      LOG("Matched with the blob %d \n",pos);
      *matched_blob = *tmp_blobs[pos];
      tmp_blobs[pos]->tracked_by(*blob);
      tmp_blobs[pos]->set_matched(true);

      return true;
   }
   else
   {
      return false;
   }
}

bool FeatureTracker::match_blob_using_klt(Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob)
{
   int count = MAX_COUNT;

   memset(new_points, 0, MAX_COUNT*sizeof(CvPoint2D32f));
   cvCalcOpticalFlowPyrLK(prev_grey,
                          grey,
                          prev_pyramid,
                          pyramid,
                          blob->points,
                          new_points,
                          blob->num_features,
                          cvSize(win_size,win_size),
                          5,
                          status,
                          0,
                          cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.1),
                          flags);

   flags |= CV_LKFLOW_PYR_A_READY;

   int k=0;
   int j=0;
   for( j = k = 0; j < blob->num_features; j++ )
   {
      if( add_remove_pt )
      {
         double dx = pt.x - new_points[j].x;
         double dy = pt.y - new_points[j].y;

         if( dx*dx + dy*dy <= 25 )
         {
            add_remove_pt = 0;
            continue;
         }
      }

      if( !status[j] )
      {
         new_points[k].x = 0;
         new_points[k].y = 0;
         k++;
         continue;
      }

      new_points[k++] = new_points[j];
   }

   count = k;

   LOG("-> Temporal update .. KLT result: blob (%d) new_count(%d)\n", blob->get_id(), count);

   if (count)
   {
      CvRect new_rect = get_bounding_rect(new_points, count);

      Blob tmp_blob;
      Tpoint2D center;
      center.x = new_rect.x+(new_rect.width/2);
      center.y = new_rect.y+(new_rect.height/2);
      vote_the_nearest_blob(tmp_blobs, new_points, count, *blob);

      LOG("new rect for blob (%d) is [(%d:%d) (%d:%d)]\n ",
          blob->get_id(),
          new_rect.x,
          new_rect.y,
          new_rect.width,
          new_rect.height);

      if (new_rect.x>0 && new_rect.y>0 && new_rect.width>0 && new_rect.height>0)
      {

         blob->init_points();

         if ((new_rect.x+new_rect.width) > image->width)
            new_rect.width -= (new_rect.x+new_rect.width)-image->width;

         if ((new_rect.y+new_rect.height) > image->height)
            new_rect.height -= (new_rect.y+new_rect.height)-image->height;

         blob->update_blob_box_center(new_rect);
         blob->track(new_rect);

         for(int idx=0; idx<count && idx<MAX_COUNT; idx++)
         {
            blob->points[idx] = new_points[idx];
         }

         LOG("Matching vehicle %d using KLT .. SUCCESS\n",blob->get_id());
         return true;
      }
   }

   LOG("Matching vehicle %d using KLT .. FAIL\n",blob->get_id());
   return false;
}

/**
 *
 */
bool FeatureTracker::match_blob(bool klt_only, Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob)
{
   Road* road = Road::Instance();

   if (road->in_detection_zone(blob->get_2d_center()))
   {
      return match_vehicle_in_detection_zone(blob, tmp_blobs, num_blobs, matched_blob);
   }
   else
   {
     if (klt_only)
       return match_blob_using_klt(blob, tmp_blobs, num_blobs, matched_blob);
     else
       return (match_blob_using_proximity(blob, tmp_blobs, num_blobs, matched_blob) || match_blob_using_klt(blob, tmp_blobs, num_blobs, matched_blob));
   }
}

/**
 *
 */
bool FeatureTracker::match_blob_using_proximity(Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob)
{
   float distance = MAX_DISTANCE;
   int pos=-1;
   Tpoint2D ec = blob->get_2d_center(); // Ellipse center
   Road* road = Road::Instance();

   for (unsigned int j=0; j<num_blobs; j++)
   {
      if (tmp_blobs[j]->is_free() || !road->in_tracking_zone(tmp_blobs[j]->get_2d_center()))
      {
         continue;
      }

      float inter_blob_2d_dis = DISTANCE_2D(blob->get_2d_center(), tmp_blobs[j]->get_2d_center());

      if (blob->IsBlobInProximityElipse(tmp_blobs[j]) && (inter_blob_2d_dis < distance))
      {
         distance = inter_blob_2d_dis;
         pos = j;
      }
   }//FOR

   if (pos >= 0)
   {
      LOG("Matching vehicle %d using proximity .. SUCCESS (blob  %d)\n", blob->get_id(), tmp_blobs[pos]->get_id());
      tmp_blobs[pos]->set_matched(true);
      tmp_blobs[pos]->tracked_by(*blob);
      return true;
   }
   else
   {
      LOG("Matching vehicle %d using proximity .. FAIL\n",blob->get_id());
      return false;
   }
}

/**
 *
 */
void FeatureTracker::get_uniform_points(Blob* blob){

   bool end = false;
   int points_cntr=0;

   int step_u = (abs(blob->get_left_corner().y - blob->get_right_corner().y))/(sqrt(MAX_COUNT));
   int step_v = (abs(blob->get_left_corner().x - blob->get_right_corner().x))/(sqrt(MAX_COUNT));

   /** Check for invalid values.*/
   step_u = (step_u == 0)?1:step_u;
   step_v = (step_v == 0)?1:step_v;

   for (int u=blob->get_left_corner().y; u<blob->get_right_corner().y && !end; u += step_u)
   {
      for (int v=blob->get_left_corner().x; v<blob->get_right_corner().x && !end; v += step_v)
      {
         Tpoint2D p;
         p.x = v;
         p.y = u;
         if (!MovModel::Instance()->movement_on(p))
            continue;

         CvPoint2D32f point;
         point.x = v;
         point.y = u;
         blob->points[points_cntr++] = point;
         end = (points_cntr == MAX_COUNT);
      }
   }

   blob->num_features = points_cntr;
}
}
