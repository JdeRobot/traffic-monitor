#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "math.h"
#include <sys/types.h>
#include <opencv/cv.h>

#include "pixeltrack_tracking.h"
#include "road_detection.h"
#include "movement_model.h"
#include "planar_geom.h"

#define MIN_ANGLE 90
#define MAX_DISTANCE 10000

namespace trafficmonitor{

#define DEBUG 1
#define LOG(args...){if (DEBUG) printf(args);}
   
#define MAX_NEW_BLOBS 10
#define MAX_TEMPORAL_ID 1000
#define DEFAULT_MIN_SEP 10
#define MIN_DISTANCE 10

/**
 *
 */
PixelShiftTracker::PixelShiftTracker(){blob_temporal_id=1;}

/**
 *
 */
void PixelShiftTracker::init()
{
   blob_temporal_id=1;
   mov_direction.a = 0;
   mov_direction.b = 0;
}

/**
 *
 */
void PixelShiftTracker::track_blobs(colorspaces::Image& new_frame
                                    ,timeval timestamp
                                    ,vector<Blob*>& tracked_blobs
                                    ,vector<Blob*>& tmp_blobs
                                    ,unsigned int num_blobs)
{
   // cvtColor(new_frame, new_frame, CV_RGB2BGR);
   Image<unsigned char>* current_frame = new Image<unsigned char>(new_frame.width
                                                                  ,new_frame.height
                                                                  ,new_frame.channels()
                                                                  ,new_frame.step
                                                                  ,(unsigned char*)new_frame.data
                                                                  ,true);
   /** Init tracked blobs*/
   for(unsigned int i=0; i<tracked_blobs.size(); i++)
   {
      tracked_blobs[i]->set_matched(false);
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

      LOG("Matching blob (%d) \n",blob->get_id());

      if (match_blob(current_frame, blob, tmp_blobs, num_blobs))
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

   process_detected_blobs(current_frame, timestamp, tracked_blobs, tmp_blobs, num_blobs);

   delete current_frame;
}

/**
 *
 */
Blob* PixelShiftTracker::find_blob(vector<Blob*>& blobs, int id)
{
   for(unsigned int i=0; i<blobs.size(); i++)
   {
      if (blobs[i]->get_id() == id)
      {
         LOG("Blob found on pos %d\n",i);
         return blobs[i];
      }
   }

   return NULL;
}

bool PixelShiftTracker::match_vehicle_in_detection_zone(Image<unsigned char>* new_frame
                                                        ,Blob* blob
                                                        ,vector<Blob*>& tmp_blobs
                                                        ,unsigned int num_blobs)
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
      blob->track(*tmp_blobs[pos]);
      tmp_blobs[pos]->set_matched(true);
      return true;
   }
   else
   {
      return false;
   }
}   

/**
 *
 */
bool PixelShiftTracker::match_blob_using_pixeltrack(Image<unsigned char>* new_frame
                                                    ,vector<Blob*>& tmp_blobs
                                                    ,Blob* blob)
{
   if (blob)
   {

      CvRect tmp = blob->get_rect();
      tmp.width = tmp.width/2;
      tmp.height = tmp.height/2;
      tmp.x += tmp.width/2;
      tmp.y += tmp.height/2;

      if (blob->m_tracker)
      {
         CvRect newBbox = blob->m_tracker->Track(new_frame, tmp);
         Tpoint2D center;
         center.x = newBbox.x + (newBbox.width)/2;
         center.y = newBbox.y + (newBbox.height)/2;

         LOG("The pixel track new BBOX is  bbox(%d,%d - %d,%d)\n"
             ,newBbox.x
             ,newBbox.y
             ,newBbox.width
             ,newBbox.height);
      
         CvRect kk;
         blob->m_expectedRect = newBbox;
         update_the_nearest_blob(tmp_blobs, center, *blob, kk);
         return true;
      }
   }

   return false;
}

/**
 *
 */
bool PixelShiftTracker::match_blob(Image<unsigned char>* new_frame
                                   ,Blob* blob
                                   ,vector<Blob*>& tmp_blobs
                                   ,unsigned int num_blobs)
{
   Road* road = Road::Instance();

   if (road->in_detection_zone(blob->get_2d_center()))
   {
      return match_vehicle_in_detection_zone(new_frame, blob, tmp_blobs, num_blobs);
   }
   else 
   {
      return match_blob_using_pixeltrack(new_frame, tmp_blobs, blob);
   }
}

/**
 *
 */
void PixelShiftTracker::process_detected_blobs(Image<unsigned char>* new_frame
                                               ,timeval timestamp
                                               ,vector<Blob*>& tracked_blobs
                                               ,vector<Blob*>& tmp_blobs
                                               ,unsigned int num_blobs)
{
   Road* road = Road::Instance();
   unsigned char free_positions[MAX_NEW_BLOBS];
   memset(free_positions, 0, sizeof(free_positions));
   int k=0;

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
      if (!tmp_blobs[i]->already_matched() and road->in_detection_zone(tmp_blobs[i]->get_2d_center()))
      {
         unsigned int free_pos = free_positions[k++];
         if (free_pos <tracked_blobs.size())
         {
            //TODO: comment
            PixelTrack* tracker = tracked_blobs[free_pos]->m_tracker;
            *tracked_blobs[free_pos] = *tmp_blobs[i];
            tracked_blobs[free_pos]->m_tracker = tracker;
            tracked_blobs[free_pos]->set_id(blob_temporal_id++ % MAX_TEMPORAL_ID);
            tracked_blobs[free_pos]->start_tracking(timestamp);

            Blob* blob = tracked_blobs[free_pos];
               
            LOG("> new vehicle (%d) tracked=%s stored on pos=%d 2dcenter(%d:%d) 3dcenter(%1.f:%1.f:%1.f) in_timestamp=%u\n\n",
                tracked_blobs[free_pos]->get_id(),
                tracked_blobs[free_pos]->is_being_tracked()?"true":"false",
                free_pos,
                tracked_blobs[free_pos]->get_2d_center().y,
                tracked_blobs[free_pos]->get_2d_center().x,
                tracked_blobs[free_pos]->get_first_3d_center().X,
                tracked_blobs[free_pos]->get_first_3d_center().Y,
                tracked_blobs[free_pos]->get_first_3d_center().Z,
                (unsigned int)tracked_blobs[free_pos]->get_ingress_timestamp().tv_usec);
         }
         else
         {
            LOG("Trying to store the new blob in a non-valid pos %d\n",free_pos);
         }
      }
      else if (tmp_blobs[i]->already_matched())
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
               blob->m_tracker->ProcessFirstFrame(new_frame, blob->get_rect());
            }
            else
            {
               LOG("BUG: cant find the blob -> %d\n", tmp_blobs[i]->tracking_blobs_id[0]);
            }
         }
         else if (tmp_blobs[i]->tracking_blobs_id.size() > 1)
         {
            LOG("\n tmp_blob %d has %d followers \n", i, tmp_blobs[i]->tracking_blobs_id.size());
               
            for(unsigned int m=0; m<tmp_blobs[i]->tracking_blobs_id.size(); m++)
            {
               Blob* blob = find_blob(tracked_blobs, tmp_blobs[i]->tracking_blobs_id[m]);
               if (blob)
               {
                  LOG(" Followed by blob %d (now is ocluded!)\n",blob->get_id());
                  blob->ocluded = true;
                  blob->track(blob->m_expectedRect);
                  // blob->m_tracker->ProcessFirstFrame(new_frame, blob->m_expectedRect);
               }
               else
               {
                  LOG("BUG: cant find the blob -> %d\n", tmp_blobs[i]->tracking_blobs_id[m]);
               }
            }
         }
      }//IF
         
   }//FOR
}

/**
 *
 */
bool PixelShiftTracker::update_the_nearest_blob(vector<Blob*>& tmp_blobs, Tpoint2D p2, Blob& tracked_blob, CvRect& output_rect)
{
   int blob_idx=-1;
   float tmp;
   float distance = INT_MAX;
   bool success=false;
   Tvector vtmp_1, vtmp_2;

   /* In this case we know the blob direction so we can use it in order to select the better match.
    * That's the nearest blob that maintains the movement direction
    */
   get_vector(&vtmp_1, tracked_blob.get_first_3d_center(), tracked_blob.get_current_3d_center());

   LOG("  Tracked blob vector (%.2f:%.2f) --> (%.2f:%.2f)\n",
       tracked_blob.get_prev_3d_center().X,
       tracked_blob.get_prev_3d_center().Y,
       tracked_blob.get_current_3d_center().X,
       tracked_blob.get_current_3d_center().Y);

   for (unsigned int j=0; j<tmp_blobs.size(); j++)
   {
      if (tmp_blobs[j]->is_free())
         continue;

      tmp_blobs[j]->update_3d_center();
         
      // tmp = DISTANCE_2D(tracked_blob.get_predicted_2d_center(), tmp_blobs[j]->get_2d_center());
      tmp = DISTANCE_2D(p2, tmp_blobs[j]->get_2d_center());

      if (tmp < distance)
      {
         /** We only calculate the distance if the detected movement has the 
          *  the same direction as the blob we are checking.
          */
         get_vector(&vtmp_2, tracked_blob.get_first_3d_center(), tmp_blobs[j]->get_current_3d_center());

         LOG("  trying the tmp_blob pos(%d)  (%d,%d) numpoint=%d (%.2f:%.2f) dis=%.2f\n",
             j,
             tmp_blobs[j]->get_2d_center().y,
             tmp_blobs[j]->get_2d_center().x,
             tmp_blobs[j]->get_num_points(),
             tmp_blobs[j]->get_current_3d_center().X,
             tmp_blobs[j]->get_current_3d_center().Y,
             tmp);
                
         if (angle_vectors(vtmp_1, vtmp_2) <= (MIN_ANGLE*3.14/180))
         {
            LOG("  OK: angle_vectors is %.2f  -- min=%.2f \n", angle_vectors(vtmp_1, vtmp_2), (MIN_ANGLE*3.14/180));
            distance = tmp;
            blob_idx = j;
            success=true;
         }
         else
         {
            LOG("  angle_vectors is %.2f  -- min=%.2f \n", angle_vectors(vtmp_1, vtmp_2), (MIN_ANGLE*3.14/180));
            LOG("  blob.y=%d  tracked.y=%d\n", tmp_blobs[j]->get_2d_center().y, tracked_blob.get_2d_center().y);
         }
      }
   }//FOR

   if (success)
   {
      CvRect bbox = tmp_blobs[blob_idx]->get_rect();
      LOG("The nearst tmp blob is idx(%d) bbox(%d,%d - %d,%d)\n"
          ,blob_idx
          ,bbox.x
          ,bbox.y
          ,bbox.width
          ,bbox.height);
         
      tmp_blobs[blob_idx]->set_matched(true);
      tmp_blobs[blob_idx]->tracked_by(tracked_blob);
      // tracked_blob.track(*tmp_blobs[blob_idx]);
      output_rect = tmp_blobs[blob_idx]->get_rect();
   }

   return success;
}
}
