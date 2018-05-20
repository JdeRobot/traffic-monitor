/*
 *  Copyright (C) 2016 Kachach Redouane
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License v3
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *  Authors : Kachach Redouane <redouane.kachach@gmail.com>
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "movement_model.h"
#include "road_detection.h"
#include "background_model.h"
#include "classifier.h"
#include "image_utils.h"

namespace trafficmonitor{

#define IN_IMAGE(p, image) (p.x>=0 && p.x<image.cols && p.y>=0 && p.y<image.rows)
#define COORD_IN_IMAGE(i, j, image) (i>=0 && i<image.cols && j>=0 && j<image.rows)
#define EXPLORED(p) ((movement_matrix((int)p.y,(int)p.x) != MOVED))

/** Define the minimum number of pixels in a blob*/
#define DEFAULT_MIN_PIXELS_IN_BLOB 10
#define BLOBS_DEBUG 0
/*
 * Define the maximum number of pixels that could move in one iteration
 */
#define MAX_MOVED_PIXELS 20000

void print_image(IplImage * img)
{
   printf("img --> %d %d %d %d %d %d %d %d %d %d %d %p %d %p\n",
          img->nSize,
          img->ID,
          img->nChannels,
          img->alphaChannel,
          img->depth,
          img->dataOrder,
          img->origin,
          img->align,
          img->width,
          img->height,
          img->imageSize,
          img->imageData,
          img->widthStep,
          img->imageDataOrigin);
}

/**
 *
 */
MovModel::MovModel() { initialized=false; }

/**
 *
 */
Mat getStructElement(int dilation_type, int size)
{
  return getStructuringElement(dilation_type,
                               Size( 2*size + 1, 2*size+1 ),
                               Point( size, size ) );
}

/**
 *
 */
void MovModel::resize(unsigned int height_val, unsigned int width_val)
{
   if (not initialized)
   {
      printf("\nCreating the MovModel with the following dim (%d:%d)\n",height_val,width_val);

      height = height_val;
      width = width_val;

      /**  Reserve memory for the movement matrix */
      movement_matrix.create(height, width);
      m_refined_blob_fg.create(height, width, CV_8UC3);

      /**  Reset moved pixels */
      moved_pixels.resize(MAX_MOVED_PIXELS);
      for (unsigned int i = 0; i < moved_pixels.size(); i++)
      {
         moved_pixels[i].x = 0;
         moved_pixels[i].y = 0;
      }

      min_pixels_in_blob = DEFAULT_MIN_PIXELS_IN_BLOB;
      initialized = true;

      movement_matrix = {STATIC};
   }
}

/**
 *
 */
bool MovModel::movement_on(Tpoint2D p){

   unsigned int row = (int)p.y;
   unsigned int col = (int)p.x;

   Background* bg = Background::Instance();
   return bg->isFg(row,col);
}

bool MovModel::movement_on(int row, int col)
{
   Background* bg = Background::Instance();
   return bg->isFg(row,col);
}

/**
 *
 */
inline bool MovModel::movement_detected_on(const Tpoint2D& p){

   unsigned int row = (int)p.y;
   unsigned int col = (int)p.x;
   Background* bg = Background::Instance();

   if ((row<height) && (col<width))
   {
      return (bg->isFg(row,col) && (movement_matrix(row,col) == STATIC));
   }
   else
   {
      return false;
   }
}

inline bool MovModel::movement_detected_on(int row, int col)
{
   Background* bg = Background::Instance();

   if ((row<height) && (col<width))
   {
      return (bg->isFg(row,col) && (movement_matrix(row,col) == STATIC));
   }
   else
   {
      return false;
   }
}

void MovModel::get_neighbors(const Tpoint2D& p, list<Tpoint2D>& neighbors)
{
   int i = p.y;
   int j = p.x;

   neighbors.clear();

   if (movement_detected_on(i-1, j-1))
   {
      neighbors.push_front(Tpoint2D(i-1, j-1));
   }

   if (movement_detected_on(i, j-1))
   {
      neighbors.push_front(Tpoint2D(i, j-1));
   }

   if (movement_detected_on(i+1, j-1))
   {
      neighbors.push_front(Tpoint2D(i+1, j-1));
   }

   if (movement_detected_on(i-1, j))
   {
      neighbors.push_front(Tpoint2D(i-1, j));
   }

   if (movement_detected_on(i+1, j))
   {
      neighbors.push_front(Tpoint2D(i+1, j));
   }

   if (movement_detected_on(i-1, j+1))
   {
      neighbors.push_front(Tpoint2D(i-1, j+1));
   }


   if (movement_detected_on(i, j+1))
   {
      neighbors.push_front(Tpoint2D(i, j+1));
   }


   if (movement_detected_on(i+1, j+1))
   {
      neighbors.push_front(Tpoint2D(i+1, j+1));
   }
}

/**
 *
 */
void MovModel::get_neighbors(const Tpoint2D& p, list<Tpoint2D>& neighbors, Blob* curr_blob)
{
   int i = p.y;
   int j = p.x;

   neighbors.clear();

   if (movement_detected_on(i-1, j-1))
   {
      neighbors.push_front(Tpoint2D(i-1, j-1));
      movement_matrix(i-1, j-1) = curr_blob->get_id();
   }

   if (movement_detected_on(i, j-1))
   {
      neighbors.push_front(Tpoint2D(i, j-1));
      movement_matrix(i, j-1) = curr_blob->get_id();
   }

   if (movement_detected_on(i+1, j-1))
   {
      neighbors.push_front(Tpoint2D(i+1, j-1));
      movement_matrix(i+1, j-1) = curr_blob->get_id();
   }

   if (movement_detected_on(i-1, j))
   {
      neighbors.push_front(Tpoint2D(i-1, j));
      movement_matrix(i-1, j) = curr_blob->get_id();
   }

   if (movement_detected_on(i+1, j))
   {
      neighbors.push_front(Tpoint2D(i+1, j));
      movement_matrix(i+1, j) = curr_blob->get_id();
   }

   if (movement_detected_on(i-1, j+1))
   {
      neighbors.push_front(Tpoint2D(i-1, j+1));
      movement_matrix(i-1, j+1) = curr_blob->get_id();
   }


   if (movement_detected_on(i, j+1))
   {
      neighbors.push_front(Tpoint2D(i, j+1));
      movement_matrix(i, j+1) = curr_blob->get_id();
   }


   if (movement_detected_on(i+1, j+1))
   {
      neighbors.push_front(Tpoint2D(i+1, j+1));
      movement_matrix(i+1, j+1) = curr_blob->get_id();
   }
}

int MovModel::iterative_region_growing(std::vector<Blob*>& new_blobs, int idx)
{
   int blob_counter = 0;
   Blob* curr_blob = NULL;

   for (int i=0; i<idx; i++)
   {
      if (not movement_detected_on(moved_pixels[i]))
      {
         continue;
      }

      curr_blob = new_blobs[blob_counter];
      curr_blob->init();
      curr_blob->set_id(blob_counter);

      movement_matrix(moved_pixels[i].y, moved_pixels[i].x) = curr_blob->get_id();

      list<Tpoint2D> mylist;
      get_neighbors(moved_pixels[i], mylist);

      // Explore the region
      while (!mylist.empty())
      {
         // Add this point to the current region
         Tpoint2D curr_point = mylist.front();
         mylist.pop_front();
         curr_blob->insert_in_blob(curr_point);
         movement_matrix(curr_point.y,curr_point.x) = curr_blob->get_id();

         // Explore the point's neighbors
         list<Tpoint2D> neighbors;
         get_neighbors(curr_point, neighbors);

         if (!neighbors.empty())
         {
            mylist.splice(mylist.begin(), neighbors);
         }
      }

      curr_blob->update_blob_box_center();

      if (is_valid_blob(curr_blob))
      {
         blob_counter++;
      }
      else
      {
         curr_blob->init();
      }
   }

   blobs_fusion(new_blobs, blob_counter);

   return blob_counter;
}

/**
 *
 */
int MovModel::explore(int i, int j, Blob* blob)
{
   Road* road = Road::Instance();
   int output=0;
   Tpoint2D p;

   p.y = i;
   p.x = j;

   if (IN_IMAGE(p, movement_matrix) and not EXPLORED(p))
   {
      blob->insert_in_blob(p);
      movement_matrix(i,j) = blob->get_id();

      explore(i-1, j-1, blob);
      explore(i, j-1, blob);
      explore(i+1,j-1, blob);

      explore(i-1,j, blob);
      explore(i+1,j, blob);

      explore(i-1,j+1, blob);
      explore(i,j+1, blob);
      explore(i+1,j+1, blob);
      output=1;

   }
   else if (IN_IMAGE(p, movement_matrix)
            &&
            (movement_matrix((int)p.y,(int)p.x) != STATIC)
            &&
            (movement_matrix((int)p.y,(int)p.x) != MOVED))
   {
      output=0;
   }

   return output;
}

/**
 *
 */
bool MovModel::is_internal_point(Tpoint2D point, colorspaces::Image& new_frame)
{
   int i = point.y; //row
   int j = point.x; //col

   Background* bg = Background::Instance();

   return (COORD_IN_IMAGE(i-1, j-1, new_frame) && bg->isFg(i-1, j-1)
           && (COORD_IN_IMAGE(i, j-1, new_frame) && bg->isFg(i, j-1))
           && (COORD_IN_IMAGE(i+1,j-1, new_frame) && bg->isFg(i+1,j-1))
           && (COORD_IN_IMAGE(i-1,j, new_frame) && bg->isFg(i-1,j))
           && (COORD_IN_IMAGE(i+1,j, new_frame) && bg->isFg(i+1,j))
           && (COORD_IN_IMAGE(i-1,j+1, new_frame) && bg->isFg(i-1,j+1))
           && (COORD_IN_IMAGE(i,j+1, new_frame) && bg->isFg(i,j+1)));
}

/**
 *
 */
int MovModel::opt_growing(colorspaces::Image& new_frame, Rect roi, std::vector<Blob*>& new_blobs)
{
   int blob_counter = 0;

   for(unsigned int i=0; i<new_blobs.capacity(); i++)
   {
      new_blobs[i]->init();
   }

   /** build the movement matrix: For each pixel we will determine
    ** if there was a movement or not. If so we mark its "i,j" position
    ** as TRUE in the movement matrix and we add a new points to the
    ** the moved_pixels array.
    **/
   for (unsigned int i=roi.y; i < roi.y+roi.height && blob_counter<new_blobs.size(); i++)
   {
     for(unsigned int j=roi.x; j < roi.x+roi.width && blob_counter<new_blobs.size(); j++)
      {
         if (movement_detected_on(i,j))
         {
            Blob* curr_blob = new_blobs[blob_counter];
            curr_blob->init();
            curr_blob->set_id(blob_counter+1);

            movement_matrix(i,j) = curr_blob->get_id();

            if (BLOBS_DEBUG)
            {
               printf("Creating blob %d on pos %d \n", curr_blob->get_id(), blob_counter);
            }

            Tpoint2D p = Tpoint2D(i,j);
            curr_blob->insert_in_blob(p);

            list<Tpoint2D> mylist;
            get_neighbors(p, mylist, curr_blob);

            // Explore the current region
            while (!mylist.empty())
            {
               // Add this point to the current region
               Tpoint2D curr_point = mylist.front();
               mylist.pop_front();
               curr_blob->insert_in_blob(curr_point);
               if (IN_IMAGE(curr_point, new_frame) && !is_internal_point(curr_point, new_frame))
               {
                  if (curr_blob->contour.size() < MAX_CONTOUR_SIZE)
                  {
                     curr_blob->contour.push_back(Point(curr_point.x, curr_point.y));
                  }
               }

               movement_matrix(curr_point.y, curr_point.x) = curr_blob->get_id();

               // Explore the point's neighbors
               list<Tpoint2D> neighbors;
               get_neighbors(curr_point, neighbors, curr_blob);

               if (!neighbors.empty())
               {
                  mylist.splice(mylist.begin(), neighbors);
               }
            }

            curr_blob->update_blob_box_center();

            if (is_valid_blob(curr_blob))
            {
               if (BLOBS_DEBUG)
               {
                  printf("Valid blob lc(%d:%d) rc(%d:%d) size(%d:%d) center(%d:%d) num_points=%d curr_min=%d blob_count=%d\n",
                         curr_blob->get_left_corner().x,
                         curr_blob->get_left_corner().y,
                         curr_blob->get_right_corner().x,
                         curr_blob->get_right_corner().y,
                         curr_blob->get_rect().height,
                         curr_blob->get_rect().width,
                         curr_blob->get_2d_center().x,
                         curr_blob->get_2d_center().y,
                         curr_blob->get_num_points(),
                         min_pixels_in_blob,
                         blob_counter);
               }

               blob_counter++;
            }
            else
            {
               if (BLOBS_DEBUG)
               {
                  printf("Blob discarded: %d num_points=%d\n", curr_blob->get_id(), curr_blob->get_num_points());
               }
               curr_blob->init();
            }
         }
      }
   }

   return blob_counter;
}

/**
 *
 */
void MovModel::build_movement_matrix(colorspaces::Image& new_frame, int* max_idx){

   Background* bg = Background::Instance();
   int idx=0;
   unsigned char * image = (unsigned char *)new_frame.data;
   Tpoint2D roi_lc,roi_rc;
   Road* road = Road::Instance();
   roi_lc.x = 0;
   roi_lc.y = 0;
   roi_rc.x = new_frame.width;
   roi_rc.y = new_frame.height;
   road->get_roi(&roi_lc, &roi_rc);

   dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));
   erode(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 3));
   dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));

   /** build the movement matrix: For each pixel we will determine
    ** if there was a movement or not. If so we mark its "i,j" position
    ** as TRUE in the movement matrix and we add a new points to the
    ** the moved_pixels array.
    **/
   bool end=false;
   for (unsigned int row=roi_lc.y; row<roi_rc.y && !end; row++)
   {
      for(unsigned int col=roi_lc.x; col<roi_rc.x && !end; col++)
      {
         if (bg->isFg(row,col))
         {
            // Movement
            movement_matrix(row,col) = MOVED;
            moved_pixels[idx].y = row;
            moved_pixels[idx].x = col;
            idx++;

            // check if the maximum of pixels storage is reached
            end = (idx == MAX_MOVED_PIXELS);

            if (end)
               printf("*********** reached max_pixels\n");
         }
         else
         {
            movement_matrix(row,col) = STATIC;
         }
      }
   }

   *max_idx = (idx<MAX_MOVED_PIXELS)?idx:(MAX_MOVED_PIXELS-1);
}

/**
 *
 */
bool SortbyXaxis(const Point & a, const Point &b)
{
   return (a.x < b.x);
}

/**
 *
 */
bool SortbyYaxis(const Point & a, const Point &b)
{
   return (a.y < b.y);
}

/**
 *
 */
bool MovModel::pre_process_blobs(colorspaces::Image& new_frame, std::vector<Blob*>& new_blobs)
{
   dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));
   erode(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 2));
   dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));

   m_refined_zones.clear();

   /** Init blobs blobs*/
   movement_matrix = {STATIC};
   opt_growing(new_frame, Road::Instance()->get_roi(), new_blobs);

   bool rescan = false;
   Mat fg_img(new_frame);
   Mat bg_mask(Background::Instance()->GetMask());

   for (int i=0; i<new_blobs.size() ; i++)
   {
      if (new_blobs[i]->is_free())
         continue;

      double w,h;
      new_blobs[i]->get_metric_size(w,h);
      if (w > TM_MAX_BLOB_WIDTH)
      {
         // An abnormal blob is found. It could be the result of a blob plus its shadow or the result
         // of several blobs being occluded because of the view angle or their shadows. In both cases
         // we will try to refine the resulting "big" blob to find the involved vehiles. This operation
         // ise based on several morphological operators. In order to avoid affecting the original background
         // image (because not all the blobs are involved in the occlusion/shadows) we apply the refine on
         // the blob only. This is done by copying the blob (using its contour information) to another temporal
         // buffer where the refine operation is perfomred. If we don't do that there's a risk of losing the
         // background information for e.g an erode operation may convert a car into a motorcycle.

         // The refine operation result is used to update the original background so we "discover" the
         // involved blobs by removing the shadows from this "big" blob

         rescan = true;
         Tpoint2D lc = new_blobs[i]->get_left_corner();
         Tpoint2D rc = new_blobs[i]->get_right_corner();
         int blob_width = (rc.x-lc.x);
         int blob_height = (rc.y-lc.y);

         Rect roi = Rect(lc.x, lc.y, blob_width, blob_height);
         Mat mask = Mat::zeros(fg_img.size(), CV_8UC1);

         // OpenCV drawContours needs the contour points to be sorted in the following
         // order to work correctly otherwise the draw operatoin doens't work as expected
         sort( new_blobs[i]->contour.begin(), new_blobs[i]->contour.end(), SortbyXaxis );
         sort( new_blobs[i]->contour.begin(), new_blobs[i]->contour.end(), SortbyYaxis );

         vector<vector<Point>> blob_contour;
         blob_contour.push_back(new_blobs[i]->contour);

         // mask is a black image where we "copy" the blob to be refined only
         drawContours(mask, blob_contour, -1, Scalar(255), CV_FILLED);

         // Extract the blob region using the "mask"
         Mat blobArea;
         Mat bgROI;
         fg_img.copyTo(blobArea, mask); // contour roi that contains the foreground image of the blob
         bg_mask.copyTo(bgROI, mask);   // contour roi that contians the background image of the blob

         // The blob to be refined is ready on the separate image, let's refine it
         refine_blobs(fg_img, blobArea, bgROI, roi, mask);

         // Record the refined  zone (it may contains 1 or more blobs as consequence of the refine operation)
         // This information is used later to discard small blobs generated by the 'refine' operation
         m_refined_zones.push_back(roi);
      }
   }

   return rescan;
}

/**
 *
 */
bool MovModel::refine_blobs(Mat image, Mat blob_area, Mat bg_roi, Rect roi, Mat mask)
{
   Background* bg = Background::Instance();
   ImageUtils image_utils;

   // First we apply a canny edge detector to get the sharp edges of the blob (colour foreground)
   image_utils.MyCanny(blob_area, m_refined_blob_fg);

   int erosion_size = 4;
   Mat element = getStructuringElement(MORPH_ELLIPSE,
                                       Size( 2*erosion_size+1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

   // We do a "closing" operation by using a big element to "enlarge" the edges as much
   // as we can the size 4 has been choosen after several experiments. Starting from this size
   // a bluring effect happens so it's not recommended to increase the size for 320x240 images.
   // As consequence of this operation the blob "body" is filled (because it contains more edges
   // than shadow) and the shadow contour is eliminated completly or partially since it doesn't
   // contain edges
   dilate(m_refined_blob_fg, m_refined_blob_fg, element);
   erode(m_refined_blob_fg, m_refined_blob_fg, element);

   // After the above operation it's possible to find some broken lines that belong to the body
   // but they were broken du to the closing operation. The idea behind the folloing thin dilate
   // operation is to join this lines, this way we will continue building the blob "body" by filling
   // the holes
   erosion_size = 1;
   element = getStructuringElement(MORPH_ELLIPSE,
                                   Size( 2*erosion_size+1, 2*erosion_size+1 ),
                                   Point( erosion_size, erosion_size ) );
   dilate(m_refined_blob_fg, m_refined_blob_fg, element);

   Mat holes;
   vector<vector<Point> > blob_contour;
   vector<Vec4i> hierarchy;

   // The blob body is mostly build but it may contain still some holes specially on the zones
   // with uniform colour (the rouf of white truck for e.g). With the following operation we
   // fill these holes by drawing them
   cv::cvtColor(m_refined_blob_fg, m_refined_blob_bg, CV_BGR2GRAY);
   m_refined_blob_bg.copyTo(holes);

   // find and fill holes
   findContours(holes, blob_contour, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
   for(vector<Vec4i>::size_type idx=0; idx<hierarchy.size(); ++idx)
   {
      Scalar color = Scalar( 255 );

      if(hierarchy[idx][3] != -1)
         drawContours(holes, blob_contour, idx, Scalar::all(255), CV_FILLED, 8, hierarchy);
   }

   // Build the body of the blob by joining the holes + its already built body
   bitwise_or(m_refined_blob_bg, holes, m_refined_blob_bg, mask);

   // Some times at the end of this process there're still some thin lines consequence of the
   // partially removed shadow blob_contour. A thin erosion is launched to remove these lines
   erosion_size = 1;
   element = getStructuringElement(MORPH_ELLIPSE,
                                   Size( 2*erosion_size+1, 2*erosion_size+1 ),
                                   Point( erosion_size, erosion_size ) );
   erode(m_refined_blob_bg, m_refined_blob_bg, element);

   // At this point we have build the blob "body" by using its edge information and several
   // morphological operations. It's time to update the blob bg area by this information.
   // Basically we remove from the background the black zones resulting from the refine
   // operation so we keep only the blob "body". This operation is equivalent to:
   // new_background = orig_background & refined_blob_grey
   for(unsigned int row=roi.y; row < roi.y+roi.height; row++) //rows
   {
      for (unsigned int col=roi.x; col < roi.x+roi.width; col++) //columns
      {
         if (mask.at<uint8_t>(row,col) == 0)
         {
            // Skip zones where no movement have been detected (TODO: check if is this equivalent to !isFg(i,j))
            continue;
         }

         if ((m_refined_blob_bg.at<uint8_t>(row,col) > 0) && bg->isFg(row,col))
         {
            bg_roi.at<uint8_t>(row,col) = 255;
         }
         else
         {
            bg_roi.at<uint8_t>(row,col) = 0;
         }
      }
   }

   // Now the background is refined with the blob body, this may leave
   // some thin lines let's remove them by a final erosion operation
   // TODO: is this realy needed?
   erosion_size = 1;
   element = getStructuringElement(MORPH_ELLIPSE,
                                   Size( 2*erosion_size+1, 2*erosion_size+1 ),
                                   Point( erosion_size, erosion_size ) );
   erode(bg_roi, bg_roi, element);

   // Merge back the refined background to the original background
   cv::Mat& bg_mask = Background::Instance()->GetMask();
   bitwise_and(bg_mask, bg_roi, bg_mask, mask);
}

/**
 *
 */
int MovModel::find_blobs(colorspaces::Image& new_frame, std::vector<Blob*>& new_blobs, bool advanced_detection)
{
   if (!initialized)
   {
      //TODO: log error
      return 0;
   }


   /** Init blobs blobs*/
   for(unsigned int i=0; i<new_blobs.capacity(); i++)
   {
      new_blobs[i]->init();
   }

   if (advanced_detection)
   {
      pre_process_blobs(new_frame, new_blobs);

      // the pre_process_blobs contains already an opt_growing call, but since this operation may generate
      // "new" blobs (as result of removing the shadows) we launch newly the opt_growing to find all the
      // generated blobs
      // TODO: this second operation could be avoided by serching the new blobs in the refined zones only
      movement_matrix = {STATIC};
      opt_growing(new_frame, Road::Instance()->get_roi(), new_blobs);
   }
   else
   {
     // Simple pre-processing by "closing" and then dilating the blobs to remove noise and very small blobs
     dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));
     erode(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 2));
     dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));

      movement_matrix = {STATIC};
      opt_growing(new_frame, Road::Instance()->get_roi(), new_blobs);
   }
}

/**
 *
 */
bool MovModel::is_refined_blob(Blob* blob)
{
   for (int i=0; i<m_refined_zones.size(); i++)
   {
      Tpoint2D center = blob->get_2d_center();
      Point p;
      p.x = center.x;
      p.y = center.y;

      if (m_refined_zones[i].contains(p))
      {
         return true;
      }
   }

   return false;
}

/**
 *
 */
bool MovModel::is_valid_blob(Blob* blob)
{
   Road* road = Road::Instance();

   if (road && blob)
   {
      Tpoint2D lcu = blob->get_left_corner();
      Tpoint2D rcd = blob->get_right_corner();
      Tpoint2D lcd, rcu;

      lcd.x = lcu.x;
      lcd.y = rcd.y;
      rcu.y = lcu.y;
      rcu.x = rcd.x;

      if (road->is_road(lcu.y,lcu.x) ||
          road->is_road(lcd.y,lcd.x) ||
          road->is_road(rcu.y,rcu.x) ||
          road->is_road(rcd.y,rcd.x))
      {
         if (is_refined_blob(blob))
         {
            /**
             * We discard all the blobs equal or below a motorcycle size in refined blobs. We do this because after removing
             * the shadows there's a high probablity of generating small blobs which are consequence of the pre-processing operations.
             */
            float min_length=0;
            unsigned int min_pixels_in_blob=0;
            Classifier::get_smallest_vehicle_dimension(&min_pixels_in_blob, &min_length, blob->get_2d_center());
            return (min_pixels_in_blob > 0) && (blob->get_num_points() >= min_pixels_in_blob);
         }
         else
         {
            double denstiy = (blob->get_num_points() / blob->get_area());
            if (denstiy > 0.4)
            {
               float min_length=0;
               unsigned int min_pixels_in_blob=0;
               Classifier::get_smallest_vehicle_dimension(&min_pixels_in_blob, &min_length, blob->get_2d_center());

               return ((min_pixels_in_blob > 0)
                       && (blob->get_num_points() > (min_pixels_in_blob*0.6))
                       && (road->in_road(blob->get_2d_center())));
            }
         }
      }
      else if (BLOBS_DEBUG)
      {
        printf("** Blob id=%d outside road lcu(%d:%d) lc(%d:%d) rcu(%d:%d) rcd(%d:%d)\n"
               ,blob->get_id()
               ,lcu.y,lcu.x
               ,lcd.y,lcd.x
               ,rcu.y,rcu.x
               ,rcd.y,rcd.x);
      }

   }

   return false;
}

/**
 *
 */
int MovModel::recursive_growing_region(std::vector<Blob*>& new_blobs, int idx)
{
   int i=0;
   unsigned int max_blobs = new_blobs.capacity();
   unsigned int blob_counter=0;
   Tsegment tmp_finish_line;
   Road* road = Road::Instance();
   Blob* curr_blob = NULL;

   /* Once the movement matrix is built we will segment the detected points. A recursive
    * growing region algorithm is used for this purpose. The goal of this step is to segment
    * the moved pixels into a set of blobs.
    */
   for (i=0; i<idx; i++)
   {
      if (not movement_on(moved_pixels[i]))
         continue;

      curr_blob = new_blobs[blob_counter];
      curr_blob->init();
      curr_blob->set_id(blob_counter);

      if (explore(moved_pixels[i].y, moved_pixels[i].x, curr_blob))
      {
         curr_blob->update_blob_box_center();

         if (is_valid_blob(curr_blob))
         {
            if (BLOBS_DEBUG)
            {
               float min_length=0;
               unsigned int min_pixels_in_blob=0;
               Classifier::get_smallest_vehicle_dimension(&min_pixels_in_blob, &min_length, curr_blob->get_2d_center());

               printf("blob lc(%d:%d) rc(%d:%d) center(%d:%d) num_points=%d curr_min=%d\n",
                      curr_blob->get_left_corner().x,
                      curr_blob->get_left_corner().y,
                      curr_blob->get_right_corner().x,
                      curr_blob->get_right_corner().y,
                      curr_blob->get_2d_center().x,
                      curr_blob->get_2d_center().y,
                      curr_blob->get_num_points(),
                      min_pixels_in_blob);
            }

            blob_counter++;
         }

         /** If there's more moved objects than the max tmp blobs we can store(first check) or
          *  the region labels have ben exhausted (second check) then then we will just stop the
          *  region growing algorithm and process the detected blobs. That way we also avoid the case
          *  when there's a lot of noise is present in the scenario.
          */
         if (blob_counter == max_blobs || blob_counter == MOVED)
            break;
      }
   }

   /** Join the near blobs*/
   blobs_fusion(new_blobs, blob_counter);

   return blob_counter;
}

/**
 *
 */
void MovModel::optContour(Blob* blob){

   Tpoint2D original_center = blob->get_2d_center();

   /** Copy the points to a temporal vector.*/
   vector<Point2f> input_points(blob->contour.size());
   for(unsigned int m=0; m<blob->contour.size(); m++)
   {
      input_points[m].x = blob->contour[m].x;
      input_points[m].y = blob->contour[m].y;
   }

   cv::Mat_<Point2f> tmp_mat(input_points);
   cv::Mat_<Point2f> tmp_mat2(input_points);

   CvMat cvmat1 = tmp_mat;
   CvMat cvmat2 = tmp_mat;

   cvConvertScale(&cvmat1, &cvmat2, 0.7, 0);

   vector<Point2f> approxCurve (tmp_mat2);

   printf("Antes\n");
   for(unsigned int m=0; m<blob->contour.size(); m++)
   {
      printf("(%d:%d)\n",blob->contour[m].x,blob->contour[m].y);
   }

   blob->contour.clear();
   blob->contour.resize(approxCurve.size());
   for(unsigned int m=0; m<approxCurve.size(); m++)
   {
      blob->contour[m].x = approxCurve[m].x;
      blob->contour[m].y = approxCurve[m].y;
   }

   blob->calculate_blob_center();

   int shift_x = original_center.x-blob->get_2d_center().x;
   int shift_y = original_center.y-blob->get_2d_center().y;

   for(unsigned int m=0; m<approxCurve.size(); m++)
   {
      blob->contour[m].x += shift_x;
      blob->contour[m].y += shift_y;
   }
}

/**
 *
 */
void MovModel::blobs_fusion(std::vector<Blob*>& blobs, int num_blobs){

   int i,j;
   float dis = 0;
   Road* road = Road::Instance();

   for (i=0; i<num_blobs; i++)
   {
      if (blobs[i]->is_free())
         continue;

      for (j=0; j<num_blobs; j++)
      {
         if (i == j || blobs[j]->is_free())
            continue;

         /** Solo fusionamos grupos en la zona de detección*/
         // if (!road->in_detection_zone(blobs[i]->get_2d_center())
         //     ||
         //     !road->in_detection_zone(blobs[j]->get_2d_center()))
         //    continue;

         dis = DISTANCE_2D(blobs[i]->get_2d_center(), blobs[j]->get_2d_center());

         if (dis < blobs[i]->get_radius()+blobs[j]->get_radius())
         {
            blobs[i]->join(*blobs[j]);
            printf("merging: dis=%f r1=%f r2=%f\n",dis, blobs[i]->get_radius(), blobs[j]->get_radius());
            blobs[j]->init();
         }
      }
   }
}


int MovModel::FindBlobs(const cv::Mat &binary, std::vector<Blob*>& new_blobs)
{
   Tpoint2D roi_lc,roi_rc;
   Road* road = Road::Instance();
   road->get_roi(&roi_lc, &roi_rc);

   dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));
   erode(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 2));
   dilate(Background::Instance()->GetMask(), Background::Instance()->GetMask(), getStructElement(MORPH_RECT, 1));

   // Fill the label_image with the blobs
   // 0  - background
   // 1  - unlabelled foreground
   // 2+ - labelled foreground

   cv::Mat label_image(binary);
   binary.convertTo(label_image, CV_32FC1); // weird it doesn't support CV_32S!

   // Init blobs blobs
   for(unsigned int i=0; i<new_blobs.capacity(); i++)
   {
      new_blobs[i]->init();
   }

   int label_count = 2; // starts at 2 because 0,1 are used already
   int blob_counter = 0;
   Blob* curr_blob = NULL;

   // for(int y=0; y < binary.rows; y++)
   // {
   //    for(int x=0; x < binary.cols; x++)
   //    {


   for (unsigned int y=roi_lc.y; y<roi_rc.y; y++)
   {
      for(unsigned int x=roi_lc.x; x<roi_rc.x; x++)
      {

         if((int)label_image.at<float>(y,x) != 255)
         {
            continue;
         }

         curr_blob = new_blobs[blob_counter];
         curr_blob->init();
         curr_blob->set_id(blob_counter+1);

         cv::Rect rect;
         cv::floodFill(label_image,
                       cv::Point(x,y),
                       cv::Scalar(label_count),
                       &rect,
                       cv::Scalar(0),
                       cv::Scalar(0),
                       8);

         for(int i=rect.y; i < (rect.y+rect.height); i++)
         {
            for(int j=rect.x; j < (rect.x+rect.width); j++)
            {
               if((int)label_image.at<float>(i,j) != label_count)
               {
                  continue;
               }
               curr_blob->insert_in_blob(Tpoint2D(i,j));
            }
         }
         curr_blob->update_blob_box_center();

         // curr_blob->update_blob_box_center(rect, (rect.width*rect.height));

         if (is_valid_blob(curr_blob))
         {
            blob_counter++;
         }

         label_count++;
      }
   }

   // printf("label count %d\n",label_count);

   return blob_counter;
}

}
