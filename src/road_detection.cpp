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
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#include <limits>

#include "road_detection.h"
#include "camera_model.h"

using namespace std;

/*****************************************************************************/
/***************** DYNAMIC ROAD DETECTION VARIABLES **************************/
/*****************************************************************************/

#define DEBUG 0

/** Color threashould for road*/
#define ROAD_THRESHOULD 10

/** Max tracking zone length in meters*/
#define MAX_TRACKING_ZONE 80
#define DETECTION_ZONE_PERC 0.3

#define ROAD_UPDATE_INTERVAL 10000 

#define TRACKING_ZONE_CORNERS 4

/** Minimum percentage that should be covered by a starting-point to be a valid starting-point*/
#define MIN_COVERAGE 0.6

/** Minimum number of road-neighbours that a point should have to be considered as road-pixel*/
#define MIN_NEIGHBOURS 4
#define MAX_POINTS_PER_ITER 50
#define idx_in_road_detection_window_range(i,j) ((i>image_center.y-DELTA && i<image_center.y+DELTA) \
                                                 &&                     \
                                                 (j>image_center.x-DELTA && j<image_center.x+DELTA))

/**
 *
 */
#define IDX_IN_IMAGE(i,j) (i>0 && j>0 && i<(signed)height && j<(signed)width)

/** Max points is the number of points that form the detection window 
 *  on vertical and horizontl size
 */
#define MAX_POINTS 26
#define TOTAL_POINTS (MAX_POINTS*MAX_POINTS)
#define DELTA (MAX_POINTS/2)*STEP
#define STEP 2

/**
 *
 */
Road::Road(){
   initialized=false;
   use_static_road=true;
   tracking_zone_corners.resize(TRACKING_ZONE_CORNERS);
   road_detection_window.resize(TOTAL_POINTS);
   detection_zone_perc = DETECTION_ZONE_PERC;
}

/**
 *
 */
void Road::resize(unsigned int height_val, unsigned int width_val){

   if (not initialized)
   {
      printf("\nCreating a new Road with the following dim (%d:%d)\n",height_val,width_val);
      initialized = true;
      height = height_val;
      width = width_val;
      road.create(height, width, CV_8UC1);
      reset();
      update_tracking_zone();
      update();
   }
}

/**
 *
 */
void Road::set_corners(const std::vector<Tpoint2D>& roadPoints)
{

   /* Init static road to point to the image corners as first aproach. The user should
    *  select the road corners manually
    */
   for(unsigned int i=0; i<tracking_zone_corners.size(); i++)
   {
      tracking_zone_corners[i] = roadPoints[i];
   }
   
   update_tracking_zone();
   update();
}

/**
 *
 */
void Road::reset(){

   unsigned int i,j;

   starting_point = 0;
   last_road_update=0;
   next_road_static_point=0;
   static_road_width = 0;
   static_road_length = 0;
   
   /** Init road*/
   for (i=0; i<height; i++)
   {
      for (j=0; j<width; j++)
      {
         road.at<unsigned char>(i,j) = 0;
      }
   }
   
   init_road_detection_window();
}

/**
 *
 */
int Road::idx_is_on_detection_window(int i, int j)
{
   unsigned int k;
  
   for (k=0; k<road_detection_window.size(); k++)
   {
      if ((int)road_detection_window[k].y == i
          &&
          (int)road_detection_window[k].x == j)
      {
         return 1;
      }
   }
        
   return 0;
}

/**
 *
 */
void Road::init_road_detection_window(){

   int i,j,idx;

   image_center.y = height/2;
   image_center.x = width/2;
  
   printf("Road detection: image center is %d,%d\n",image_center.y, image_center.x);
  
   /** Init road detection window*/
   idx=0;
   for (i=image_center.y-DELTA; i<image_center.y+DELTA; i += STEP)
   {
      for (j=image_center.x-DELTA; j<image_center.x+DELTA; j += STEP)
      {
         road_detection_window[idx].y = i;
         road_detection_window[idx].x = j;
         idx++;
      }
   }
}

/**
 *
 */
int Road::road_detection(unsigned char* image,
                         int i,
                         int j, 
                         unsigned char r, 
                         unsigned char g,  
                         unsigned char b,
                         int* detection_window_sum){

   int output=0;
  
   if (IDX_IN_IMAGE(i,j) && !road.at<unsigned char>(i,j))
   {
      unsigned char pr,pg,pb;
      int idx = (i*width+j)*3;

      pr = image[idx];
      pg = image[idx+1];
      pb = image[idx+2];
      
      if (DEBUG)
         printf(" checking the pos %d,%d cmp %u,%u,%u -- %u,%u,%u\n",i,j,r,g,b,pr,pg,pb);
        
      if (abs(r-pr) <= ROAD_THRESHOULD
          &&
          abs(g-pg) <= ROAD_THRESHOULD
          &&
          abs(b-pb) <= ROAD_THRESHOULD)
      {
         road.at<unsigned char>(i,j) = 255;

         /** if this idx/point is on the detection window then we increment the coverage count */
         if (idx_in_road_detection_window_range(i,j) && idx_is_on_detection_window(i,j))
            (*detection_window_sum)++;

         if (DEBUG)
            printf(" %d,%d may be road \n",i,j);

         /** Do the call recursively on the 8 neighbours*/ 
      
         road_detection(image, i-1, j-1, pr,pg,pb,detection_window_sum);
         road_detection(image, i, j-1, pr,pg,pb,detection_window_sum);
         road_detection(image, i+1,j-1, pr,pg,pb,detection_window_sum);
      
         road_detection(image, i-1,j, pr,pg,pb,detection_window_sum);
         road_detection(image, i+1,j, pr,pg,pb,detection_window_sum);
      
         road_detection(image, i-1,j+1, pr,pg,pb,detection_window_sum);
         road_detection(image, i,j+1, pr,pg,pb,detection_window_sum);
         road_detection(image, i+1,j+1, pr,pg,pb,detection_window_sum);

         output = 1;
      }
    
   }
   else
   {
      output = 0;
   }

   return output;
}

/**
 *
 */
void Road::update_road_matrix()
{

   unsigned int i,j;

   for (i=1; i<height-1; i++)
      for (j=1; j<width-1; j++)
      {
         int count=0;
         if (road.at<unsigned char>(i-1,j-1))
            count++;
         if (road.at<unsigned char>(i,j-1))
            count++;
         if (road.at<unsigned char>(i+1,j-1))
            count++;
         if (road.at<unsigned char>(i-1,j))
            count++;
         if (road.at<unsigned char>(i+1,j))
            count++;
         if (road.at<unsigned char>(i-1,j+1))
            count++;
         if (road.at<unsigned char>(i,j+1))
            count++;
         if (road.at<unsigned char>(i+1,j+1))
            count++;
        
         /** A pixel is in considered in the road if it has more than 
          * MIN_NEIGHBOURS neighbhours that are on the road
          */
         road.at<unsigned char>(i,j) = (count >= MIN_NEIGHBOURS);
      }
}

/**
 *
 */
void Road::update(const colorspaces::Image& new_image){

   /** If static road is being used then nothign is updated*/
   // if (use_static_road)
   //    return;
   //TODO(redo): By now only static road option is available
   return; 

   unsigned int k;
   unsigned int idx;
   int detection_window_coverage=0;
   int iter=0;
   struct timeval time;
   unsigned int now=0;
   unsigned char* image = new_image.data;

   /** Do the needed initialization if this is the first time we use the object*/
   // resize(new_image.height, new_image.width);

   gettimeofday(&time, NULL);
   now = (time.tv_sec*1000+time.tv_usec/1000);

   if ((now - last_road_update) < ROAD_UPDATE_INTERVAL)
   {
      return;
   }
  
   printf("updating_road -- starting point is %d\n", starting_point);

   /** 
    *  For each point in the road detection window we will calculate how much points
    *  of the detection window we cover if we use it as starting-point for road detection.
    *  If its coverage percentage is higher than MIN_COVERAGE then we update the road.
    * 
    */
   for (k=starting_point; k<road_detection_window.size(); k++)
   {
      iter++;

      /** to keep all the system running we do just a few iterations*/
      if (iter == MAX_POINTS_PER_ITER)
         break;

      idx=road_detection_window[k].y*width+road_detection_window[k].x;

      reset();
      detection_window_coverage = 0;
      
      /*******************************/
      road_detection(image, 
                     road_detection_window[k].y,
                     road_detection_window[k].x , 
                     image[idx*3], 
                     image[idx*3+1], 
                     image[idx*3+2],
                     &detection_window_coverage);

      if ((float)detection_window_coverage/road_detection_window.size()>MIN_COVERAGE)
      {
         printf(" road detection results: iter %d -- %d,%d %d coverage=%.2f per/cent\n",
                k,
                road_detection_window[k].y,
                road_detection_window[k].x,
                detection_window_coverage,
                100*((float)detection_window_coverage/road_detection_window.size()));

         starting_point = 0;
         last_road_update = now;
         update_road_matrix();
         break;
      }
   }
   
   calculate_dynamic_tracking_zone();
   
   starting_point=k++;
   if (starting_point == road_detection_window.size())
   {
      last_road_update = now;
      starting_point=0;
   }
}

/**
 *
 */
void Road::get_tracking_zone_finish_line(Tsegment* s)
{
   if (switch_detection_zone)
   {
      finish_line.orig = tracking_zone_corners[2];
      finish_line.end = tracking_zone_corners[3];
   }
   else
   {
      finish_line.orig = tracking_zone_corners[0];
      finish_line.end = tracking_zone_corners[1];
   }
   
   calculate_line(&finish_line);
   *s = finish_line;
}

/**
 *
 */
void Road::push_tracking_zone_point(int i, int j)
{
   Tpoint2D p;

   p.y = i;
   p.x = j;

   tracking_zone_corners[next_road_static_point] = p;
   /** Circular inc*/
   next_road_static_point = (next_road_static_point+1) % tracking_zone_corners.size();
   update_tracking_zone();
   update();
}

/**
 *
 */
void Road::get_tracking_zone_corners(Tpoint2D *left_corner, Tpoint2D *right_corner)
{
   /** Calculate the left most and right most corners*/ 
   get_polygon_corners(&tracking_zone, false, left_corner, right_corner);
}

/**
 *
 */
void Road::get_tracking_zone_points(std::vector<Tpoint2D>& roadPoints)
{
   int i=0;

   roadPoints.resize(tracking_zone.num_seg);
   for (i=0; i<tracking_zone.num_seg; i++)
   {
      roadPoints[i] = tracking_zone.segments[i].orig;
   }
}

/**
 *
 */
void Road::get_detection_zone_points(std::vector<Tpoint2D>& detection_zone_points)
{
   int i=0;

   detection_zone_points.resize(detection_zone.num_seg);
   for (i=0; i<detection_zone.num_seg; i++)
   {
      detection_zone_points[i] = detection_zone.segments[i].orig;
   }
}

/**
 *
 */
void Road::calculate_dynamic_tracking_zone()
{
   HPoint3D A;
   Tpoint2D proj;
   Tpoint2D upleft_itx,upright_itx;
   Tpoint2D downleft_itx,downright_itx;
   unsigned int i,j;
   HPoint2D finish_point_proj;
   PinHoleCamera* camera = PinHoleCamera::Instance();
   
   upright_itx.x = upright_itx.y = 0;
   upleft_itx.x = upleft_itx.y = 0;
   downleft_itx.x = downleft_itx.y = 0;
   downright_itx.x = downright_itx.y = 0;

   A.X = 0;
   A.Z = 0;
   A.Y = MAX_TRACKING_ZONE;
   A.H = 1;

   camera->myproject(A, &finish_point_proj);
   camera->optic2grafic(&finish_point_proj);
   proj.x = finish_point_proj.x;
   proj.y = finish_point_proj.y;

   /** calculate the intersection between the horizontal line that passes
    *  form the calculate finish_point (finish_line) and the calculated dynamic
    *  road
    */
   if (IDX_IN_IMAGE(proj.x,proj.y))
   {
      /** set the v coor to be the same as the projection*/
      upleft_itx.y = proj.y;
      upright_itx.y = proj.y;
      
      //Calculate the left intersection
      for (j=0; j<width; j++)
         if (road.at<unsigned char>((int)proj.y,j))
         {
            upleft_itx.x = j;
            break;
         }
      
      //Calculate the right intersection
      for (j=width-1; j>=0; j--)
         if (road.at<unsigned char>((int)proj.y,j))
         {
            upright_itx.x = j;
            break;
         }
      
      /** The labels on the next loops are used to break the nested
       *  loops since using only break is not enough to break this 
       *  kind of  loops.
       */

      for (j=0; j<width; j++)
         for (i=0; i<height; i++)
            if (road.at<unsigned char>(i,j))
            {
               downleft_itx.y = i;
               downleft_itx.x = j;
               goto label_1;
            }

     label_1:
      //Calculate the left intersection
      for (j=width-1; j>=0; j--)
         for (i=0; i<height; i++)
            if (road.at<unsigned char>(i,j))
            {
               downright_itx.y = i;
               downright_itx.x = j;
               goto label_2;
            }

     label_2:

      /** Calculate the tracking zone using those corners*/
      tracking_zone_corners[0] = upleft_itx;
      tracking_zone_corners[1] = upright_itx;
      tracking_zone_corners[2] = downright_itx;
      tracking_zone_corners[3] = downleft_itx;
      update_tracking_zone();

   }//IF

   printf(" lf(%d:%d) righ(%d:%d) dwlf(%d:%d) dwrigh(%d:%d) \n",
          upleft_itx.y, upleft_itx.x,
          upright_itx.y, upright_itx.x,
          downleft_itx.y, downleft_itx.x,
          downright_itx.y, downright_itx.x);

}

/**
 *
 */
void Road::calculate_tracking_zone()
{
   unsigned int i,j;
   Tpoint2D p;

   /** Init road*/
   for (i=0; i<height; i++)
   {
      for (j=0; j<width; j++)
      {
         road.at<unsigned char>(i,j) = 0;
      }
   }

   /** Init road*/
   for (i=0; i<height; i++)
   {
      for (j=0; j<width; j++)
      {
         p.y = i;
         p.x = j;
         road.at<unsigned char>(i,j) = pip(&tracking_zone, p)?255:0;
      }
   }
}

/**
 *
 */
void Road::update()
{
   if (initialized)
   {
      calculate_tracking_zone();
      calculate_dimensions();
   }
}

/**
 *
 */
Tpoint2D Road::get_point(Tpoint2D a, Tpoint2D b, double alfa)
{
   Tpoint2D r;
   r.x = (1-alfa)*a.x + alfa*b.x;
   r.y = (1-alfa)*a.y + alfa*b.y;
   return r;
}

/**
 *
 */
Rect Road::get_roi()
{
   Tpoint2D tracking_zone_lc, tracking_zone_rc;
   Tpoint2D detection_zone_lc, detection_zone_rc;
   Tpoint2D left_most_corner, right_most_corner;
   Rect roi;

   get_polygon_corners(&tracking_zone, false, &tracking_zone_lc, &tracking_zone_rc);
   get_polygon_corners(&detection_zone, false, &detection_zone_lc, &detection_zone_rc);

   /** get the left_most_corner*/
   left_most_corner.x = floor(min(tracking_zone_lc.x, detection_zone_lc.x));
   left_most_corner.y = floor(min(tracking_zone_lc.y, detection_zone_lc.y));

   /** get the left_most_corner*/
   right_most_corner.x = floor(max(tracking_zone_rc.x, detection_zone_rc.x));
   right_most_corner.y = floor(max(tracking_zone_rc.y, detection_zone_rc.y));
   

   roi.x = left_most_corner.x;
   roi.y = left_most_corner.y;
   roi.height = fabs(right_most_corner.y-left_most_corner.y);
   roi.width = fabs(right_most_corner.x-left_most_corner.x);

   return roi;
}

/**
 *
 */
void Road::update_detection_zone_up(){

   /** Object segments*/
   Tsegment s1,s2,s3,s4;
   Tsegment ds1,ds2,ds3,ds4;
   Tpoint2D E,F,C,D;

   /***************************
    *
    *  Road layout:
    *
    *   0--------------1
    *   |  detectoion  |
    *   |     zone     |
    *   E_____________ F
    *   |              |
    *   |              |
    *   |              |
    *   3--------------2
    */

   E = get_point(tracking_zone_corners[0], tracking_zone_corners[3], detection_zone_perc);
   F = get_point(tracking_zone_corners[1], tracking_zone_corners[2], detection_zone_perc);

   ds1.orig = tracking_zone_corners[0];
   ds1.end = tracking_zone_corners[1];

   ds2.orig = tracking_zone_corners[1];
   ds2.end = F;


   ds3.orig = F;
   ds3.end = E;
   
   ds4.orig = E;
   ds4.end = tracking_zone_corners[0];

   calculate_line(&ds1);
   calculate_line(&ds2);
   calculate_line(&ds3);
   calculate_line(&ds4);

   detection_zone.num_seg=4;
   detection_zone.num_shadow_seg=0;
   detection_zone.segments[0]=ds1;
   detection_zone.segments[1]=ds2;
   detection_zone.segments[2]=ds3;
   detection_zone.segments[3]=ds4;

}

/**
 *
 */
void Road::update_detection_zone_down(){

   /** Object segments*/
   Tsegment s1,s2,s3,s4;
   Tsegment ds1,ds2,ds3,ds4;
   Tpoint2D E,F,C,D;

   E = get_point(tracking_zone_corners[3], tracking_zone_corners[0], detection_zone_perc);
   F = get_point(tracking_zone_corners[2], tracking_zone_corners[1], detection_zone_perc);

   ds1.orig = E;
   ds1.end = F;

   ds2.orig = F;
   ds2.end = tracking_zone_corners[2];

   ds3.orig = tracking_zone_corners[2];
   ds3.end = tracking_zone_corners[3];
   
   ds4.orig = tracking_zone_corners[3];
   ds4.end = E;

   calculate_line(&ds1);
   calculate_line(&ds2);
   calculate_line(&ds3);
   calculate_line(&ds4);

   detection_zone.num_seg=4;
   detection_zone.num_shadow_seg=0;
   detection_zone.segments[0]=ds1;
   detection_zone.segments[1]=ds2;
   detection_zone.segments[2]=ds3;
   detection_zone.segments[3]=ds4;

}

/**
 *
 */
void Road::update_tracking_zone(){

   /** Object segments*/
   Tsegment s1,s2,s3,s4;

   find_convex_hull(tracking_zone_corners);
   order_points(tracking_zone_corners);

   s1.orig = tracking_zone_corners[0];
   s1.end = tracking_zone_corners[1];

   s2.orig = tracking_zone_corners[1];
   s2.end = tracking_zone_corners[2];

   s3.orig = tracking_zone_corners[2];
   s3.end = tracking_zone_corners[3];

   s4.orig = tracking_zone_corners[3];
   s4.end = tracking_zone_corners[0];

   calculate_line(&s1);
   calculate_line(&s2);
   calculate_line(&s3);
   calculate_line(&s4);

   tracking_zone.num_seg=4;
   tracking_zone.num_shadow_seg=0;
   tracking_zone.segments[0]=s1;
   tracking_zone.segments[1]=s2;
   tracking_zone.segments[2]=s3;
   tracking_zone.segments[3]=s4;

   if (switch_detection_zone)
   {
      update_detection_zone_up();
      orig.x = (tracking_zone_corners[0].x+tracking_zone_corners[1].x)/2;
      orig.y = (tracking_zone_corners[0].y+tracking_zone_corners[1].y)/2;
      end.x = (tracking_zone_corners[2].x+tracking_zone_corners[3].x)/2;
      end.y = (tracking_zone_corners[2].y+tracking_zone_corners[3].y)/2;
   }
   else
   {
      update_detection_zone_down();
      orig.x = (tracking_zone_corners[2].x+tracking_zone_corners[3].x)/2;
      orig.y = (tracking_zone_corners[2].y+tracking_zone_corners[3].y)/2;
      end.x = (tracking_zone_corners[0].x+tracking_zone_corners[1].x)/2;
      end.y = (tracking_zone_corners[0].y+tracking_zone_corners[1].y)/2;
   }
}

/**
 *
 */
void Road::calculate_dimensions(){

   PinHoleCamera* camera = PinHoleCamera::Instance();
   Tpoint2D left_corner, right_corner;
   HPoint2D lhcp,rhcp;
   HPoint3D lcp,rcp;
   
   /** Calculate the left most and right most corners*/
   //FIXME: avoid this direct access
   left_corner = tracking_zone_corners[0];
   right_corner = tracking_zone_corners[2];
   // get_lr_corners(&tracking_zone, &left_corner, &right_corner);

   lhcp.x = left_corner.x;
   lhcp.y = left_corner.y;
   rhcp.x = right_corner.x;
   rhcp.y = right_corner.y;

   camera->reproject(lhcp, &lcp, 0);
   camera->reproject(rhcp, &rcp, 0);

   static_road_width  = abs(lcp.X-rcp.X);   
   static_road_length = abs(lcp.Y-rcp.Y);

   if (DEBUG)
   {
      printf("(%d:%d) -> (%.2f:%.2f)   (%d:%d) -> (%.2f:%.2f) \n",
             left_corner.y,left_corner.x,
             lcp.X,lcp.Y,
             right_corner.y, right_corner.x,
             rcp.X,rcp.Y);

      printf("w=%.2f h=%.2f\n", static_road_width, static_road_length);
   }
}

/**
 *
 */
void Road::set_switch_detection_zone(bool new_val){
   switch_detection_zone=new_val;
   update_tracking_zone();
}

/**
 *
 */
void Road::set_detection_zone_perc(double new_perc){
   detection_zone_perc=(double)(new_perc/100.0);
   update_tracking_zone();
};
