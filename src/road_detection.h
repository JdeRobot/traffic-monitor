/*
 *  Copyright (C) 1997-2008 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Authors : Redouane Kachach <redo.robot at gmail.com>
 *
 * This module is used to detect the road on the image using as input the current estimated background. We
 * are supposing that the camera is centered on the road so if we take some pixel from the image center
 * it's likely to belong to the road. However, if we take just a single point this may have slighlty different
 * color than the road, to deal with this we will be using a detection_window centered on the image center. 
 * The detection road algorithm basically will try each point from the detection window do recursive detection
 * to check if its neighbours belong to the road also (color based check), if so then we check its neighbours
 * recursively and so on. If some starting point cover a high percentage of the detection window then it's
 * used as starting point, else, we try an other point from the detection window.
 *
 * 
 */
#ifndef __ROAD_DETECTION__
#define __ROAD_DETECTION__

#include <colorspacesmm.h>

/* #include <opencv/cv.h> */
#include <iostream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include "planar_geom.h"
#include "singleton.h"

using std::vector;
using namespace cv;


class Road: public CSingleton<Road>{

public:

   /**
    *
    */
   void set_corners(const std::vector<Tpoint2D>& roadPoints);

   /**
    *
    */
   void set_static_road(bool new_val){use_static_road=new_val;}

   /**
    *
    */
   void set_switch_detection_zone(bool new_val);

   /**
    *
    */
   void resize(unsigned int height_val, unsigned int width_val);
   
   /**
    *
    */
   void update(const colorspaces::Image& image);

   /**
    *
    */
   void set_detection_zone_perc(double new_perc);

   /**
    *
    */
   void update();

   /**
    *
    */
   void get_tracking_zone_points(std::vector<Tpoint2D>& roadPoints);
   void get_detection_zone_points(std::vector<Tpoint2D>& detection_zone_points);
   
   /**
    *
    */
   void get_tracking_zone_finish_line(Tsegment* s);

   /**
    *
    */
   bool is_road(unsigned int row, unsigned int col){return road.at<unsigned char>(row,col);}
   void get_roi(Tpoint2D* lfc, Tpoint2D* rhc) {get_polygon_corners(&tracking_zone, false, lfc, rhc);};
   Rect get_roi();
   
   /**
    *
    */
   bool in_detection_zone(Tpoint2D point) {return pip(&detection_zone,point);}

   /**
    *
    */
   bool in_road(Tpoint2D point) {return in_detection_zone(point) || in_tracking_zone(point);}
   
   /**
    *
    */
   bool in_tracking_zone(Tpoint2D point) {return (!pip(&detection_zone,point)  && pip(&tracking_zone,point));}

   /**
    *
    */
   float get_length(){return static_road_length;};
   float get_width(){return static_road_width;};

   Tpoint2D orig;
   Tpoint2D end;
   // road (i,j) where i is the vertical axes (0..height) and j is the vertical dimension (0..width)
   Mat road; 

protected:

   friend class CSingleton<Road>;

   /**
    *
    */
   Road();


private:

   /**
    *
    */
   void reset();
   
   /**
    *
    */
   void get_tracking_zone_corners(Tpoint2D *left_corner, Tpoint2D *right_corner);

   /**
    *
    */
   void calculate_dynamic_tracking_zone();
   

   /**
    *
    */
   void update_tracking_zone();
   void update_detection_zone_up();
   void update_detection_zone_down();

   /**
    *
    */
   void order_tracking_zone_corners(vector<Tpoint2D>& points);

   /**
    *
    */
   void calculate_tracking_zone();
   
   /**
    *
    */
   int idx_is_on_detection_window(int i, int j);

   /**
    *
    */
   void push_tracking_zone_point(int i, int j);

   /**
    *
    */
   int road_detection(unsigned char* image,
                      int i,
                      int j, 
                      unsigned char r, 
                      unsigned char g,  
                      unsigned char b,
                      int* detection_window_sum);
   /**
    *
    */
   void init_road_detection_window();
   
   /**
    *
    */
   void update_road_matrix();

   /**
    *
    */
   void calculate_dimensions();
   Tpoint2D get_point(Tpoint2D a, Tpoint2D b, double alfa);


   /**
    * Static tracking zone related variables
    */
   vector<Tpoint2D> tracking_zone_corners;
   Tpolygon tracking_zone;
   Tpolygon detection_zone;
   double detection_zone_perc;
   int next_road_static_point;

   /**
    * Dynamic/Automatic tracking zone related variables
    */
   Tsegment finish_line;
   unsigned int starting_point;
   unsigned int last_road_update;
   vector<Tpoint2D> road_detection_window;
   Tpoint2D image_center;

   unsigned int width;
   unsigned int height;

   float static_road_width;
   float static_road_length;
   bool use_static_road;
   bool switch_detection_zone;
   bool initialized;
};

#endif
