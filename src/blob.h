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
 */

#ifndef _BLOB_H_
#define _BLOB_H_

#include "planar_geom.h"
#include "progeo.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#define MAX_COUNT 50
#define MAX_CONTOUR_SIZE 3000
#define MIN_LOCATION_HISTORY 10
#define MIN_POINTS_FOR_LINE_FIT 6
#define MIN_LIFE 6

namespace trafficmonitor{


/**
 *
 */
class Blob{

public:

   /**
    *
    */
   Blob();

   /**
    *
    */
   bool IsBlobInProximityElipse(Blob* blob) const;

   /**
    *
    */
   float get_covered_distance();

   /**
    *
    */
  int get_life() {return m_life;}
  int inc_life() {m_life++;}

   /**
    *
    */
   bool GetDirection(Vec4f& direction) const;

   /**
    *
    */
   bool get_estimated_size(int& w, int& h);

   /**
    *
    */
   bool get_metric_size(double& w, double& h);

   /**
    *
    */
   static bool compare_size(const Blob* first, const Blob* second);
   
   /**
    *
    */
   virtual void init();

   /**
    *
    */
   void join(Blob& v2);

   /**
    *
    */
   void insert_in_blob(const Tpoint2D& point)
      {
         free=false;
         num_points++;

         /** Updating the left and rigth corners*/
         left_corner.x = min(point.x,left_corner.x);
         left_corner.y = min(point.y,left_corner.y);
         right_corner.x = max(point.x,right_corner.x);
         right_corner.y = max(point.y,right_corner.y);

         center_of_mass.x += point.x;
         center_of_mass.y += point.y;
      }

   /**
    *
    */
   virtual bool is_valid_tracking();
   
   /**
    *
    */
   void calculate_blob_center(){

      left_corner.x = INT_MAX;
      left_corner.y = INT_MAX;

      right_corner.x = 0;
      right_corner.y = 0;
         
      for(unsigned int m=0; m<contour.size(); m++)
      {
         Tpoint2D p;
         p.x = contour[m].x;
         p.y = contour[m].y;
         insert_in_blob(p);
      }

      update_blob_box_center();
   }
      
   void update_blob_box_center();
   void update_3d_center();

   void init_points(){memset(points, 0, MAX_COUNT*sizeof(CvPoint2D32f));};

   /**
    *
    */
   void update_blob_box_center(CvRect rect){

      /** Updating the left and rigth corners*/
      left_corner.x = rect.x;
      left_corner.y = rect.y;
      right_corner.x = rect.x+rect.width;
      right_corner.y = rect.y+rect.height;
      //TODO: How to update the points in this case!! num_points = rect.width*rect.height;
      num_points = 0;

      current_2d_center.x = (int)(right_corner.x + left_corner.x)/2;
      current_2d_center.y = (int)(right_corner.y + left_corner.y)/2;
   }

   void ShiftCenter(Tpoint2D newCenter)
   {
      Tpoint2D prev = current_2d_center;

      int x0 = left_corner.x + (right_corner.x - left_corner.x)/2;
      int y0 = left_corner.y + (right_corner.y - left_corner.y)/2;

      int deltaX = newCenter.x - x0;
      int deltaY = newCenter.y - y0;

      left_corner.x += deltaX;
      left_corner.y += deltaY;
      right_corner.x += deltaX;
      right_corner.y += deltaY;

      current_2d_center.x += deltaX;
      current_2d_center.y += deltaY;

      printf("** Delta(%d,%d) - prev(%d,%d) - curr(%d,%d) \n", deltaX, deltaY, prev.x, prev.y, current_2d_center.x,current_2d_center.y);
   }

   /**
    *
    */
   CvRect get_rect() const
   {
      CvRect tmp;
      tmp.x = left_corner.x;
      tmp.y = left_corner.y;
      tmp.width = fabs(left_corner.x-right_corner.x);
      tmp.height = fabs(left_corner.y-right_corner.y);
      return tmp;
   }

   /**
    *
    */
   void set_rect(CvRect newRect)
   {
      left_corner.x = newRect.x;
      left_corner.y = newRect.y;
      right_corner.x = left_corner.x+newRect.width;
      right_corner.y = left_corner.y+newRect.height;

      current_2d_center.x = (int)(right_corner.x + left_corner.x)/2;
      current_2d_center.y = (int)(right_corner.y + left_corner.y)/2;
   }
   

   /**
    *
    */
   bool is_being_tracked(){return is_tracked;};

   /**
    *
    */
   int get_id() const {return id;};
   void set_id(int new_id){id=new_id;};

   /**
    *
    */
   const bool is_free(){return free;};
   void set_free(bool new_value){free=new_value;};

   /**
    *
    */
   const bool already_matched(){return matched;};
   void set_matched(bool value){matched=value;};

   /**
    *
    */
   void set_prev_2d_center(Tpoint2D new_center){prev_2d_center=new_center;};
   const Tpoint2D get_prev_2d_center(){return prev_2d_center;};

   /**
    *
    */
   void set_first_2d_center(Tpoint2D new_center){first_2d_center=new_center;};
   const Tpoint2D get_first_2d_center(){return first_2d_center;};

   /**
    *
    */
   void set_2d_center(Tpoint2D new_center){current_2d_center=new_center;};
   const Tpoint2D get_2d_center() const {return current_2d_center;};
   const Tpoint2D get_3d_center() const {return Tpoint2D(current_3d_center.Y, current_3d_center.X);}; // return the current 3D center (XY plan since Z=0)
   const Tpoint2D get_predicted_3d_center();

   void calculate_center_of_mass()
   {
      center_of_mass.x = center_of_mass.x / get_num_points();
      center_of_mass.y = center_of_mass.y / get_num_points();
   };

   const Tpoint2D get_center_of_mass() const
   {
      return center_of_mass;
   };
   
   /**
    *
    */
   void set_first_3d_center(HPoint3D new_center){first_3d_center=new_center;};
   const HPoint3D get_first_3d_center(){return first_3d_center;};

   /**
    *
    */
   void set_current_3d_center(HPoint3D new_center){current_3d_center=new_center;};
   const HPoint3D get_current_3d_center(){return current_3d_center;};

   /**
    *
    */
   void set_prev_3d_center(HPoint3D new_center){prev_3d_center=new_center;};
   const HPoint3D get_prev_3d_center(){return prev_3d_center;};

   /**
    *
    */
   void set_ingress_timestamp(timeval new_ts){ingress_timestamp = new_ts;};
   const timeval get_ingress_timestamp(){return ingress_timestamp;};

   /**
    *
    */
   void set_egress_timestamp(timeval new_ts){egress_timestamp = new_ts;};
   const timeval get_egress_timestamp(){return egress_timestamp;};

   /**
    *
    */
   const Tpoint2D get_left_corner(){return left_corner;};
   void set_left_corner(Tpoint2D lc){left_corner=lc;};
   const Tpoint2D get_right_corner(){return right_corner;};
   void set_right_corner(Tpoint2D rc){right_corner=rc;};

   /**
    *
    */
   const Tpoint2D get_left_most_corner(){return left_most_corner;};
   void set_left_most_corner(Tpoint2D lc){left_most_corner=lc;};
   const Tpoint2D get_right_most_corner(){return right_most_corner;};
   void set_right_most_corner(Tpoint2D rc){right_most_corner=rc;};

   /**
    *
    */
   virtual void start_tracking(timeval timestamp);

   /**
    *
    */
   virtual void track(Blob& blob);
   virtual void track(CvRect& newRect);

   /**
    *
    */
   virtual void tracked_by(Blob& blob)
   {
      tracking_blobs_id.push_back(blob.get_id());
   };      

   /**
    *
    */
   virtual void end_of_tracking(timeval timestamp);

   /**
    *
    */
   const int get_num_points() const {return num_points;};

   /**
    *
    */
   const int size()
   {
      CvRect bbox = get_rect();
      return bbox.width*bbox.height;
   };
   
   /**
    *
    */
   const float aspect_ratio()
   {
      CvRect bbox = get_rect();
      return (float)bbox.width/(float)bbox.height;
   };

   /**
    *
    */
   const float density()
   {
      return ((float)num_points/(float)size())*100;
   }
   
   /**
    *
    */
   double get_area(){
      return fabs(left_corner.x-right_corner.x)*fabs(left_corner.y-right_corner.y);
   };

   /**
    *
    */
   const float get_radius(){return radius;};

   /**
    *
    */
   Tpoint2D predict_next_position();
   void UpdateDirection();

   /**
    *
    */
   void print();

   //TODO: use set/get methods to access this fields.
   bool ocluded;
   std::vector<int> tracking_blobs_id;
   CvPoint2D32f points[MAX_COUNT];
   bool obsIsUsed;
   int num_features;
   std::vector<Point> contour;
   Tvector direction;
   bool in_opp_direction;
   int m_life;
   CvRect m_expectedRect;

   std::vector<Point> m_locationHistory;
   std::vector<Point> m_blobWitdh;
   std::vector<Point> m_blobHeigth;
   int m_idx;

   // number of new measurements needed to update the the position of the vehicle
   int m_upadte_interval;

   // Number of votes (used in KLT)
   int votes;

protected:
   
   /** This the unique identifier assigned to the blob
    *  during its presence in the tracking zone.
    */
   int id;


   /** Used to determine if this blob entry is in use*/
   bool free;

   bool is_tracked;

   /** Use during the tracking process to mark the blob as
    *  being already matched
    */
   bool matched;

   /** This is the center in the last movement detection. We will use it
    *  in order to detect movement direction of the blob.
    */
   Tpoint2D first_2d_center;
   Tpoint2D prev_2d_center;
   Tpoint2D current_2d_center;
   Tpoint2D center_of_mass;
   

   /** Those fields store the first 3D center of the blob and the current one*/
   HPoint3D first_3d_center;
   HPoint3D prev_3d_center;
   HPoint3D current_3d_center;

   /** This field store the timestamp (in ms) when the blob enter the detection zone.*/
   timeval ingress_timestamp;
   timeval egress_timestamp;
   timeval last_ts;

   /** Those fields are used to store the blob window corners its number of points/pixels*/
   int num_points;
   Tpoint2D left_corner;
   Tpoint2D right_corner;
   Tpoint2D left_most_corner;
   Tpoint2D right_most_corner;

   bool first_time;

   float speed;
   // float direction;
  
   /** This field is used the fusion operation where close blobs are merged in one blob*/
   float radius;

   // Measurements, only one parameter for angle
   CvMat* z_k;
};
}
#endif
