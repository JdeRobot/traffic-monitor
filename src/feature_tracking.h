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
#ifndef _FEATURE_TRACKER_
#define _FEATURE_TRACKER_

#include <iostream>
#include <vector>

#include "blob.h"
#include "road_detection.h"
#include "singleton.h" 

using std::vector;

namespace trafficmonitor{

class FeatureTracker: public CSingleton<FeatureTracker>{

public:
   /**
    *
    */
   void init();

   /**
    * Update the tracking information by detecting new blobs on the image received
    * as input. The new blobs are matched with the already tracked vehicles so the
    * new vehicles 2D/3D position is updated.
    */
  void track_blobs(bool klt_only,
                   colorspaces::Image& new_frame,
                   timeval timestamp,
                   vector<Blob*>& tracked_blobs,
                   vector<Blob*>& tmp_blobs,
                   unsigned int num_blobs);

   /**
    *
    */
   Tvector get_movement_direction() {return mov_direction;};

protected:

   friend class CSingleton<FeatureTracker>;
   
private:

   IplImage *image , *grey , *prev_grey , *pyramid , *prev_pyramid , *swap_temp;
   unsigned short blob_temporal_id;

   //TODO: comment
   Tvector mov_direction;
      
   /**
    *
    */
   FeatureTracker();
   
   /**
    *
    */
   CvRect get_bounding_rect(CvPoint2D32f* points, int count);

   /**
    *
    */
   int merge_vehicles(vector<Blob*>& tracked_blobs);

   /**
    *
    */
   bool vote_the_nearest_blob(vector<Blob*>& tmp_blobs, CvPoint2D32f* new_points, int num_points, Blob& tracked_blob);

   /**
    *
    */
   bool match_blob_using_proximity(Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob);
      
   /**
    *
    */
   bool update_the_nearest_blob(vector<Blob*>& tmp_blobs, Tpoint2D p, Blob& tracked_blob);

   bool match_vehicle_in_detection_zone(Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob);
   bool match_blob_using_klt(Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob);
   void get_good_features(IplImage* grey, Blob* blob);
   void get_uniform_points(Blob* blob);
   bool match_blob(bool klt_only, Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob);

   /**
    *
    */
   Blob* find_blob(vector<Blob*>& blobs, int id);
      
   /**
    *
    */
   void process_detected_blobs(IplImage* frame,
                               timeval timestamp,
                               vector<Blob*>& tracked_blobs,
                               vector<Blob*>& tmp_blobs,
                               unsigned int num_blobs);
};
}        
#endif
