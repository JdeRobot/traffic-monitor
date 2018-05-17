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
#ifndef _PIXEL_SHIFT_TRACKER_
#define _PIXEL_SHIFT_TRACKER_

#include <iostream>
#include <vector>

#include "blob.h"
#include "road_detection.h"
#include "singleton.h" 
#include "Image.h"

using std::vector;

namespace trafficmonitor{

class PixelShiftTracker: public CSingleton<PixelShiftTracker>{

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
   void track_blobs(colorspaces::Image& new_frame,
                    timeval timestamp,
                    vector<Blob*>& tracked_blobs,
                    vector<Blob*>& tmp_blobs,
                    unsigned int num_blobs);

   /**
    *
    */
   Tvector get_movement_direction() {return mov_direction;};
      
protected:

   friend class CSingleton<PixelShiftTracker>;
   
private:

   unsigned short blob_temporal_id;

   //TODO: comment
   Tvector mov_direction;
      
   /**
    *
    */
   PixelShiftTracker();

   bool match_vehicle_in_detection_zone(Image<unsigned char>* new_frame
                                        ,Blob* blob
                                        ,vector<Blob*>& tmp_blobs
                                        ,unsigned int num_blobs);
   
   bool match_blob(Image<unsigned char>* new_frame
                   ,Blob* blob
                   ,vector<Blob*>& tmp_blobs
                   ,unsigned int num_blobs);

   bool match_blob_using_pixeltrack(Image<unsigned char>* new_frame
                                    ,vector<Blob*>& tmp_blobs
                                    ,Blob* blob);

   bool update_the_nearest_blob(vector<Blob*>& tmp_blobs, Tpoint2D p2, Blob& tracked_blob, CvRect& output_rect);



   /**
    *
    */
   Blob* find_blob(vector<Blob*>& blobs, int id);
      
   /**
    *
    */
   void process_detected_blobs(Image<unsigned char>* new_frame
                               ,timeval timestamp,
                               vector<Blob*>& tracked_blobs,
                               vector<Blob*>& tmp_blobs,
                               unsigned int num_blobs);

};
}        
#endif
