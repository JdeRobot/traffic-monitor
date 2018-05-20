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
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "math.h"
#include <sys/types.h>
#include <opencv/cv.h>

#include "proximity_tracking.h"
#include "road_detection.h"
#include "movement_model.h"
#include "planar_geom.h"

#define MIN_ANGLE 90
#define MAX_DISTANCE 10000

namespace trafficmonitor{

#define DEBUG 0
#define LOG(args...){if (DEBUG) printf(args);}

#define MAX_NEW_BLOBS 10
#define MAX_TEMPORAL_ID 1000
#define MIN_DISTANCE 10

/**
 *
 */
ProximityTracker::ProximityTracker(){blob_temporal_id=1;}

/**
 *
 */
void ProximityTracker::init()
{
   blob_temporal_id=1;
   mov_direction.a = 0;
   mov_direction.b = 0;
}

/**
 *
 */
void ProximityTracker::track_blobs(colorspaces::Image& new_frame
                                 ,timeval timestamp
                                 ,vector<Blob*>& tracked_blobs
                                 ,vector<Blob*>& tmp_blobs
                                 ,unsigned int num_blobs)
{
   LOG("--> tracking %zu blobs\n", tracked_blobs.size());

   /** Init tracked blobs*/
   for(unsigned int i=0; i<tracked_blobs.size(); i++)
   {
      tracked_blobs[i]->set_matched(false);
   }

   Road* road = Road::Instance();
   Tsegment finish_line;
   if (road)
   {
      road->get_tracking_zone_finish_line(&finish_line);
   }

   for (unsigned int i=0; i<tracked_blobs.size(); i++)
   {
      Blob* blob = tracked_blobs[i];

      if (blob->is_free())
      {
         continue;
      }

      Blob matched_blob;
      if (match_vehicle(blob, tmp_blobs, num_blobs, &matched_blob))
      {
         /** Check if we are close to the finish line, in this case calculate the blob
          *  trajectory related stats.
          */
         if (GET_LINE_POINT_DISTANCE(finish_line, blob->get_2d_center()) < MIN_DISTANCE)
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

   process_detected_blobs(timestamp, tracked_blobs, tmp_blobs, num_blobs);
}

/**
 *
 */
void ProximityTracker::process_detected_blobs(timeval timestamp
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
            *tracked_blobs[free_pos] = *tmp_blobs[i];
            tracked_blobs[free_pos]->set_id(blob_temporal_id++ % MAX_TEMPORAL_ID);
            tracked_blobs[free_pos]->start_tracking(timestamp);

            Blob* blob = tracked_blobs[free_pos];

            LOG(">>>> new vehicle (%d) tracked=%s stored on pos=%d 2dcenter(%d:%d) 3dcenter(%1.f:%1.f:%1.f) in_timestamp=%u\n\n",
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
   }//FOR

   blob_temporal_id -= merge_vehicles(tracked_blobs);
}

int ProximityTracker::merge_vehicles(vector<Blob*>& tracked_blobs)
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
Blob* ProximityTracker::find_blob(vector<Blob*>& blobs, int id)
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

/**
 *
 */
bool ProximityTracker::match_vehicle(Blob* blob, vector<Blob*>& tmp_blobs, unsigned int num_blobs, Blob* matched_blob)
{
   float distance = MAX_DISTANCE;
   int pos=-1;
   Tpoint2D ec = blob->get_2d_center(); // Ellipse center

   for (unsigned int j=0; j<num_blobs; j++)
   {
      if (not tmp_blobs[j]->already_matched())
      {
         float inter_blob_2d_dis = DISTANCE_2D(blob->get_2d_center(), tmp_blobs[j]->get_2d_center());

         if (blob->IsBlobInProximityElipse(tmp_blobs[j]) && (inter_blob_2d_dis < distance))
         {
            distance = inter_blob_2d_dis;
            pos = j;
         }
         else if (inter_blob_2d_dis < distance)
         {
            distance = inter_blob_2d_dis;
            pos = j;
         }
      }

   }//FOR

   if (pos>=0)
   {
      *matched_blob = *tmp_blobs[pos];
      blob->track(*matched_blob);
      tmp_blobs[pos]->set_matched(true);

      return true;
   }
   else
   {
      return false;
   }
}

}
