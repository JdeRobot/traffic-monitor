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
#include <sys/time.h>
#include <stdio.h>
#include <string.h>

#include "stats.h"
#include "database.h"

namespace trafficmonitor{

#define DISTANCE(a,b) sqrt((a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y))
#define GET_TS(ts) (ts.tv_sec*1000+(ts.tv_usec/1000))

/**
 *
 */
Stats::Stats()
{
   init(false);
   reset_stats();
}

/**
 *
 */
void Stats::init(bool classification_active)
{
   memset(models_counts, 0, sizeof(models_counts));
   last_vehicles.resize(MAX_VEHICLES_HISTORY);
   vehicles_trajectories.resize(MIN_VEHICLES_TRAJECTORIES);
   vehicles_trajectories.clear();
   num_trajectories = 0;
   m_classification_active = classification_active;
}

/**
 *
 */

  void Stats::collect_invalid_vehicle(Vehicle& vehicle)
  {

    if (not vehicle.is_free())
    {
      timeval ingress_tiemstamp = vehicle.get_ingress_timestamp();
      timeval egress_tiemstamp = vehicle.get_egress_timestamp();
      unsigned int duration = (GET_TS(egress_tiemstamp)-GET_TS(ingress_tiemstamp));

      printf("<<*** vehicle (%d) leaving: INVALID distance=%.3f duration=%u life=%d\n",
             vehicle.get_id(),
             vehicle.get_covered_distance(),
             duration,
             vehicle.get_life());

      /** get time(in milliseconds) and the distance (en meters)*/
      vehicle.set_speed(0); //km/sec
      last_vehicles.push(vehicle);
      database.store(vehicle);
    }
  }

  void Stats::collect_stats(Vehicle& vehicle){

   if (not vehicle.is_free())
   {
      Tsegment trajectory;
      Vec2f trajectory_polar_eq;
      float r,theta;

      /** get time(in milliseconds) and the distance (en meters)*/
      float distance = vehicle.get_covered_distance();
      timeval ingress_tiemstamp = vehicle.get_ingress_timestamp();
      timeval egress_tiemstamp = vehicle.get_egress_timestamp();
      unsigned int duration = (GET_TS(egress_tiemstamp)-GET_TS(ingress_tiemstamp));
      vehicle.set_speed(3600*(distance/duration)); //km/sec

      if (distance>0)
      {
         trajectory.orig = vehicle.get_first_2d_center();
         trajectory.end = vehicle.get_2d_center();
         get_polar_repr(&trajectory, &r, &theta);

         trajectory_polar_eq[0] = r;
         trajectory_polar_eq[1] = theta;
         vehicles_trajectories.push_back(trajectory_polar_eq);
         num_trajectories++;
      }

      printf("<<<< vehicle (%d) leaving: distance=%.3f duration=%u spd=%.3f life=%d r=%2.f theta=%2.f\n",
             vehicle.get_id(),
             distance,
             duration,
             vehicle.get_speed(),
             vehicle.get_life(),
             r,
             theta);

      printf("                      ingress_time=%lu egress_time=%lu  in(%.2f:%.2f:%.2f) out(%.2f:%.2f:%.2f)\n",
             GET_TS(ingress_tiemstamp),
             GET_TS(egress_tiemstamp),
             vehicle.get_first_3d_center().X,
             vehicle.get_first_3d_center().Y,
             vehicle.get_first_3d_center().Z,
             vehicle.get_current_3d_center().X,
             vehicle.get_current_3d_center().Y,
             vehicle.get_current_3d_center().Z);

      printf("                      MC=%.2f CA=%.2f VA=%.2f TR=%.2f \n\n",
             vehicle.get_class_prob(MOTORCYCLE),
             vehicle.get_class_prob(CAR),
             vehicle.get_class_prob(VAN),
             vehicle.get_class_prob(TRUCK));

      inc_model_count(vehicle.get_matched_class());

      /** Store this vehicle for stats purpose*/
      if (!m_classification_active || (vehicle.get_matched_class() != INVALID_VEHICLE_CLASS))
      {
        last_vehicles.push(vehicle);
        database.store(vehicle);
      }
   }
   else
   {
     printf("Error: Collecting stats of free vehicle! id=%d\n",vehicle.get_id());
   }
}


/**
 *
 */
Vehicle* Stats::get_next_vehicle(){
   return last_vehicles.pull();
}

/**
 *
 */
int Stats::get_model_count(tvehicle_category category){
   return models_counts[category];
}

/**
 *
 */
void Stats::reset_stats()
{
   memset(models_counts, 0, sizeof(models_counts));
   last_vehicles.reset();
   vehicles_trajectories.clear();
   num_trajectories=0;
}

/**
 *
 */
void Stats::inc_model_count(tvehicle_category category){
   models_counts[category]++;
}
}
