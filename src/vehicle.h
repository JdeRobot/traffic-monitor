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

#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "blob.h"
#include "vehicle_model.h"
#include "planar_geom.h"

namespace trafficmonitor{

class Vehicle: public Blob{

public:

   /**
    *
    */
   Vehicle();

   /**
    *
    */
   Vehicle(const Blob &obj);

   /**
    *
    */
   virtual void init();

   /**
    *
    */
   tvehicle_category get_matched_class() const;

   /**
    *
    */
   void set_category(tvehicle_category new_category){category = new_category;};

   /**
    *
    */
   void set_projection(Tpolygon* p_projection);

   /**
    *
    */
   void inc_class_prob(tvehicle_category veh, float class_prob);

   /**
    *
    */
   float get_class_prob(tvehicle_category category);

   /**
    *
    */
   virtual void end_of_tracking(timeval timestamp);

   /**
    *
    */
   virtual void start_tracking(timeval timestamp);

   /**
    *
    */
   void set_speed(float new_speed){speed=new_speed;};
   float get_speed() const {return speed;};

   /**
    *
    */
   const Tpolygon* get_projection(){return &projection;};

private:
   
   /** This field is updated by the best matched class in each iteration.*/
   int category;

   /* The first_v3d_center,current_v3d_center and in_timestamp are used to estimate the blob speed*/
   float speed;

   /** This array is used to accumulated the probability for each class. In each iteration
    *  the probability for each class is calculated and accumulated on this array. At the
    *  end of the tracking zone, the vehicle class is selected computing the class which has
    *  the maximum accumulated probability.
    */
   float categories_prob_acm[MAX_MODELS];

   /** Store the 2D projection of the 3D vehicle in each classification iteration*/
   Tpolygon projection;
};
}

#endif
