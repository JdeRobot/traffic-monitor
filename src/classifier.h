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

#ifndef _CLASSIFIER_
#define _CLASSIFIER_

#include "vehicle.h"
#include "progeo.h"
#include "planar_geom.h"
#include "svm.h"

namespace trafficmonitor{

class Classifier {

public:

   /**
    *
    */
   Classifier();

   /**
    *
    */
   void classify_vehicle(Vehicle *vehicle, colorspaces::Image& inputImage);

   /**
    *
    */
   static float calculate_model_prob(Vehicle* vehicle, Tpoint2D center, tvehicle_category category);

   /**
    *
    */
   static void update_sun_pos(float theta_angle, float psi_angle);

   /**
    *
    */
   static void get_smallest_vehicle_dimension(unsigned int* dimensions, float* length, Tpoint2D center);

   /**
    *
    */
   static bool project_vehicle(Tpoint2D center, tvehicle_category category, Tpolygon* model_proj);

   /**
    *
    */
   static void project_model_and_shadow(tvehicle_category category, 
                                        HPoint3D center,
                                        Tpolygon* polygong);

private:

   /**
    *
    */
   float exp_prob(int a, int b, int c);

   /**
    *
    */
   inline static bool point_in_model_projection(Tpolygon *projection, Tpoint2D p);
   static bool cv_point_in_model_projection(Tpolygon *projection, Tpoint2D p);


   /**
    *
    */
   static int point_in_model_projection_no_shadows(Tpolygon *projection, Tpoint2D p);

   /**
    *
    */
   static void set_orig_end(Tsegment* s, HPoint2D a, HPoint2D b);

   /**
    *
    */
   static void get_shadow(HPoint3D A, HPoint3D* shadow);

   /**
    *
    */
   static int shadows_are_present();
   
   /**
    *
    */
   void project_model(T3dmodel model, HPoint3D center, Tpolygon* polygon);

   /**
    *
    */
   static void match_model_projection_with_blob(Vehicle *vehicle, 
                                                Tpolygon* projection,
                                                int* blob_minos_hyp,
                                                int* blob_and_hyp,
                                                int* hyp_minos_blob,
                                                int* hyp,
                                                int* blob);
   
   /**
    *
    */
   void shift_model(Tpolygon *polygon, int delta_u, int delta_v);
   
   /**
    *
    */
   void test_pip_projection(Tpoint2D p);
   
   /** Shadow Modeling */
   TrafficMonitorSVM m_svm_classifier;
   static Tpoint3D sun_pos;
   static float sun_radius;
   Tpolygon models_projection[MAX_MODELS];

};
}
#endif
