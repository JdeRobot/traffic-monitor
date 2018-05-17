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

/*
  # camera configuration file syntax

  #extrinsics, position
  positionX 0.0
  positionY 0.0
  positionZ 7000.0
  positionH 1.000000

  #extrinsics, orientation
  FOApositionX 0
  FOApositionY 7680
  FOApositionZ 0
  FOApositionH 1.000000
  roll 0

  #intrinsics
  fx 207.84
  fy 207.84
  skew 0.000000
  u0 120.0
  v0 160.0
  columns 320
  rows 240

*/

#ifndef _CAMER_MODEL_
#define _CAMER_MODEL_

#include <stdio.h>
#include <string>
#include "progeo.h"
#include "singleton.h"
#include "planar_geom.h"
#include <vector>

class PinHoleCamera: public CSingleton<PinHoleCamera>{

public:

   /**
    * This method should be called before operating with the camera
    */
   void config(const std::string camera_cfg_file);

   /**
    *
    */
   void init(const std::vector<Tpoint2D>& road_points);

   /**
    *
    */
   void update_camera_config();

   /**
    *
    */
   void update_camera_heigth(float new_value);

   /**
    *
    */
   float get_focal_distance(){return m_focal_distance_x;};

   /**
    *
    */
   void restore_config();

   /**
    *
    */
   float get_pan(){return m_pan;};

   /**
    *
    */
   float get_roll(){return m_roll;};

   /**
    *
    */
   float get_tilt(){return m_tilt;};

   /**
    *
    */
   Tpoint2D get_vanishing_p1(){return m_vanishing_p1;};

   /**
    *
    */
   Tpoint2D get_vanishing_p2(){return m_vanishing_p2;};

   /**
    *
    */
   void save_config();

   /**
    *
    */
   const std::vector<Tpoint2D>& get_calibration_rectangle(){return calibration_rectangle;};

   /**
    *
    */
   bool autocalib();

   /**
    *
    */
   void move_point(int i, int j);

   /**
    *
    */
   int myproject(HPoint3D in, HPoint2D *out);


   /**
    * This method receives as input a 2D point and produce the 3D repojection
    * of the same using calculating the intersection between the estimated 3D
    * line and the plane z=Z the 3rd param.
    */
   void reproject(HPoint2D in, HPoint3D *out, float Z);

   /**
    *
    */
   void optic2grafic(HPoint2D *in);

   /**
    *
    */
   void grafic2optic(HPoint2D *in);

protected:

   friend class CSingleton<PinHoleCamera>;

private:

   /**
    *
    */
   PinHoleCamera();

   /**
    * Load the camera conf from the text file. This should has the same
    * format as produced by the jderobot extrnsics schema. See above for
    * An example of the file syntax
    */
   void load_cam(std::string fich_in);
   int load_cam_line(FILE *myfile);

   const char* camera_cfg_file;
   TPinHoleCamera camera;
   TPinHoleCamera camera_orig;

   float m_focal_distance_x;
   float m_focal_distance_y;

   float m_tilt;
   float m_pan;
   float m_roll;

   HPoint3D m_camera_pos;
   HPoint3D m_camera_foa;

   Tpoint2D m_vanishing_p1;
   Tpoint2D m_vanishing_p2;

   /**
    * Camera autocalibration variables.
    */
   std::vector<Tpoint2D> calibration_rectangle;

};

#endif
