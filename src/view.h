/*
 *
 *  Copyright (C) 1997-2010 JDERobot Developers Team
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
 *  Authors : Redouane Kacahch <redouane.kachach@gmail.com>
 *
 */

#ifndef VIEW_H
#define VIEW_H

#include <observer.h>
#include <cairomm/context.h>
#include <pangomm/context.h>

#include "model.h"
#include "vehicle.h"
#include "blob.h"

namespace trafficmonitor
{
class View;
typedef std::tr1::shared_ptr<View> ViewPtr;

class View: public jderobotutil::Observer
{
public:
   View(Model& _model) throw(): model(_model) {};
   virtual ~View() throw() {};
   virtual void update(const jderobotutil::Subject* o, jderobotutil::ObserverArg* arg = 0) = 0;

protected:
   Model& model;

   void draw_tmp_blobs(Cairo::RefPtr<Cairo::Context> cr, const vector<Blob*>& blobs );
   void draw_vehicles(Cairo::RefPtr<Cairo::Context> cr, const vector<Blob*>& vehicles);
   void show_road_direction(Cairo::RefPtr<Cairo::Context> cr);
   void display_tracking_zone(Cairo::RefPtr<Cairo::Context> cr);
   void display_detection_zone(Cairo::RefPtr<Cairo::Context> cr);
   void show_vehicle_description(Cairo::RefPtr<Cairo::Context> cr, Vehicle* vehicle);
   void show_vehicle_trajectory(Cairo::RefPtr<Cairo::Context> cr, Vehicle* vehicle);
   void show_blob_description(Cairo::RefPtr<Cairo::Context> cr, Blob* blob);
   void draw_vehicle_projection(Cairo::RefPtr<Cairo::Context> cr, const Tpolygon* polygon);
   void draw_line(Cairo::RefPtr<Cairo::Context> cr, const Tpoint2D& a, const Tpoint2D& b);
   int  out_of_image(const Tpolygon* pol);
   void show_camera_calibration_rectangle(Cairo::RefPtr<Cairo::Context> cr);
};

} //namespace

#endif
