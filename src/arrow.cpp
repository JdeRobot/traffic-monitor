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
 *  Authors : http://kapo-cpp.blogspot.com/2008/10/drawing-arrows-with-cairo.html
 *
 */
#include "arrow.h"

namespace trafficmonitor{

void ArrowHead::calcVertexes(double start_x, double start_y, double end_x, double end_y, double& x1, double& y1, double& x2, double& y2){
   double angle = atan2 (end_y - start_y, end_x - start_x) + M_PI;

   x1 = end_x + arrow_lenght_ * cos(angle - arrow_degrees_);
   y1 = end_y + arrow_lenght_ * sin(angle - arrow_degrees_);
   x2 = end_x + arrow_lenght_ * cos(angle + arrow_degrees_);
   y2 = end_y + arrow_lenght_ * sin(angle + arrow_degrees_);
}

void ArrowOpen::draw(Cairo::RefPtr< Cairo::Context > context_ref, double start_x, double start_y, double end_x, double end_y){
   double x1;
   double y1;
   double x2;
   double y2;

   calcVertexes (start_x, start_y, end_x, end_y, x1, y1, x2, y2);

   //context_ref->set_source_rgb (line_color_.get_red_p(), line_color_.get_blue_p(), line_color_.get_green_p());

   context_ref->move_to (end_x, end_y);
   context_ref->line_to (x1, y1);
   context_ref->stroke();

   context_ref->move_to (end_x, end_y);
   context_ref->line_to (x2, y2);
   context_ref->stroke();
}

void ArrowSolid::draw(Cairo::RefPtr< Cairo::Context > context_ref, double start_x, double start_y, double end_x, double end_y){
   double x1;
   double y1;
   double x2;
   double y2;

   calcVertexes (start_x, start_y, end_x, end_y, x1, y1, x2, y2);

   context_ref->move_to (end_x, end_y);
   context_ref->line_to (x1, y1);
   context_ref->line_to (x2, y2);
   context_ref->close_path();

   //context_ref->set_source_rgb (line_color_.get_red_p(), line_color_.get_blue_p(), line_color_.get_green_p());
   context_ref->stroke_preserve();

   //context_ref->set_source_rgb (fill_color_.get_red_p(), fill_color_.get_blue_p(), fill_color_.get_green_p());
   context_ref->fill();
}

}//namespace
