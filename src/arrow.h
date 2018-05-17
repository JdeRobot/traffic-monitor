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

#ifndef   	CARSPEED_ARROW_H
# define   	CARSPEED_ARROW_H

#include <cairomm/context.h>

namespace trafficmonitor{
class ArrowHead{
public:
   ArrowHead() :
      arrow_lenght_( 20 ),
      arrow_degrees_( 0.5 ) {}
    
   virtual ~ArrowHead(){}
   
   void calcVertexes(double start_x, double start_y, double end_x, double end_y, double& x1, double& y1, double& x2, double& y2);
   virtual void draw(Cairo::RefPtr< Cairo::Context > context_ref, double start_x, double start_y, double end_x, double end_y) = 0; 
protected:
   double arrow_lenght_;
   double arrow_degrees_;
};


class ArrowOpen : public ArrowHead{
public:
   ArrowOpen() :
      ArrowHead() {}

   virtual ~ArrowOpen(){}

   void draw(Cairo::RefPtr< Cairo::Context > context_ref, double start_x, double start_y, double end_x, double end_y);
};

class ArrowSolid : public ArrowHead{
public:
   ArrowSolid() :
      ArrowHead() {}

   virtual ~ArrowSolid(){}

   void draw(Cairo::RefPtr< Cairo::Context > context_ref, double start_x, double start_y, double end_x, double end_y);
};
}//namespace

#endif 	    /* CARSPEED_ARROW_H */
