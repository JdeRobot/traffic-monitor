/*
 *
 *  Copyright (C) 2016 Kachach Redouane / David Lobato Bravo
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
 *  Authors : redouane kachach <redouane.kachach@gmail.com>
 *  Authors : David Lobato Bravo <dav.lobato@gmail.com>
 *
 */
#include <stdlib.h>
#include <string>
#include <gtkmm.h>
#include <gdkmm.h>
#include <iomanip>

#include "feature_tracking.h"
#include "classifier.h"
#include "road_detection.h"
#include "background_model.h"
#include "trafficmonitor_algorithm.h"
#include "camera_model.h"
#include "planar_geom.h"
#include "movement_model.h"
#include "datatypes.h"
#include "view.h"
#include "arrow.h"

using namespace std;

namespace trafficmonitor {

#define IN_IMAGE(p, image) (p.x>=0 && p.x<image.width && p.y>=0 && p.y<image.height)

static const char* models_colors[MAX_MODELS] = {"Red", "Yellow", "Green", "Blue", "Pink", "Aqua"};

/**
 *
 */
void View::draw_tmp_blobs(Cairo::RefPtr<Cairo::Context> cr, const vector<Blob*>& blobs ){

   cr->save();
   cr->set_line_width(1.0);
   Gdk::Color color("Green");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1);

   for (int i=0; i<blobs.size() ; i++)
   {
      /** We only draw the temporal blobs that appears on the current frame but haven't
       *  been asociated with any tracked blob (already_machted check).
       */
      if (blobs[i]->is_free())
         continue;

      Gdk::Color color;

      if (blobs[i]->tracking_blobs_id.size()>1)
         color = Gdk::Color("Orange");
      else
         color = Gdk::Color("Blue");

      cr->set_source_rgba(color.get_red_p(),
                          color.get_green_p(),
                          color.get_blue_p(),
                          1.0);

      cr->rectangle(blobs[i]->get_left_corner().x,
                    blobs[i]->get_left_corner().y,
                    blobs[i]->get_right_corner().x-blobs[i]->get_left_corner().x,
                    blobs[i]->get_right_corner().y-blobs[i]->get_left_corner().y);

      color = Gdk::Color("Yellow");
      cr->set_source_rgba(color.get_red_p(),
                          color.get_green_p(),
                          color.get_blue_p(),
                          1.0);

      for(int k=0; k<blobs[i]->contour.size(); k++)
      {
         cr->rectangle(blobs[i]->contour[k].x,
                       blobs[i]->contour[k].y,
                       1, 1);
      }

      // cr->fill();

   }//FOR

   cr->stroke();
   cr->restore();
}

/**
 *
 */
void View::draw_line(Cairo::RefPtr<Cairo::Context> cr, const Tpoint2D& a, const Tpoint2D& b)
{
   cr->move_to(a.x, a.y);
   cr->line_to(b.x, b.y);
}

/**
 *
 */
int View::out_of_image(const Tpolygon* pol){

   int i=0;
   const colorspaces::Image& img = model.getImage();

   for (i=0; i<MAX_SEGMENTS; i++)
   {
      if (!(IN_IMAGE(pol->segments[i].orig, img)
            &&
            IN_IMAGE(pol->segments[i].end, img)))
      {
         return 1;
      }
   }

   return 0;
}


/**
 *
 */
void View::draw_vehicle_projection(Cairo::RefPtr<Cairo::Context> cr, const Tpolygon* polygon)
{
   if (out_of_image(polygon))
      return;

   Tpoint2D a,b,c,d,e,f,g,h;
   Tpoint2D as,bs,cs,ds,es,fs,gs,hs;

   cr->save();
   cr->set_line_width(1.5);

   a = polygon->segments[0].orig;
   b = polygon->segments[0].end;
   c = polygon->segments[1].end;
   d = polygon->segments[2].end;
   e = polygon->segments[4].orig;
   f = polygon->segments[4].end;
   g = polygon->segments[5].end;
   h = polygon->segments[6].end;

   /** Draw model center*/
   Gdk::Color color("Blue");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1.0);

   cr->rectangle(polygon->center.x, polygon->body_center.y, 4, 4);

   color = Gdk::Color("Green");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1.0);
   draw_line(cr, a,b);
   draw_line(cr, c,b);
   draw_line(cr, c,d);
   draw_line(cr, a,d);

   draw_line(cr, e,f);
   draw_line(cr, f,g);
   draw_line(cr, g,h);
   draw_line(cr, h,e);

   draw_line(cr, a,e);
   draw_line(cr, b,f);
   draw_line(cr, c,g);
   draw_line(cr, d,h);

   cr->stroke();
}

/**
 *
 */
void View::show_vehicle_trajectory(Cairo::RefPtr<Cairo::Context> cr, Vehicle* vehicle)
{
   cr->save();
   cr->set_line_width(2.0);
   Gdk::Color color = Gdk::Color("Yellow");
   cr->set_source_rgb(color.get_red_p(),
                      color.get_green_p(),
                      color.get_blue_p());

   Vec4f line;
   if (fit_line(vehicle->m_locationHistory, line, MIN_POINTS_FOR_LINE_FIT))
   {
      cr->save();

      // cr->set_line_width(2.0);
      // Gdk::Color color = Gdk::Color("Red");
      // cr->set_source_rgb(color.get_red_p(),
      //                    color.get_green_p(),
      //                    color.get_blue_p());

      // cr->move_to(line[2], line[3]);
      // cr->line_to(line[2]+line[0]*100, line[3]+line[1]*100);

      float w = min (max(vehicle->get_rect().height/2, 4), 15);
      float h = w/2;
      cr->save();
      cr->translate((int)vehicle->get_2d_center().x, (int)vehicle->get_2d_center().y);
      cr->rotate(atanf(line[1]/line[0]));
      cr->scale(w / 2.0, h / 2.0);

      color = Gdk::Color("Yellow");
      cr->set_source_rgba(color.get_red_p()
                          ,color.get_green_p()
                          ,color.get_blue_p()
                          ,0.5);

      cr->arc(0.0, 0.0, 1.0, 0.0, 2 * M_PI);
      cr->stroke();
      cr->restore();
   }
   else
   // {
   //    // float w = min(max(vehicle->get_rect().height, 4), 20);
   //    float w = 20;
   //    cr->save();
   //    cr->translate((int)vehicle->get_2d_center().x, (int)vehicle->get_2d_center().y);
   //    cr->scale(w / 2.0, w / 2.0);

   //    color = Gdk::Color("Yellow");
   //    cr->set_source_rgba(color.get_red_p()
   //                        ,color.get_green_p()
   //                        ,color.get_blue_p()
   //                        ,0.5);

   //    cr->arc(0.0, 0.0, 1.0, 0.0, 2 * M_PI);
   //    cr->stroke();
   //    cr->restore();
   // }

   // for(int i=0; i<vehicle->m_locationHistory.size(); i++)
   // {
   //    Point p = vehicle->m_locationHistory[i];
   //    if (p.x>0 && p.y>0)
   //    {
   //       cr->rectangle((int)p.x, (int)p.y, 3, 3);
   //       cr->fill();
   //    }
   // }

   cr->stroke();
   cr->restore();
}


/**
 *
 */
void View::show_vehicle_description(Cairo::RefPtr<Cairo::Context> cr, Vehicle* vehicle)
{
   Gdk::Color color;
   const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();
   Glib::RefPtr<Pango::Layout> pangoLayout = Pango::Layout::create (cr);
   Pango::FontDescription font_descr( "serif,monospace bold" );
   pangoLayout->set_font_description( font_descr);

   cr->save();
   cr->set_line_width(1.0);
   if (cfg.showOclusion and vehicle->ocluded)
   {
      color = Gdk::Color("Red");
      cr->set_source_rgb(color.get_red_p(),
                         color.get_green_p(),
                         color.get_blue_p());
   }
   else
   {
      color = Gdk::Color("Yellow");
      cr->set_source_rgb(color.get_red_p(),
                         color.get_green_p(),
                         color.get_blue_p());
   }

   /** Show the blob center and a description. This is build depending on the configuration:
    *  when we are only tracking then the description only contains the vehicle id and its
    *  speed however when classification is active also then the description contains the
    *  vehicle category also.
    */

   if (cfg.showTrackingInfo && (cfg.trackingIsActive() || cfg.classify))
   {
      /** If only tracking then we show only the vehicle identifier, else if
       *  classification is active then show the vehicle classes also. we use the
       *  following ecuacion to get how many digits has the vehicle id. The > 0 check is
       *  to avoid applying log to a negative or 0 value.
       */
      std::stringstream s;
      cr->move_to(vehicle->get_2d_center().x, vehicle->get_2d_center().y);

      if (cfg.classify)
      {
         if (cfg.showTrackingInfo)
         {
            s << vehicle->get_id();

            if (cfg.showCategories)
            {
               s << " - "
                 << VehicleModel::get_model_desc(vehicle->get_matched_class());
            }

            if (cfg.showExtendedInfo)
            {
               double w,h;
               vehicle->get_metric_size(w,h);
               s << "\n"
                 << std::fixed
                 << std::setw( 2 )
                 << std::setprecision( 0 )
                 << vehicle->density()
                 << "\n"
                 << std::fixed
                 << std::setw( 2 )
                 << std::setprecision( 2 )
                 << w
                 << " - "
                 << h
                 << "\n"
                 << "2dc "
                 << vehicle->get_2d_center().x
                 << " - "
                 << vehicle->get_2d_center().y
                 << "\n"
                 << "mass "
                 << vehicle->get_center_of_mass().x
                 << " - "
                 << vehicle->get_center_of_mass().y;
            }
         }

      }
      else if (cfg.trackingIsActive())
      {
         s << vehicle->get_id();
      }

      cr->rectangle((int)vehicle->get_2d_center().x,
                    (int)vehicle->get_2d_center().y,
                    4,4);


      pangoLayout->set_text(s.str());
      pangoLayout->update_from_cairo_context(cr);  //gets cairo cursor position
      pangoLayout->add_to_cairo_context(cr);       //adds text to cairos stack of stuff to be drawn

      cr->fill();
   }

   cr->stroke();
   cr->restore();
}

/**
 *
 */
void View::show_blob_description(Cairo::RefPtr<Cairo::Context> cr, Blob* blob)
{
   Gdk::Color color;
   const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();
   Glib::RefPtr<Pango::Layout> pangoLayout = Pango::Layout::create (cr);
   Pango::FontDescription font_descr( "sans light 11" );
   pangoLayout->set_font_description( font_descr);

   cr->save();
   cr->set_line_width(1.0);

   /** Show the blob center and a description. This is build depending on the configuration:
    *  when we are only tracking then the description only contains the blob id and its
    *  speed however when classification is active also then the description contains the
    *  blob category also.
    */

   if (cfg.showTrackingInfo && (cfg.trackingIsActive() || cfg.classify))
   {
      /** If only tracking then we show only the blob identifier, else if
       *  classification is active then show the blob classes also. we use the
       *  following ecuacion to get how many digits has the blob id. The > 0 check is
       *  to avoid applying log to a negative or 0 value.
       */
      std::stringstream s;
      cr->move_to(blob->get_2d_center().x, blob->get_2d_center().y);
      s << blob->get_id();

      pangoLayout->set_text(s.str());
      pangoLayout->update_from_cairo_context(cr);  //gets cairo cursor position
      pangoLayout->add_to_cairo_context(cr);       //adds text to cairos stack of stuff to be drawn
   }

   cr->stroke();
   cr->restore();
}

/**
 *
 */
void View::show_road_direction(Cairo::RefPtr<Cairo::Context> cr)
{
   cr->save();
   cr->set_line_width(1.5);

   Gdk::Color color("Green");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1);

   Road *road = Road::Instance();

   if (road)
   {
      ArrowOpen openArrow;
      Tpoint2D orig, end;
      orig = road->orig;
      end = road->end;
      cr->move_to(orig.x, orig.y);
      cr->line_to(end.x, end.y);
      openArrow.draw(cr, orig.x, orig.y, end.x, end.y);
   }

   cr->stroke();
   cr->restore();
}

/**
 *
 */
void View::draw_vehicles(Cairo::RefPtr<Cairo::Context> cr, const vector<Blob*>& vehicles)
{
   unsigned int i=0;
   const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();

   cr->save();
   cr->set_line_width(1.5);

   /** Draw tracked blogs*/
   for (i=0; i<vehicles.size(); i++)
   {
      Vehicle* vehicle = (Vehicle*)vehicles[i];

      if (vehicle->is_free() or (! vehicle->is_being_tracked()))
         continue;

      if (cfg.classify and cfg.showProjections and !vehicle->ocluded)
      {
        draw_vehicle_projection(cr, vehicle->get_projection());
      }

      if (cfg.showTrackingInfo)
      {
         /** Draw the Vehicle rectangle. If the classification is active then we use a different
          *  color for each category.
          */
         if ((!cfg.showCategories && cfg.classify) || (cfg.trackingIsActive() && !cfg.classify))
         {
            string color_desc = cfg.classify ? models_colors[vehicle->get_matched_class()] : "Green";
            if (!vehicle->ocluded)
            {
               Gdk::Color color(color_desc);
               cr->set_source_rgba(color.get_red_p(),
                                   color.get_green_p(),
                                   color.get_blue_p(),
                                   1.0);

               // for(int k=0; k<vehicle->contour.size(); k++)
               // {
               //    cr->rectangle(vehicle->contour[k].x,
               //                  vehicle->contour[k].y,
               //                  2, 2);
               // }
            }

            if (cfg.showBoundingBox)
            {
               Gdk::Color color(color_desc);
               cr->set_source_rgba(color.get_red_p(),
                                   color.get_green_p(),
                                   color.get_blue_p(),
                                   0.5);

               cr->rectangle(vehicle->get_left_corner().x,
                             vehicle->get_left_corner().y,
                             vehicle->get_right_corner().x-vehicle->get_left_corner().x,
                             vehicle->get_right_corner().y-vehicle->get_left_corner().y);

               cr->rectangle(vehicle->get_left_corner().x-1,
                             vehicle->get_left_corner().y-1,
                             vehicle->get_right_corner().x-vehicle->get_left_corner().x+2,
                             vehicle->get_right_corner().y-vehicle->get_left_corner().y+2);
            }
         }

         cr->fill();

         show_vehicle_description(cr, vehicle);
         show_vehicle_trajectory(cr,vehicle);
      }

      if (cfg.showKltPoints)
      {
         for(int k=0; k<MAX_COUNT; k++)
         {
            Gdk::Color color("Red");
            cr->set_source_rgba(color.get_red_p(),
                                color.get_green_p(),
                                color.get_blue_p(),
                                1);

            cr->rectangle((int)vehicle->points[k].x,
                          (int)vehicle->points[k].y,
                          1, 1);
            cr->fill();
         }
      }

      cr->stroke();
   }

   cr->restore();
}

/**
 *
 */
void View::display_tracking_zone(Cairo::RefPtr<Cairo::Context> cr){

   int i;
   Gdk::Color color;
   Tsegment output_line;
   Road *road = Road::Instance();
   std::vector<Tpoint2D> road_points;

   /** Draw All the road segments output*/
   road->get_tracking_zone_points(road_points);

   cr->save();
   cr->set_line_width(2.0);
   color = Gdk::Color("Yellow");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1.0);

   if (road_points.size()>0)
   {
      cr->move_to(road_points[0].x, road_points[0].y);
      for (i=0; i<road_points.size(); i++)
      {
         cr->line_to(road_points[i].x, road_points[i].y);
      }
      cr->close_path();
   }
   cr->stroke();

   /** Draw just the road output*/
   color = Gdk::Color("Blue");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1.0);

   road->get_tracking_zone_finish_line(&output_line);
   cr->move_to(output_line.orig.x,output_line.orig.y);
   cr->line_to(output_line.end.x,output_line.end.y);
   cr->stroke();

   cr->restore();
}

/**
 *
 */
void View::show_camera_calibration_rectangle(Cairo::RefPtr<Cairo::Context> cr)
{
   int i;
   Gdk::Color color;
   Tsegment output_line;
   PinHoleCamera* camera = PinHoleCamera::Instance();
   const std::vector<Tpoint2D>& calibration_rectangle = camera->get_calibration_rectangle();

   /*
    * Camera calib rectangle points layout:
    *
    *   0_________1
    *   |         |
    *   |         |
    *   |         |
    *   |         |
    *   |         |
    *   3_________2
    *
    */

   cr->save();
   cr->set_line_width(2.0);

   color = Gdk::Color("Yellow");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1.0);

   // Draw segmetn [0,1]
   cr->move_to(calibration_rectangle[0].x, calibration_rectangle[0].y);
   cr->line_to(calibration_rectangle[1].x, calibration_rectangle[1].y);

   // Draw segmetn [2,3]
   cr->move_to(calibration_rectangle[2].x, calibration_rectangle[2].y);
   cr->line_to(calibration_rectangle[3].x, calibration_rectangle[3].y);

   cr->stroke();

   color = Gdk::Color("Red");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1.0);

   // Draw segmetn [1,2]
   cr->move_to(calibration_rectangle[1].x, calibration_rectangle[1].y);
   cr->line_to(calibration_rectangle[2].x, calibration_rectangle[2].y);

   // Draw segmetn [3,0]
   cr->move_to(calibration_rectangle[3].x, calibration_rectangle[3].y);
   cr->line_to(calibration_rectangle[0].x, calibration_rectangle[0].y);

   cr->stroke();

   cr->restore();
}

/**
 *
 */
void View::display_detection_zone(Cairo::RefPtr<Cairo::Context> cr){

   int i;
   Gdk::Color color;
   Road *road = Road::Instance();
   std::vector<Tpoint2D> detection_zone_points;

   /** Draw All the road segments output*/
   road->get_detection_zone_points(detection_zone_points);

   cr->save();
   cr->set_line_width(2.0);
   color = Gdk::Color("Red");
   cr->set_source_rgba(color.get_red_p(),
                       color.get_green_p(),
                       color.get_blue_p(),
                       1.0);

   if (detection_zone_points.size()>0)
   {
      cr->move_to(detection_zone_points[0].x, detection_zone_points[0].y);
      for (i=0; i<detection_zone_points.size(); i++)
      {
         cr->line_to(detection_zone_points[i].x, detection_zone_points[i].y);
      }
      cr->close_path();
   }
   cr->stroke();
   cr->restore();
}

}//namespace
