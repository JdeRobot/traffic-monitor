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
#include <gdkmm.h>
#include <gtkmm.h>

#include "viewfile.h"

using namespace std;

namespace trafficmonitor {

ViewFile::ViewFile(Model& _model) throw (): View(_model), gtkmain(0,0){}

void ViewFile::update(const jderobotutil::Subject* o, jderobotutil::ObserverArg* arg)
{
   static int switch_buff = 0;
   static unsigned int max = 0;
   const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();

   colorspaces::ImageRGB8 img_rgb8(model.getImage());
   Glib::RefPtr<Gdk::Pixbuf> pixbuf = Gdk::Pixbuf::create_from_data((const guint8*) img_rgb8.data,
                                                                    Gdk::COLORSPACE_RGB,
                                                                    false,
                                                                    8,
                                                                    img_rgb8.width,
                                                                    img_rgb8.height,
                                                                    img_rgb8.step);

   // Detect transparent colors for loaded image
   Cairo::Format format = Cairo::FORMAT_RGB24;
   if (pixbuf->get_has_alpha())
   {
      format = Cairo::FORMAT_ARGB32;
   }

   // Create a new ImageSurface
   Cairo::RefPtr<Cairo::ImageSurface> surface = Cairo::ImageSurface::create(format, pixbuf->get_width(), pixbuf->get_height());
   Cairo::RefPtr<Cairo::Context> cr = Cairo::Context::create (surface);
   Gdk::Cairo::set_source_pixbuf (cr, pixbuf, 0.0, 0.0);
   cr->paint();

   if (cfg.showTrackingZone && !cfg.cameraAutoCalibration)
   {
      display_tracking_zone(cr);
      show_road_direction(cr);

      if (cfg.showDetectionZone && !cfg.cameraAutoCalibration)
         display_detection_zone(cr);
   }

   draw_tmp_blobs(cr, model.getState().tmp_blobs);
   draw_vehicles(cr, model.getState().vehicles);

   //TODO: Use variables instead of hardcoded names
   if (switch_buff)
   {
      surface->write_to_png("image1.png");
      int rc = system("ln -s -f image1.png image.png");
   }
   else
   {
      surface->write_to_png("image2.png");
      int rc = system("ln -s -f image2.png image.png");
   }

   switch_buff = !switch_buff;

}
}//namespace
