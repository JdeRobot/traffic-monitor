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
 *  Authors : David Lobato Bravo <dav.lobato@gmail.com>
 *
 */

#include <stdlib.h>
#include <gtkmm.h>
#include <gdkmm.h>
#include <cairomm/context.h>
#include <pangomm/context.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <unistd.h>
#include <sys/time.h>

#include "stats.h"
#include "viewgtk.h"
#include "feature_tracking.h"
#include "classifier.h"
#include "model.h"
#include "road_detection.h"
#include "background_model.h"
#include "trafficmonitor_algorithm.h"
#include "camera_model.h"
#include "planar_geom.h"
#include "movement_model.h"
#include "vehicle_model.h"
#include "datatypes.h"
#include "rectify.h"

using namespace std;

namespace trafficmonitor {

#define M_TO_MM(a)(a*1000)
#define IN_IMAGE(p, image) (p.x>0 && p.x<image.width && p.y>0 && p.y<image.height)
#define TO_DEG(rad_angle) ((rad_angle*180)/CV_PI)

#define LEFT_BUTTON 1
#define RIGHT_BUTTON 3

  static const char* models_colors[MAX_MODELS] = {"Red", "Yellow", "Green", "Blue", "Pink", "Aqua"};

  ViewGtk::ViewGtk(Model& _model, const std::string& gladepath) throw ():
    View(_model),
    gtkmain(0, 0), //FIXME: do we need gtk params?? maybe we can get them from config file
    builder(Gtk::Builder::create_from_file(gladepath)), //FIXME: check for existence

    //main window
    INIT_WIDGET(mainwindow, builder),
    INIT_WIDGET(input_image_window, builder),
    INIT_WIDGET(backgrond_window, builder),
    INIT_WIDGET(camera_calibration_window, builder),

    //MENU
    INIT_WIDGET(checkmenuitemShowInputVideo, builder),
    INIT_WIDGET(checkmenuitemShowBgImage , builder),
    INIT_WIDGET(checkmenuitemShowCameraCalibration, builder),

    //ComboBoxes
    INIT_WIDGET(comboboxBGModel, builder),
    INIT_WIDGET(comboboxVehicleSeletion, builder),
    INIT_WIDGET(comboboxVehicleClassSeletion, builder),

    //Buttons
    INIT_WIDGET(reset_stats, builder),
    INIT_WIDGET(capture, builder),
    INIT_WIDGET(play, builder),
    INIT_WIDGET(show_bg_mask, builder),
    INIT_WIDGET(show_orig_bg_mask, builder),
    INIT_WIDGET(classify, builder),
    INIT_WIDGET(switch_detection_zone, builder),
    INIT_WIDGET(shadow_detection, builder),
    INIT_WIDGET(save_config, builder),
    INIT_WIDGET(save_camera_config, builder),
    INIT_WIDGET(restore_camera_config, builder),

    //Check Buttons
    INIT_WIDGET(show_tracking_zone, builder),
    INIT_WIDGET(show_detection_zone, builder),
    INIT_WIDGET(use_static_road, builder),
    INIT_WIDGET(show_tracking_info, builder),
    INIT_WIDGET(show_categories, builder),
    INIT_WIDGET(show_oclusion, builder),
    INIT_WIDGET(show_bounding_box, builder),
    INIT_WIDGET(show_klt_points, builder),
    INIT_WIDGET(show_projections, builder),
    INIT_WIDGET(show_tmp_blobs, builder),
    INIT_WIDGET(show_extended_info, builder),
    INIT_WIDGET(show_blob_bg, builder),
    INIT_WIDGET(show_blob_fg, builder),
    INIT_WIDGET(auto_calib, builder),
    INIT_WIDGET(klt_tracking, builder),
    INIT_WIDGET(proximity_tracking, builder),
    INIT_WIDGET(pixeltrack_tracking, builder),
    INIT_WIDGET(advanced_detection, builder),

    //Vertical status bars
    INIT_WIDGET(detection_zone_perc_bar, builder),
    INIT_WIDGET(camera_height_bar, builder),

    //Text views
    INIT_WIDGET(vehicles_stats, builder),
    INIT_WIDGET(moto_view , builder),
    INIT_WIDGET(car_view , builder),
    INIT_WIDGET(van_view , builder),
    INIT_WIDGET(truck_view , builder),
    INIT_WIDGET(bus_view , builder),
    INIT_WIDGET(total_view, builder),
    INIT_WIDGET(theta_view, builder),
    INIT_WIDGET(psi_view, builder),
    INIT_WIDGET(camera_height_view, builder),
    INIT_WIDGET(focal_distance_view, builder),
    INIT_WIDGET(tilt_view , builder),
    INIT_WIDGET(pan_view , builder),
    INIT_WIDGET(roll_view , builder),
    INIT_WIDGET(p1_coordinates_view, builder),
    INIT_WIDGET(p2_coordinates_view, builder),

    //Drawing area
    INIT_WIDGET(drawingarea_bg_image, builder),
    INIT_WIDGET(drawingarea_rear, builder),
    INIT_WIDGET(drawingarea_input_image, builder){

    /* initialize random seed: */
    srand ( time(NULL) );

    // Init controls:
    const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();
    use_static_road->set_active(cfg.useStaticRoad);
    auto_calib->set_active(cfg.cameraAutoCalibration);
    classify->set_active(cfg.classify);
    klt_tracking->set_active(cfg.kltTrackingActive);
    proximity_tracking->set_active(cfg.proximityTrackingActive);
    pixeltrack_tracking->set_active(cfg.pixelTrackTrackingActive);
    advanced_detection->set_active(cfg.advancedDetection);
    switch_detection_zone->set_active(cfg.switchDetectionZone);
    shadow_detection->set_active(cfg.shadowDetectionActive);
    show_tracking_zone->set_active(cfg.showTrackingZone);
    show_detection_zone->set_active(cfg.showDetectionZone);
    show_tracking_info->set_active(cfg.showTrackingInfo);
    show_categories->set_active(cfg.showCategories);
    show_bounding_box->set_active(cfg.showBoundingBox);
    show_oclusion->set_active(cfg.showOclusion);
    show_bg_mask->set_active(cfg.showBgMask);
    show_orig_bg_mask->set_active(cfg.showOrigBgMask);
    show_klt_points->set_active(cfg.showKltPoints);
    show_projections->set_active(cfg.showProjections);
    show_extended_info->set_active(cfg.showExtendedInfo);
    show_tmp_blobs->set_active(cfg.showTmpBlobs);
    show_blob_fg->set_active(cfg.showBlobFg);
    show_blob_bg->set_active(cfg.showBlobBg);

    //Drawing areas
    drawingarea_input_image->signal_draw().connect(sigc::mem_fun(this, &ViewGtk::onDrawingAreaMainExposeEvent));
    drawingarea_bg_image->signal_draw().connect(sigc::mem_fun(this, &ViewGtk::onDrawingAreaBgImageExposeEvent));
    drawingarea_rear->signal_draw().connect(sigc::mem_fun(this, &ViewGtk::onDrawingAreaRearExposeEvent));

    // drawingarea_camera_calibration_image->signal_button_press_event().connect(sigc::mem_fun(this, &ViewGtk::onDrawingAreaCameraCalibrationButtonPressEvent));
    drawingarea_input_image->signal_button_press_event().connect(sigc::mem_fun(this, &ViewGtk::onDrawingAreaButtonPressEvent));
    drawingarea_input_image->signal_button_release_event().connect(sigc::mem_fun(this, &ViewGtk::onDrawingAreaMainButtonReleaseEvent));
    drawingarea_input_image->signal_motion_notify_event().connect(sigc::mem_fun(this, &ViewGtk::onMotionEvent));

    //Connect Buttons
    reset_stats->signal_pressed().connect(sigc::mem_fun(this, &ViewGtk::on_reset_stats_pressed));
    capture->signal_pressed().connect(sigc::mem_fun(this, &ViewGtk::on_capture_background_pressed));
    save_config->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::on_save_config_pressed));
    save_camera_config->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::on_save_camera_config_pressed));
    restore_camera_config->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::on_restore_camera_config_pressed));
    play->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::on_play_pressed));

    //Connect Toggle Buttons
    show_bg_mask->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_orig_bg_mask->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_tracking_zone->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_detection_zone->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_tracking_info->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_categories->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_oclusion->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_bounding_box->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_klt_points->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_projections->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_tmp_blobs->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_extended_info->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_blob_bg->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    show_blob_fg->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));

    classify->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    use_static_road->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    switch_detection_zone->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    shadow_detection->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    klt_tracking->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    proximity_tracking->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    pixeltrack_tracking->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));
    advanced_detection->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::update_ctl));

    auto_calib->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::onCameraCalibToggled));

    //Connect Slide bars
    detection_zone_perc_bar->signal_value_changed().connect(sigc::mem_fun(this, &ViewGtk::on_detection_zone_slider_change));
    camera_height_bar->signal_value_changed().connect(sigc::mem_fun(this, &ViewGtk::on_camera_heigth_slider_change));

    //Connect BGmodel selection Box
    updateComboboxBGModelItems();
    updateComboboxVehicleSelection();

    comboboxBGModel->signal_changed().connect(sigc::mem_fun(this, &ViewGtk::onComboboxBGModelChanged));
    comboboxBGModel->set_sensitive(not play->get_active());
    comboboxVehicleSeletion->signal_changed().connect(sigc::mem_fun(this, &ViewGtk::onComboboxVehicleSelection));
    comboboxVehicleClassSeletion->signal_changed().connect(sigc::mem_fun(this, &ViewGtk::onComboboxVehicleClassSelection));

    //Set Menu default configuration
    //
    checkmenuitemShowInputVideo->set_active(true);
    checkmenuitemShowInputVideo->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::onCheckmenuitemShowInputVideoToggled));
    //
    checkmenuitemShowBgImage->set_active(true);
    checkmenuitemShowBgImage->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::onCheckmenuitemShowBackgrondToggled));
    //
    checkmenuitemShowCameraCalibration->set_active(true);
    checkmenuitemShowCameraCalibration->signal_toggled().connect(sigc::mem_fun(this, &ViewGtk::onCheckmenuitemShowCameraCalibrationToggled));

    //update widget values according with model
    mainwindow->show();
    input_image_window->show();

    checkmenuitemShowBgImage->set_active(cfg.showBgMask);
    if (cfg.showBgMask)
    {
      backgrond_window->show();
    }
    else
    {
      backgrond_window->hide();
    }

    checkmenuitemShowCameraCalibration->set_active(false);
    camera_calibration_window->hide();
    detection_zone_perc_bar->set_value(cfg.detectionZonePercentage);
  }

/**
 *
 */
  void ViewGtk::draw_vehicle_rear(Vehicle* vehicle)
  {
    if (vehicle)
    {
#define WITDH 96
#define HEIGHT 128
      Tpoint2D e,f,g,h;
      Tpoint2D inputPoints[4];
      Tpoint2D rectifiedPoints[4];
      const Tpolygon* polygon = vehicle->get_projection();

      e = polygon->segments[4].orig;
      f = polygon->segments[4].end;
      g = polygon->segments[5].end;
      h = polygon->segments[6].end;

      inputPoints[0] = e;
      inputPoints[1] = f;
      inputPoints[2] = g;
      inputPoints[3] = h;

      colorspaces::Image img = model.getImage();
      if (!IN_IMAGE(e, img)
          ||!IN_IMAGE(f, img)
          ||!IN_IMAGE(g, img)
          ||!IN_IMAGE(h, img))
      {
        return;
      }

      rectifiedPoints[0] = Tpoint2D(0,0);
      rectifiedPoints[1] = Tpoint2D(0,WITDH);
      rectifiedPoints[2] = Tpoint2D(HEIGHT,WITDH);
      rectifiedPoints[3] = Tpoint2D(HEIGHT,0);

      colorspaces::Image output_image (cv::Mat::zeros(HEIGHT,WITDH, CV_8UC3), colorspaces::ImageRGB8::FORMAT_RGB8);
      cv::Rect rectifiedRoi = cv::Rect(e.x, e.y, (f.x-e.x), (g.y-f.y));
      colorspaces::Image input_image = model.getImage();

      cv::Rect rect_mat(0, 0, input_image.height, input_image.width);
      bool roi_inside_image = (rectifiedRoi & rect_mat) == rectifiedRoi;

      if (roi_inside_image)
      {
        cv::Mat croppedFaceImage = input_image(rectifiedRoi).clone();

        Rectifier rectifier;
        rectifier.rectify(inputPoints, rectifiedPoints);
        rectifier.rectify(input_image, output_image);

        drawImage(output_image, drawingarea_rear.get());
      }
    }
  }

/**
 *
 */
  bool ViewGtk::onDrawingAreaMainExposeEvent(const Cairo::RefPtr<Cairo::Context>& ctx) {

    static unsigned int max = 0;
    const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();

    //display main image
    drawImage2(model.getImage(), ctx);

    //FIXME:event has a window
    Glib::RefPtr<Gdk::Window> window = drawingarea_input_image->get_window();
    if (window)
    {
      Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();

      if (cfg.showTrackingZone && !cfg.cameraAutoCalibration)
      {
        display_tracking_zone(cr);
        // show_road_direction(cr);

        if (cfg.showDetectionZone && !cfg.cameraAutoCalibration)
          display_detection_zone(cr);
      }

      if (cfg.showKltPoints)
      {
        draw_tmp_blobs(cr, model.getState().tmp_blobs);
      }

      draw_vehicles(cr, model.getState().vehicles);
      update_stats();

      if (cfg.cameraAutoCalibration)
      {
        show_camera_calibration_rectangle(cr);
      }

      bool tracking_is_active = (cfg.kltTrackingActive || cfg.proximityTrackingActive);

      if (cfg.classify || tracking_is_active)
      {
        update_vehicles_counts();
      }
    }

    return true;
  }

  void ViewGtk::draw_vehicles_images()
  {
    const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();
    const vector<Blob*>& vehicles = model.getState().vehicles;

    for (int i=0; i<vehicles.size(); i++)
    {
      Vehicle* vehicle = (Vehicle*)vehicles[i];

      if (vehicle->is_free() or (! vehicle->is_being_tracked()))
      {
        continue;
      }

      if (cfg.showTrackingInfo)
      {
        if (cfg.classify && ((vehicle->get_matched_class() == TRUCK) || (vehicle->get_matched_class() == BUS)))
        {
          draw_vehicle_rear(vehicle);
        }
      }
    }
  }



/**
 *
 */
  void ViewGtk::onCameraCalibToggled()
  {
    if (auto_calib->get_active())
    {
      checkmenuitemShowCameraCalibration->set_active(true);
      camera_calibration_window->show();
    }
    else
    {
      checkmenuitemShowCameraCalibration->set_active(false);
      camera_calibration_window->hide();
    }

    update_ctl();
  }


/**
 *
 */
  bool ViewGtk::onDrawingAreaRearExposeEvent(const Cairo::RefPtr<Cairo::Context>& ctx)
  {
    draw_vehicles_images();
    return true;
  }

/**
 *
 */
  bool ViewGtk::onDrawingAreaBgImageExposeEvent(const Cairo::RefPtr<Cairo::Context>& ctx)
  {
    const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();

    // display main image
    if (cfg.showBgMask)
    {
      const colorspaces::ImageGRAY8 bg_image = colorspaces::Image(cv::Mat(model.getState().background->GetMask()), colorspaces::ImageGRAY8::FORMAT_GRAY8);
      drawImage(bg_image, drawingarea_bg_image.get());
    }
    else if (cfg.showOrigBgMask)
    {
      const colorspaces::ImageGRAY8 bg_image = colorspaces::Image(cv::Mat(model.getState().background->GetMaskOrig()), colorspaces::ImageGRAY8::FORMAT_GRAY8);
      drawImage(bg_image, drawingarea_bg_image.get());
    }
    else if (cfg.showBlobFg)
    {
      const colorspaces::Image bg_image = colorspaces::Image(MovModel::Instance()->get_blob_fg(), colorspaces::ImageRGB8::FORMAT_RGB8);
      drawImage(bg_image, drawingarea_bg_image.get());
    }
    else if (cfg.showBlobBg)
    {
      const colorspaces::ImageGRAY8 bg_image = colorspaces::Image(MovModel::Instance()->get_blob_bg(), colorspaces::ImageGRAY8::FORMAT_GRAY8);
      drawImage(bg_image, drawingarea_bg_image.get());
    }

    //FIXME:event has a window
    Glib::RefPtr<Gdk::Window> window = drawingarea_bg_image->get_window();
    return true;
  }

  void ViewGtk::drawImage2(const colorspaces::Image& image, const Cairo::RefPtr<Cairo::Context>& cr) {

    /*convert to RGB*/
    colorspaces::ImageRGB8 img_rgb8(image);
    Glib::RefPtr<Gdk::Pixbuf> imgBuff = Gdk::Pixbuf::create_from_data((const guint8*) img_rgb8.data,
                                                                      Gdk::COLORSPACE_RGB,
                                                                      false,
                                                                      8,
                                                                      img_rgb8.width,
                                                                      img_rgb8.height,
                                                                      img_rgb8.step);

    Gdk::Cairo::set_source_pixbuf(cr, imgBuff, 0, 0);
    cr->rectangle(0, 0, img_rgb8.width, img_rgb8.height);
    cr->fill();
    drawingarea_input_image->set_size_request(img_rgb8.width, img_rgb8.height);
  }

/**
 *
 */
  void ViewGtk::drawImage(const colorspaces::Image& image, Gtk::DrawingArea* drawingArea) {

    Glib::RefPtr<Gdk::Window> window = drawingArea->get_window();

    if (window)
    {
      /*convert to RGB*/
      colorspaces::ImageRGB8 img_rgb8(image);
      Glib::RefPtr<Gdk::Pixbuf> imgBuff = Gdk::Pixbuf::create_from_data((const guint8*) img_rgb8.data,
                                                                        Gdk::COLORSPACE_RGB,
                                                                        false,
                                                                        8,
                                                                        img_rgb8.width,
                                                                        img_rgb8.height,
                                                                        img_rgb8.step);

      Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
      cr->save();
      Gdk::Cairo::set_source_pixbuf(cr, imgBuff, 0, 0);
      cr->rectangle(0, 0, img_rgb8.width, img_rgb8.height);
      cr->fill();
      cr->restore();

      // // Create an empty GC
      // const Glib::RefPtr<const Gdk::GC> gc;
      // window->draw_pixbuf(gc,
      //                     imgBuff,
      //                     0, 0, /*starting point from imgBuff*/
      //                     0, 0, /*starting point into drawable*/
      //                     imgBuff->get_width(),
      //                     imgBuff->get_height(),
      //                     Gdk::RGB_DITHER_NONE, 0, 0);
      // drawingArea->set_size_request(img_rgb8.width, img_rgb8.height);
    }
  }

/**
 *
 */
  void ViewGtk::drawImage(const colorspaces::Image& image, cv::Rect roi, Gtk::DrawingArea* drawingArea) {
    cv::Mat m(image, roi);
    colorspaces::Image img(m, image.format());
    drawImage(img, drawingArea);
  }

/**
 *
 */
  void ViewGtk::update(const jderobotutil::Subject* o, jderobotutil::ObserverArg* arg) {

    //image drawing is done on expose events send with these calls
    drawingarea_input_image->queue_draw();
    drawingarea_bg_image->queue_draw();
    drawingarea_rear->queue_draw();

    mainwindow->resize(1,1);
    input_image_window->resize(1,1);
    backgrond_window->resize(1,1);
    camera_calibration_window->resize(1,1);

    //FIXME: gui use main thread, we have to move this to another thread
    while (gtkmain.events_pending())
      gtkmain.iteration();
  }

//main window
  bool onDrawingAreaMainButtonPressEvent(GdkEventButton* event){};

/**
 *
 */
  void ViewGtk::onCheckmenuitemShowInputVideoToggled(){
    if (checkmenuitemShowInputVideo->get_active())
      input_image_window->show();
    else
      input_image_window->hide();
  };

/**
 *
 */
  void ViewGtk::onCheckmenuitemShowCameraCalibrationToggled()
  {
    if (checkmenuitemShowCameraCalibration->get_active())
      camera_calibration_window->show();
    else
      camera_calibration_window->hide();
  };

/**
 *
 */
  void ViewGtk::onCheckmenuitemShowBackgrondToggled(){
    if (checkmenuitemShowBgImage->get_active())
      backgrond_window->show();
    else
      backgrond_window->hide();
  };

/**
 *
 */
  bool ViewGtk::onMotionEvent(GdkEventMotion* const& event)
  {
    if (event->state & GDK_BUTTON1_MASK)
    {
      model.processMouseMovement(event->y,event->x);
      update_camera_parameters();
    }

    return true;
  }

/**
 *
 */
  void ViewGtk::update_camera_parameters()
  {
    std::stringstream ss;
    PinHoleCamera* camera = PinHoleCamera::Instance();

    if (camera)
    {
      ss << std::fixed << std::setprecision(2) << camera->get_focal_distance();
      focal_distance_view->get_buffer()->set_text(ss.str());
      ss.str("");

      ss << std::fixed << std::setprecision(2) << TO_DEG(camera->get_pan()) << "º";
      pan_view->get_buffer()->set_text(ss.str());
      ss.str("");

      ss << std::fixed << std::setprecision(2) << TO_DEG(camera->get_tilt()) << "º";
      tilt_view->get_buffer()->set_text(ss.str());
      ss.str("");

      ss << std::fixed << std::setprecision(2) << TO_DEG(camera->get_roll()) << "º";
      roll_view->get_buffer()->set_text(ss.str());
      ss.str("");

      Tpoint p1 = camera->get_vanishing_p1();
      ss << std::fixed << std::setprecision(2) << "(" << p1.x << "," << p1.y << ")" ;
      p1_coordinates_view->get_buffer()->set_text(ss.str());
      ss.str("");

      Tpoint p2 = camera->get_vanishing_p2();
      ss << std::fixed << std::setprecision(2) << "(" << p2.x << "," << p2.y << ")" ;
      p2_coordinates_view->get_buffer()->set_text(ss.str());
      ss.str("");
    }
  }

/**
 *
 */
  bool ViewGtk::onDrawingAreaMainButtonReleaseEvent(GdkEventButton* event){
    model.updateAlgorithmCfg();
    return true;
  }

/**
 *
 */
  int ViewGtk::find_the_nearest_vehicle(const vector<Blob*>& vehicles, Tpoint2D p){

    int vehicle_idx=-1;
    float tmp;

    /** Set the maximum 2D distance posible between 2 pixels on the image (aprox)*/
    float distance = std::numeric_limits<float>::max();

    for (unsigned int i=0; i<vehicles.size(); i++)
    {
      if (!vehicles[i]->is_free())
      {
        tmp = DISTANCE_2D(vehicles[i]->get_2d_center(), p);
        if (tmp<distance)
        {
          distance = tmp;
          vehicle_idx = i;
        }
      }
    }

    return vehicle_idx;
  }

/**
 *
 */
  bool ViewGtk::onDrawingAreaButtonPressEvent(GdkEventButton* event)
  {
    Tpoint2D p;
    HPoint2D proj;
    gint win_x, win_y;
    int i=0;
    float center_u,center_v;
    HPoint3D out;
    PinHoleCamera * camera = PinHoleCamera::Instance();
    const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();

    /*get the window pos*/
    gdk_device_get_window_at_position(gtk_get_current_event_device (), &win_x, &win_y);

    p.y = win_y;
    p.x = win_x;

    proj.x =  win_x;
    proj.y =  win_y;
    camera->reproject(proj, &out, 0);

    printf("\n\n == %d:%d  (%.2f - %.2f - %.2f)-- \n",win_y, win_x, out.X, out.Y, out.Z);

    if (cfg.useStaticRoad)
    {
      if (!IN_IMAGE(p, model.getImage()))
      {
        printf("*** Out of image! *** ");
        return false;
      }
    }

    if (cfg.cameraAutoCalibration)
    {
      if (event->button == RIGHT_BUTTON)
      {
        Tpoint2D selected_center;
        selected_center.x = win_x;
        selected_center.y = win_y;
      }
    }

    printf("Click on button %d (%d:%d)\n",event->button, win_y, win_x);

    return true;
  };

/**
 *
 */
  void ViewGtk::update_stats()
  {
    const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();
    Stats* stats = Stats::Instance();
    std::stringstream stats_text;
    Vehicle* vehicle;
    int iter=0;
    int last_vehicle_pos = stats->get_last_elem_pos();

    stats->rewind();

    while (iter++ < MAX_VEHICLES_HISTORY)
    {
      vehicle = stats->Instance()->get_next_vehicle();

      if (vehicle && !vehicle->is_free())
      {
        if (cfg.classify && (vehicle->get_matched_class() == INVALID_VEHICLE_CLASS))
        {
          continue;
        }
        else if (vehicle->get_speed() <= 0)
        {
          continue;
        }

        string unit = (vehicle->get_speed() > 0)? "  km/h" : "  ---- ";

        if (iter == last_vehicle_pos)
        {
          stats_text << "> "
                     << vehicle->get_id()
                     << " :: "
                     << std::fixed
                     << std::setprecision(2)
                     << vehicle->get_speed()
                     << unit;
        }
        else
        {
          stats_text << "  "
                     << vehicle->get_id()
                     << " :: "
                     << std::fixed
                     << std::setprecision(2)
                     << vehicle->get_speed()
                     << unit;
        }

        /** When classifying we show the vehicle class also*/
        if (cfg.classify)
          stats_text << " -- " << VehicleModel::get_model_desc(vehicle->get_matched_class()) << endl;
        else
          stats_text << endl;

        string color_desc;
        color_desc = models_colors[vehicle->get_matched_class()];
        vehicles_stats->get_buffer()->set_text(stats_text.str());
        Gdk::Color color(color_desc);
        // vehicles_stats->modify_text(Gtk::STATE_NORMAL, color);
        vehicles_stats->show();
      }
    }
  }

/**
 *
 */
  void  ViewGtk::update_vehicles_counts()
  {
    const TrafficMonitorAlgorithmConfig& cfg = model.getAlgorithmCfg();
    Stats* stats = Stats::Instance();
    std::stringstream ss;

    ss << stats->get_model_count(MOTORCYCLE);
    moto_view->get_buffer()->set_text(ss.str());
    ss.str("");

    ss << stats->get_model_count(CAR);
    car_view->get_buffer()->set_text(ss.str());
    ss.str("");

    ss << stats->get_model_count(VAN);
    van_view->get_buffer()->set_text(ss.str());
    ss.str("");

    ss << stats->get_model_count(TRUCK);
    truck_view->get_buffer()->set_text(ss.str());
    ss.str("");

    ss << stats->get_model_count(BUS);
    bus_view->get_buffer()->set_text(ss.str());
    ss.str("");

    int sum=0;
    tvehicle_category veh,init_class;

    if (cfg.classify)
      init_class=MOTORCYCLE;
    else
      init_class=INVALID_VEHICLE_CLASS;


    for (veh=init_class; veh<MAX_MODELS; veh++)
    {
      sum += stats->get_model_count(veh);
    }

    ss << sum;
    total_view->get_buffer()->set_text(ss.str());
    ss.str("");

    moto_view->show();
    car_view->show();
    van_view->show();
    truck_view->show();
    bus_view->show();
    total_view->show();
  }

/**
 *
 */
  void ViewGtk::on_reset_stats_pressed()
  {
    FeatureTracker::Instance()->init();
    Stats::Instance()->reset_stats();
    model.init();
    vehicles_stats->get_buffer()->set_text("");
  }

/**
 *
 */
  void ViewGtk::on_play_pressed(){
    comboboxBGModel->set_sensitive(! play->get_active());
    update_ctl();
  };

/**
 *
 */
  void ViewGtk::on_save_config_pressed(){
    model.saveAlgorithmCfg();
  };

/**
 *
 */
  void ViewGtk::on_save_camera_config_pressed()
  {
    PinHoleCamera* camera = PinHoleCamera::Instance();
    if (camera)
      camera->save_config();
  };

/**
 *
 */
  void ViewGtk::on_restore_camera_config_pressed()
  {
    PinHoleCamera* camera = PinHoleCamera::Instance();
    if (camera)
      camera->restore_config();
  };

/**
 *
 */
  void ViewGtk::on_camera_heigth_slider_change()
  {
    std::stringstream ss;
    PinHoleCamera* camera = PinHoleCamera::Instance();

    camera_height = camera_height_bar->get_value();
    if (camera)
    {
      camera->update_camera_heigth(camera_height);
    }

    ss << camera_height;
    camera_height_view->get_buffer()->set_text(ss.str());

    update_camera_parameters();
  }

/**
 *
 */
  void ViewGtk::on_detection_zone_slider_change(){

    int percentage = (int)detection_zone_perc_bar->get_value();
    std::stringstream ss;
    ss << percentage;

    TrafficMonitorAlgorithmConfig alg_cfg = model.getAlgorithmCfg();
    alg_cfg.detectionZonePercentage = percentage;
    model.setAlgorithmCfg(alg_cfg);
  }

/**
 *
 */
  void ViewGtk::update_ctl()
  {
    TrafficMonitorAlgorithmConfig cfg = model.getAlgorithmCfg();

    // Update model control
    cfg.play                     = play->get_active();
    cfg.classify                 = classify->get_active();
    cfg.useStaticRoad            = use_static_road->get_active();
    cfg.switchDetectionZone      = switch_detection_zone->get_active();
    cfg.shadowDetectionActive    = shadow_detection->get_active();
    cfg.kltTrackingActive        = klt_tracking->get_active();
    cfg.proximityTrackingActive  = proximity_tracking->get_active();
    cfg.pixelTrackTrackingActive = pixeltrack_tracking->get_active();
    cfg.cameraAutoCalibration    = auto_calib->get_active();
    cfg.advancedDetection        = advanced_detection->get_active();

    cfg.showTrackingZone         = show_tracking_zone->get_active();
    cfg.showDetectionZone        = show_detection_zone->get_active();
    cfg.showTrackingInfo         = show_tracking_info->get_active();
    cfg.showCategories           = show_categories->get_active();
    cfg.showBoundingBox          = show_bounding_box->get_active();
    cfg.showOclusion             = show_oclusion->get_active();
    cfg.showBgMask               = show_bg_mask->get_active();
    cfg.showOrigBgMask           = show_orig_bg_mask->get_active();
    cfg.showKltPoints            = show_klt_points->get_active();
    cfg.showExtendedInfo         = show_extended_info->get_active();
    cfg.showProjections          = show_projections->get_active();
    cfg.showTmpBlobs             = show_tmp_blobs->get_active();
    cfg.showBlobBg               = show_blob_bg->get_active();
    cfg.showBlobFg               = show_blob_fg->get_active();

    printf("Updating configurtion \n");

    model.setAlgorithmCfg(cfg);
  }

/**
 *
 */
  void ViewGtk::updateComboboxVehicleSelection(){

    ModelColumns vehicleSelectionCols;
    comboboxVehicleSelectionLSRef = Gtk::ListStore::create(vehicleSelectionCols);
    Gtk::TreeModel::iterator tmIt;

    comboboxVehicleSeletion->clear();
    comboboxVehicleClassSeletion->clear();
    comboboxVehicleSeletion->set_model(comboboxVehicleSelectionLSRef);
    comboboxVehicleClassSeletion->set_model(comboboxVehicleSelectionLSRef);

    Gtk::TreeModel::Row r;
    for (tvehicle_category i=MOTORCYCLE; i<MAX_MODELS; i++)
    {
      tmIt = comboboxVehicleSelectionLSRef->append();
      r = *tmIt;
      r[vehicleSelectionCols.m_col_desc] = VehicleModel::get_model_desc(i);
      r[vehicleSelectionCols.m_col_name] = VehicleModel::get_model_name(i);
    }

    comboboxVehicleSeletion->pack_start(vehicleSelectionCols.m_col_name);
    comboboxVehicleSeletion->pack_start(vehicleSelectionCols.m_col_desc);
    comboboxVehicleClassSeletion->pack_start(vehicleSelectionCols.m_col_name);
    comboboxVehicleClassSeletion->pack_start(vehicleSelectionCols.m_col_desc);
  }

/**
 *
 */
  void ViewGtk::updateComboboxBGModelItems(){

    // BGModelFactory::FactoryDict::const_iterator fIt;

    // ModelColumns comboboxBGModelCols;
    // comboboxBGModelLSRef = Gtk::ListStore::create(comboboxBGModelCols);
    // Gtk::TreeModel::iterator tmIt;

    // comboboxBGModel->clear();
    // comboboxBGModel->set_model(comboboxBGModelLSRef);

    // Gtk::TreeModel::Row r;
    // for(fIt = BGModelFactory::factories.begin(); fIt != BGModelFactory::factories.end(); fIt++)
    // {
    //    tmIt = comboboxBGModelLSRef->append();
    //    r = *tmIt;
    //    r[comboboxBGModelCols.m_col_name] = fIt->first;
    //    r[comboboxBGModelCols.m_col_desc] = fIt->second->description;
    // }

    // comboboxBGModel->pack_start(comboboxBGModelCols.m_col_name);
    // comboboxBGModel->pack_start(comboboxBGModelCols.m_col_desc);
  }

/**
 *
 */
  void ViewGtk::onComboboxBGModelChanged(){
    ModelColumns comboboxBGModelCols;
    Gtk::TreeModel::iterator it = comboboxBGModel->get_active();
    Gtk::TreeModel::Row row = *it;

    TrafficMonitorAlgorithmConfig alg_cfg = model.getAlgorithmCfg();
    alg_cfg.bgalg = row[comboboxBGModelCols.m_col_name];
    model.setAlgorithmCfg(alg_cfg);
  }

/**
 *
 */
  void ViewGtk::onComboboxVehicleSelection(){
    ModelColumns vehicleSelectionCols;
    Gtk::TreeModel::iterator it = comboboxVehicleSeletion->get_active();
    Gtk::TreeModel::Row row = *it;
    selected_model_for_debuging = VehicleModel::get_model_id(row[vehicleSelectionCols.m_col_name]);
    printf("Selected Class %s\n",  VehicleModel::get_model_name(selected_model_for_debuging));
  }

/**
 *
 */
  void ViewGtk::onComboboxVehicleClassSelection(){
    ModelColumns vehicleSelectionCols;
    Gtk::TreeModel::iterator it = comboboxVehicleClassSeletion->get_active();
    Gtk::TreeModel::Row row = *it;
    selected_model_for_debuging = VehicleModel::get_model_id(row[vehicleSelectionCols.m_col_name]);
    printf("Selected Class %s\n",  VehicleModel::get_model_name(selected_model_for_debuging));
  }

//Toggle Button connections
  void ViewGtk::on_capture_background_pressed(){}


}//namespace
