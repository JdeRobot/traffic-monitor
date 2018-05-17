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

#ifndef CARCLASSIER_VIEW_H
#define CARCLASSIER_VIEW_H

#include <gtkmm.h>
#include <gtkmm/socket.h>
#include <gdkmm.h>
#include <colorspacesmm.h>
#include "model.h"
#include "widget.h"
#include "arrow.h"
#include "vehicle.h"
#include "view.h"

namespace trafficmonitor
{
class ViewGtk: public View
{
public:
   ViewGtk(Model& _model, const std::string& gladepath) throw();
   virtual ~ViewGtk() throw() {}
   void update(const jderobotutil::Subject* o, jderobotutil::ObserverArg* arg = 0);

private:
   void drawImage(const colorspaces::Image& image, Gtk::DrawingArea* drawingArea);
   void drawImage(const colorspaces::Image& image, cv::Rect roi, Gtk::DrawingArea* drawingArea);
   void drawImage2(const colorspaces::Image& image, const Cairo::RefPtr<Cairo::Context>& cr);


   void update_stats();
   void update_vehicles_counts();
   void update_ctl();
   void updateComboboxBGModelItems();
   void updateComboboxVehicleSelection();
   void onComboboxModelSelection();
   void onComboboxVehicleSelection();
   void onComboboxVehicleClassSelection();
   int  out_of_image(const Tpolygon* pol);

   void onCameraCalibToggled();

   //main window
   bool onDrawingAreaMainExposeEvent(const Cairo::RefPtr<Cairo::Context> &);
   bool onDrawingAreaBgImageExposeEvent(const Cairo::RefPtr<Cairo::Context> &);
   bool onDrawingAreaRearExposeEvent(const Cairo::RefPtr<Cairo::Context> &);

   bool onDrawingAreaMainButtonPressEvent(GdkEventButton* event);
   bool onDrawingAreaMainButtonReleaseEvent(GdkEventButton* event);
   bool onDrawingAreaButtonPressEvent(GdkEventButton* event);
   bool onMotionEvent(GdkEventMotion* const& event);

   void MyCanny(Mat&src, Mat& dst);

   //Menu connections
   void onCheckmenuitemShowInputVideoToggled();
   void onCheckmenuitemShowBackgrondToggled();
   void onCheckmenuitemShowCameraCalibrationToggled();
   void on_bg_model_selection_change();

   //Button connections
   void on_reset_stats_pressed();
   void on_capture_background_pressed();
   void on_play_pressed();
   void on_restore_camera_config_pressed();
   void on_save_config_pressed();
   void on_save_camera_config_pressed();

   //Toggle Button connections
   void on_show_tracking_zone_toggled();
   void on_show_detection_zone_toggled();
   void on_camera_heigth_slider_change();
   void on_detection_zone_slider_change();
   int find_the_nearest_vehicle(const vector<Blob*>& vehicles, Tpoint2D p);
   void onComboboxBGModelChanged();
   void update_camera_parameters();
   void draw_vehicle_rear(Vehicle* vehicle);
   void draw_vehicles_images();

   //Glade and main window
   Gtk::Main gtkmain;
   Glib::RefPtr<Gtk::Builder> builder;
   Widget<Gtk::Window> mainwindow;
   Widget<Gtk::Window> input_image_window;
   Widget<Gtk::Window> backgrond_window;
   Widget<Gtk::Window> camera_calibration_window;

   //Menu items
   Widget<Gtk::CheckMenuItem> checkmenuitemShowInputVideo;
   Widget<Gtk::CheckMenuItem> checkmenuitemShowBgImage;
   Widget<Gtk::CheckMenuItem> checkmenuitemShowCameraCalibration;

   //Buttons
   Widget<Gtk::ToggleButton> capture;
   Widget<Gtk::ToggleButton> save_config;
   Widget<Gtk::ToggleButton> save_camera_config;
   Widget<Gtk::ToggleButton> restore_camera_config;
   Widget<Gtk::ToggleButton> play;
   Widget<Gtk::ToggleButton> reset_stats;

   //Check buttons
   // View controls
   Widget<Gtk::CheckButton> show_bg_mask;
   Widget<Gtk::CheckButton> show_tracking_zone;
   Widget<Gtk::CheckButton> show_detection_zone;
   Widget<Gtk::CheckButton> show_tracking_info;
   Widget<Gtk::CheckButton> show_categories;
   Widget<Gtk::CheckButton> show_oclusion;
   Widget<Gtk::CheckButton> show_bounding_box;
   Widget<Gtk::CheckButton> show_klt_points;
   Widget<Gtk::CheckButton> show_projections;
   Widget<Gtk::CheckButton> show_tmp_blobs;
   Widget<Gtk::CheckButton> show_extended_info;
   Widget<Gtk::CheckButton> shadow_detection;
   Widget<Gtk::CheckButton> show_orig_bg_mask;
   Widget<Gtk::CheckButton> show_blob_bg;
   Widget<Gtk::CheckButton> show_blob_fg;


   // Model controls
   Widget<Gtk::CheckButton> use_static_road;
   Widget<Gtk::CheckButton> auto_calib;
   Widget<Gtk::CheckButton> classify;
   Widget<Gtk::CheckButton> klt_tracking;
   Widget<Gtk::CheckButton> proximity_tracking;
   Widget<Gtk::CheckButton> pixeltrack_tracking;
   Widget<Gtk::CheckButton> switch_detection_zone;
   Widget<Gtk::CheckButton> advanced_detection;

   //Text views
   Widget<Gtk::TextView> vehicles_stats;
   Widget<Gtk::TextView> moto_view;
   Widget<Gtk::TextView> car_view;
   Widget<Gtk::TextView> van_view;
   Widget<Gtk::TextView> truck_view;
   Widget<Gtk::TextView> bus_view;
   Widget<Gtk::TextView> total_view;
   Widget<Gtk::TextView> theta_view;
   Widget<Gtk::TextView> psi_view;
   Widget<Gtk::TextView> camera_height_view;
   Widget<Gtk::TextView> focal_distance_view;
   Widget<Gtk::TextView> tilt_view;
   Widget<Gtk::TextView> pan_view;
   Widget<Gtk::TextView> roll_view;
   Widget<Gtk::TextView> p1_coordinates_view;
   Widget<Gtk::TextView> p2_coordinates_view;

   //Vertical/horizontal bars
   Widget<Gtk::Scrollbar> detection_zone_perc_bar;
   Widget<Gtk::Scrollbar> camera_height_bar;

   //Drawing areas
   Widget<Gtk::DrawingArea> drawingarea_input_image;
   Widget<Gtk::DrawingArea> drawingarea_bg_image;
   Widget<Gtk::DrawingArea> drawingarea_rear;

   //algorithm selection dialog
   //Tree model columns for comboboxBGModel
   class ModelColumns : public Gtk::TreeModel::ColumnRecord
   {
   public:
      ModelColumns()
         { add(m_col_name);
            add(m_col_desc); }

      Gtk::TreeModelColumn<std::string> m_col_name;
      Gtk::TreeModelColumn<std::string> m_col_desc;
   };

   Glib::RefPtr<Gtk::ListStore> comboboxBGModelLSRef;
   Widget<Gtk::ComboBox> comboboxBGModel;

   Glib::RefPtr<Gtk::ListStore> comboboxVehicleSelectionLSRef;
   Widget<Gtk::ComboBox> comboboxVehicleSeletion;

   Glib::RefPtr<Gtk::ListStore> comboboxVehicleClassSelectionLSRef;
   Widget<Gtk::ComboBox> comboboxVehicleClassSeletion;

   //Other auxiliar vars
   cv::Point mousePoint;
   float camera_height;
   tvehicle_category selected_model_for_debuging;
};

}//namespace

#endif /*CARCLASSIER_VIEW_H*/
