/*
 *  Copyright (C) 2006 José María Cañas Plaza / Kachach Redouane
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *  Authors : José María Cañas Plaza <jmplaza@gsyc.escet.urjc.es>
 *  Authors : Kachach Redouane
 */
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "trafficmonitor_algorithm.h"
#include "camera_model.h"
#include "feature_tracking.h"
#include "proximity_tracking.h"
#include "movement_model.h"
#include "stats.h"
#include "database.h"

namespace trafficmonitor{

#define M_TO_MM(a)(a*1000)
#define vehicle_is_visible(v, image) (!ON_IMAGE_BORDERS(v->get_left_corner(), image) && !ON_IMAGE_BORDERS(v->get_right_corner(), image))
#define ON_IMAGE_BORDERS(p, image) (p.x==0 || p.x==image.width-1 || p.y==0 || p.y==image.height-1)
#define IN_IMAGE(p, image) (p.x>0 && p.x<image.width && p.y>0 && p.y<image.height)

const string TrafficMonitorAlgorithmConfig::FILE_VIEW = "file_view";
const string TrafficMonitorAlgorithmConfig::GTK_VIEW  = "gtk_view";

/**
 *
 */
TrafficMonitorAlgorithm::TrafficMonitorAlgorithm(TrafficMonitorAlgorithmConfig& algcfg)
{
   _state.background = Background::Instance();
   _state.road = Road::Instance();

   // DO NOT change the order (_cfg is set in the init method)
   init(algcfg);

   _state.vehicles.resize(_cfg.get_max_vehicles());
   _state.tmp_blobs.resize(_cfg.get_max_vehicles());

   for(unsigned int i=0; i<_state.vehicles.size(); i++)
   {
      cout << "Creating vehicles and blobs" << endl;
      _state.vehicles[i] = new Vehicle();
      _state.vehicles[i]->init();
      _state.tmp_blobs[i] = new Vehicle();
      _state.tmp_blobs[i]->init();
   }
}

/**
 *
 */
void TrafficMonitorAlgorithm::init()
{
  for(unsigned int i=0; i<_state.vehicles.size(); i++)
  {
    _state.vehicles[i]->init();
    _state.tmp_blobs[i]->init();
  }
}


/**
 *
 */
void TrafficMonitorAlgorithm::init(const TrafficMonitorAlgorithmConfig& new_cfg)
{
   PinHoleCamera *camera = PinHoleCamera::Instance();
   Stats *stats = Stats::Instance();

   // First time, do not update the configuration
   set_cfg(new_cfg, false);

   if (camera)
   {
      camera->config(_cfg.get_camera_filename());
      camera->init(_state.roadPoints);
   }

   if (stats)
   {
      stats->init(_cfg.classify);
   }

   database.connect();
}

/**
 *
 */
void TrafficMonitorAlgorithm::set_cfg(const TrafficMonitorAlgorithmConfig& new_cfg, bool update) throw()
{
    _cfg = new_cfg;

   /** Change the sub-modules configuratin */
   _state.road->set_static_road(_cfg.useStaticRoad);
   _state.road->set_switch_detection_zone(_cfg.switchDetectionZone);
   _state.road->set_detection_zone_perc(_cfg.detectionZonePercentage);
   _state.background->SetShadowDetectionState(_cfg.shadowDetectionActive);

   if (_state.road)
   {
      _state.roadPoints = _cfg.get_road_points();
      _state.road->set_corners(_state.roadPoints);
   }

   // TODO
   // if (camera)
   // {
   //    camera->config(_cfg.get_camera_filename());
   //    camera->init(_state.roadPoints);
   // }

   if (update)
   {
      update_cfg();
   }
};

/**
 *
 */
bool TrafficMonitorAlgorithm::feature_tracking(bool klt_only)
{
   unsigned int min_pixels_in_blob;
   float min_length=0;
   MovModel* movement_mod = MovModel::Instance();
   FeatureTracker* feature_tracker = FeatureTracker::Instance();

   /** Get the minimum length and the minimum objects dimensions: Those params are used
    *  for the following purposes:
    *
    *  1) the min pixels in blob is used to configure the movement model so it can
    *     disacrd the very small blobls mainlly result of noise on the scene
    *
    *  2) the min_lengh is the minimum heigh of a blob, this value is used to detect
    *     blobs very near from the 'finish line' so we can stop tracking them and generate
    *     the vehicle stats.
    */
   // Classifier::get_smallest_vehicle_dimension(&min_pixels_in_blob, &min_length);

   //TODO: uncoment this!.
   movement_mod->set_min_pixels_in_blob(min_pixels_in_blob);

   /** Detect the new blobs in the new frame*/
   unsigned int new_blobs_nb = movement_mod->find_blobs(_state.current_frame, _state.tmp_blobs, _cfg.advancedDetection);

   std::sort(_state.tmp_blobs.begin(), _state.tmp_blobs.end(), Blob::compare_size);

   feature_tracker->track_blobs(klt_only,
                                _state.current_frame,
                                _state.timestamp,
                                _state.vehicles,
                                _state.tmp_blobs,
                                _state.tmp_blobs.size());

   return (new_blobs_nb > 0);
}

/**
 *
 */
bool TrafficMonitorAlgorithm::proximity_tracking()
{
   unsigned int min_pixels_in_blob;
   float min_length=0;
   MovModel* movement_mod = MovModel::Instance();
   ProximityTracker* proximity_tracker = ProximityTracker::Instance();

   /** Get the minimum length and the minimum objects dimensions: Those params are used
    *  for the following purposes:
    *
    *  1) the min pixels in blob is used to configure the movement model so it can
    *     disacrd the very small blobls mainlly result of noise on the scene
    *
    *  2) the min_lengh is the minimum heigh of a blob, this value is used to detect
    *     blobs very near from the 'finish line' so we can stop tracking them and generate
    *     the vehicle stats.
    */
   // Classifier::get_smallest_vehicle_dimension(&min_pixels_in_blob, &min_length);
   movement_mod->set_min_pixels_in_blob(min_pixels_in_blob);

   /** Detect the new blobs in the new frame*/
   unsigned int new_blobs_nb = movement_mod->find_blobs(_state.current_frame, _state.tmp_blobs, _cfg.advancedDetection);

   proximity_tracker->track_blobs(_state.current_frame
                                  ,_state.timestamp
                                  ,_state.vehicles
                                  ,_state.tmp_blobs
                                  ,new_blobs_nb);

   return (new_blobs_nb > 0);
}

/**
 *
 */
void TrafficMonitorAlgorithm::processMouseMovement(int xCoor, int yCoor)
{
   // Find the nearest road point
   Tpoint2D tmp;
   std::vector<Tpoint2D> road_points = _cfg.get_road_points();
   float dist = INT_MAX;
   float tmp_dist=0;
   int pos=-1;

   tmp.y = xCoor;
   tmp.x = yCoor;

   /** Find the nearst point*/
   for(unsigned int i=0; i<road_points.size(); i++)
   {
      tmp_dist = DISTANCE_2D(road_points[i], tmp);
      if (tmp_dist<dist)
      {
         dist = tmp_dist;
         pos = i;
      }
   }

   /** If we find the nearst point then we move the current point to this location*/
   if (pos >= 0 && IN_IMAGE(tmp, _state.current_frame))
   {
      road_points[pos] = tmp;

      _cfg.set_road_points(road_points);

      if (_cfg.cameraAutoCalibration)
      {
         PinHoleCamera* camera = PinHoleCamera::Instance();
         camera->move_point(xCoor, yCoor);
         camera->autocalib();
      }
      else
      {
         // _state.road->move_tracking_zone_point(xCoor, yCoor);
         _state.road->set_corners(road_points);
      }
   }
}

/**
 *
 */
void TrafficMonitorAlgorithm::update_cfg(){

   // EvoTracking* evo_tracker = EvoTracking::Instance();

   if (_state.road) // && evo_tracker)
   {
      _state.road->update();

      /**
       * Scaled H to rectify road over an image. origin aligned with bottom-left corner.
       * Images have (0,0) in the top-left corner, so we need to adjust transformation.
       * The order of the rectified points is the below. Since this order is fixed we
       * should addapt our road-points (image pixels) points to match this order. This
       * consists basically in rotating the image by 90º.
       *
       *  Image         Rectified Road
       *
       * 0------1              3-----2
       * |      |  rotate 90º  |     |
       * 3------2              0-----1
       *
       */
      std::vector<Tpoint2D> road_points;
      _state.road->get_tracking_zone_points(road_points);
      _state.roadPoints[0] = road_points[3];
      _state.roadPoints[1] = road_points[0];
      _state.roadPoints[2] = road_points[1];
      _state.roadPoints[3] = road_points[2];

      // EvoTrackingConfig cfg = EvoTrackingConfig(M_TO_MM(road->get_width()),
      //                                           M_TO_MM(road->get_length()),
      //                                           _state.roadPoints);
      // evo_tracker->set_config(cfg);
   }

   Stats *stats = Stats::Instance();
   if (stats)
     stats->set_classification_status(_cfg.classify);
}

/**
 *
 */
void TrafficMonitorAlgorithm::classify_vehicles()
{
   Vehicle* vehicle;

   for (unsigned int i=0; i<_state.vehicles.size(); i++)
   {
      vehicle = (Vehicle*)_state.vehicles[i];

      if (vehicle)
      {
         if ((not vehicle->is_free()) && (not vehicle->ocluded))
         {
            if (_state.road->in_tracking_zone(vehicle->get_2d_center())
                &&
                vehicle_is_visible(vehicle, _state.current_frame))
            {
               m_classifier.classify_vehicle(vehicle, _state.current_frame);
            }
         }
      }
   }
}

/**
 *
 */
void TrafficMonitorAlgorithm::processFirstFrame(const colorspaces::Image& firstFrame, const timeval timeStamp)
{
   // This is the first frame, we should resize all the models using the received
   // frame image dimensions.
   MovModel* movement_mod = MovModel::Instance();
   _state.road->resize(firstFrame.height, firstFrame.width);
   movement_mod->resize(firstFrame.height, firstFrame.width);
   iteration(firstFrame,timeStamp);
}

/*****************************************************/
/*********** CLASSIFIER MAIN ITERATION ***************/
/*****************************************************/
bool TrafficMonitorAlgorithm::iteration(const colorspaces::Image& currentImg, const timeval timeStamp)
{
   static bool track_init=true;
   static int frames = 0;

   frames++;

   if (_cfg.play)
   {
      /********** UPDATE THE INPUT IMAGE *************/
      _state.timestamp = timeStamp;
      _state.prev_frame = _state.current_frame.clone();
      _state.current_frame = currentImg.clone();

      /******* BACKGROUND UPDATE *******/
      _state.background->Process(cv::Mat(_state.current_frame));

      /******* ROAD UPDATE
       * The Road estimation algorithm is based on the Background calcualtion.
       * The road is only updated when a new background is estimated.
       */
      colorspaces::Image bgImage = colorspaces::Image(cv::Mat(Background::Instance()->GetBackground()), _state.current_frame.format());
      _state.background_image = bgImage.clone();
      _state.road->update(bgImage);

      track_init=false;
   }

   /********** TRACKING *************/
   if (_cfg.play)
   {
      if (_cfg.trackingIsActive() || _cfg.classify)
      {
         if (_cfg.kltTrackingActive)
         {
            feature_tracking(_cfg.pixelTrackTrackingActive);
         }
         else if (_cfg.proximityTrackingActive)
         {
            proximity_tracking();
         }
      }
   }

   /********* CLASSIFICATION ********/
   if ((_cfg.play && _cfg.classify) || (_cfg.classify && _cfg.cameraAutoCalibration))
   {
      classify_vehicles();
   }

   return true;
}
}
