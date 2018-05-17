#ifndef _TRAFFICMONITOR_CONFIG_
#define _TRAFFICMONITOR_CONFIG_

#include "planar_geom.h"

namespace trafficmonitor{

/**
 *
 */
class TrafficMonitorAlgorithmConfig{
public:

  static const std::string FILE_VIEW;
  static const std::string GTK_VIEW;

  TrafficMonitorAlgorithmConfig();

  std::string dump() const;
  const std::string get_camera_filename() const {return tm_conf_path + "/" + camera_conf_file;}; //TODO: use os.sep instead of '/'
  const std::string get_glade_filename() const {return tm_conf_path + "/" + glade_file_name;};
  const std::vector<Tpoint2D>& get_road_points() const{return roadPoints;};
  unsigned int get_max_vehicles() const {return max_vehicles;};
  double get_road_length() const{return road_length;};
  double get_road_width() const{return road_width;};
  int get_detection_zone_percentage() const{return detectionZonePercentage;};

  void set_road_points(std::vector<Tpoint2D>& road_points) {roadPoints = road_points;};

  bool isValid();
  bool trackingIsActive() const {return (proximityTrackingActive || pixelTrackTrackingActive || kltTrackingActive);};
  void show();
  void save();
  void loadConfiguration();
  void loadDefaultConfiguration();

  // Gui control options
  bool showTrackingZone;
  bool showDetectionZone;
  bool showTrackingInfo;
  bool showCategories;
  bool showBoundingBox;
  bool showOclusion;
  bool showKltPoints;
  bool showBgMask;
  bool showOrigBgMask;
  bool showProjections;
  bool showTmpBlobs;
  bool showExtendedInfo;
  bool showBlobBg;
  bool showBlobFg;

  // Algorithm control options
  bool play;
  bool classify;
  bool useStaticRoad;
  bool switchDetectionZone;
  bool kltTrackingActive;
  bool proximityTrackingActive;
  bool pixelTrackTrackingActive;
  bool shadowDetectionActive;
  bool cameraAutoCalibration;
  bool advancedDetection;
  int  detectionZonePercentage;
  std::string gui;
  std::string bgalg;
  std::string tm_conf_path;
  std::string glade_file_name;

private:

  std::string camera_conf_file;
  unsigned int max_vehicles;
  int road_width;
  int road_length;
  std::vector<Tpoint2D> roadPoints;

};

}//namspace

#endif
