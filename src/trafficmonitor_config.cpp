#include <sstream>
#include <string>
#include <iostream>
#include <trafficmonitor_config.h>
#include <map>
#include <fstream>
#include <unordered_map>

using namespace std;

namespace trafficmonitor {

const string prefix                            = "TrafficMonitor.";
const string MAX_VEHICLES                      = prefix+"MaxVehicles";
const string CONTROL_PLAY                      = prefix+"Control.play";
const string CONTROL_CLASIFY                   = prefix+"Control.classify";
const string CONTROL_USE_STATIC_ROAD           = prefix+"Control.useStaticRoad";
const string CONTROL_SWITCH_DETECTION_ZONE     = prefix+"Control.switchDetectionZone";
const string CONTROL_DETECTION_ZONE_PERCENTAGE = prefix+"Control.detectionZonePercentage";
const string CONTROL_KLT_TRACKING              = prefix+"Control.kltTrackingActive";
const string CONTROL_PROXIMITY_TRACKING        = prefix+"Control.proximityTrackingActive";
const string CONTROL_PIXELTRACK_TRACKING       = prefix+"Control.pixelTrackTrackingActive";
const string CONTROL_ADVANCED_DETECTION        = prefix+"Control.advancedDetection";
const string CONTROL_SHADOW_DETECTION          = prefix+"Control.shadowDetectionActive";
const string CONTROL_CAMERA_AUTO_CALIB         = prefix+"Control.cameraAutoCalibration";
const string CONTROL_GUI                       = prefix+"Control.gui.output";
const string CONTROL_GUI_SHOW_TRACKING_ZONE    = prefix+"Control.gui.showTrackingZone";
const string CONTROL_GUI_SHOW_DETECTION_ZONE   = prefix+"Control.gui.showDetectionZone";
const string CONTROL_GUI_SHOW_TRACKING_INFO    = prefix+"Control.gui.showTrackingInfo";
const string CONTROL_GUI_SHOW_CATEGORIES       = prefix+"Control.gui.showCategories";
const string CONTROL_GUI_SHOW_BBOX             = prefix+"Control.gui.showBoundingBox";
const string CONTROL_GUI_SHOW_OCLUSION         = prefix+"Control.gui.showOclusion";
const string CONTROL_GUI_SHOW_KLT_POINTS       = prefix+"Control.gui.showKltPoints";
const string CONTROL_GUI_SHOW_BG_MASK          = prefix+"Control.gui.showBgMask";
const string CONTROL_GUI_SHOW_ORIG_BG_MASK     = prefix+"Control.gui.showOrigBgMask";
const string CONTROL_GUI_SHOW_PROJECTIONS      = prefix+"Control.gui.showProjections";
const string CONTROL_GUI_SHOW_TMP_BLOBS        = prefix+"Control.gui.showTmpBlobs";
const string CONTROL_GUI_SHOW_EXTENDED_INFO    = prefix+"Control.gui.showExtendedInfo";
const string CONTROL_BG_GUI_SHOW_BLOB_BG       = prefix+"Control.bg.gui.showBlobBg";
const string CONTROL_BG_GUI_SHOW_BLOB_FG       = prefix+"Control.bg.gui.showBlobFg";

const string CONTROL_BGALB                     = prefix+"Control.bgalg";
const string CONTROL_CAMERA_CALIB_FILE         = prefix+"Control.CameraCalibFile";
const string ROAD_LENGTH                       = prefix+"Road.Length";
const string ROAD_WIDTH                        = prefix+"Road.Width";
const string ROAD_A_X                          = prefix+"Road.A.x";
const string ROAD_A_Y                          = prefix+"Road.A.y";
const string ROAD_B_X                          = prefix+"Road.B.x";
const string ROAD_B_Y                          = prefix+"Road.B.y";
const string ROAD_C_X                          = prefix+"Road.C.x";
const string ROAD_C_Y                          = prefix+"Road.C.y";
const string ROAD_D_X                          = prefix+"Road.D.x";
const string ROAD_D_Y                          = prefix+"Road.D.y";
const string TM_CONF_PATH                      = prefix+"tmConfPath";
const string GLADE_FILE_NAME                   = prefix+"gladeFileName";

  /**
   *
   */
  TrafficMonitorAlgorithmConfig::TrafficMonitorAlgorithmConfig()
  {
    loadConfiguration();
  }

  void TrafficMonitorAlgorithmConfig::loadDefaultConfiguration()
  {
    Tpoint2D p;

    max_vehicles = 10;
    road_length = 40825;
    road_width = 5602;
    detectionZonePercentage = 36;

    play                     = 1;
    classify                 = 1;
    useStaticRoad            = 1;
    switchDetectionZone      = 0;
    kltTrackingActive        = 1;
    proximityTrackingActive  = 0;
    pixelTrackTrackingActive = 0;
    advancedDetection        = 0;
    shadowDetectionActive    = 0;
    cameraAutoCalibration    = 0;
    bgalg                    = "CvMog";
    camera_conf_file         = "camera.cfg";
    tm_conf_path             = ".";
    glade_file_name          = "trafficmonitor.glade";

    gui               = "gtk_view";
    showTrackingZone  = 1;
    showDetectionZone = 1;
    showTrackingInfo  = 1;
    showCategories    = 1;
    showBoundingBox   = 1;
    showOclusion      = 0;
    showKltPoints     = 0;
    showBgMask        = 0;
    showOrigBgMask    = 0;
    showProjections   = 1;
    showTmpBlobs      = 0;
    showExtendedInfo  = 0;

    showBlobFg  = 0;
    showBlobBg  = 0;

    p.x = 1;
    p.y = 237;
    roadPoints.push_back(p);
    p.x = 127;
    p.y = 61;
    roadPoints.push_back(p);
    p.x = 185;
    p.y = 61;
    roadPoints.push_back(p);
    p.x = 310;
    p.y = 238;
    roadPoints.push_back(p);
  }

  void TrafficMonitorAlgorithmConfig::loadConfiguration()
  {
    stringstream ss;
    Tpoint2D p;

    unordered_map <string, string> cfg;
    ifstream cfg_file("trafficmonitor.cfg");

    if (!cfg_file.good())
    {
      std::cout << "Cannot open configuration file 'trafficmonitor.cfg' " << std::endl;
      exit(1);
    }

    string line;
    while( getline(cfg_file, line) )
    {
      istringstream is_line(line);
      string key;
      if( getline(is_line, key, '=') )
      {
        string value;
        if( getline(is_line, value) ){
          cfg[key] = value;
        }
      }
    }

    max_vehicles = stoi(cfg[MAX_VEHICLES]);
    road_length = stoi(cfg[ROAD_LENGTH]);
    road_width = stoi(cfg[ROAD_WIDTH]);
    detectionZonePercentage = stoi(cfg[CONTROL_DETECTION_ZONE_PERCENTAGE]);

    play                     = stoi(cfg[CONTROL_PLAY]);
    classify                 = stoi(cfg[CONTROL_CLASIFY]);
    useStaticRoad            = stoi(cfg[CONTROL_USE_STATIC_ROAD]);
    switchDetectionZone      = stoi(cfg[CONTROL_SWITCH_DETECTION_ZONE]);
    kltTrackingActive        = stoi(cfg[CONTROL_KLT_TRACKING]);
    proximityTrackingActive  = stoi(cfg[CONTROL_PROXIMITY_TRACKING]);
    pixelTrackTrackingActive = stoi(cfg[CONTROL_PIXELTRACK_TRACKING]);
    advancedDetection        = stoi(cfg[CONTROL_ADVANCED_DETECTION]);
    shadowDetectionActive    = stoi(cfg[CONTROL_SHADOW_DETECTION]);
    cameraAutoCalibration    = stoi(cfg[CONTROL_CAMERA_AUTO_CALIB]);
    bgalg                    = cfg[CONTROL_BGALB];
    camera_conf_file         = cfg[CONTROL_CAMERA_CALIB_FILE];
    tm_conf_path             = cfg[TM_CONF_PATH];
    glade_file_name          = cfg[GLADE_FILE_NAME];

    gui               = cfg[CONTROL_GUI];
    showTrackingZone  = stoi(cfg[CONTROL_GUI_SHOW_TRACKING_ZONE]);
    showDetectionZone = stoi(cfg[CONTROL_GUI_SHOW_DETECTION_ZONE]);
    showTrackingInfo  = stoi(cfg[CONTROL_GUI_SHOW_TRACKING_INFO]);
    showCategories    = stoi(cfg[CONTROL_GUI_SHOW_CATEGORIES]);
    showBoundingBox   = stoi(cfg[CONTROL_GUI_SHOW_BBOX]);
    showOclusion      = stoi(cfg[CONTROL_GUI_SHOW_OCLUSION]);
    showKltPoints     = stoi(cfg[CONTROL_GUI_SHOW_KLT_POINTS]);
    showBgMask        = stoi(cfg[CONTROL_GUI_SHOW_BG_MASK]);
    showProjections   = stoi(cfg[CONTROL_GUI_SHOW_PROJECTIONS]);
    showTmpBlobs      = stoi(cfg[CONTROL_GUI_SHOW_TMP_BLOBS]);
    showExtendedInfo  = stoi(cfg[CONTROL_GUI_SHOW_EXTENDED_INFO]);

    p.x = stoi(cfg[ROAD_A_X]);
    p.y = stoi(cfg[ROAD_A_Y]);
    roadPoints.push_back(p);
    p.x = stoi(cfg[ROAD_B_X]);
    p.y = stoi(cfg[ROAD_B_Y]);
    roadPoints.push_back(p);
    p.x = stoi(cfg[ROAD_C_X]);
    p.y = stoi(cfg[ROAD_C_Y]);
    roadPoints.push_back(p);
    p.x = stoi(cfg[ROAD_D_X]);
    p.y = stoi(cfg[ROAD_D_Y]);
    roadPoints.push_back(p);
  }

/**
 *
 */
string TrafficMonitorAlgorithmConfig::dump() const
{
   stringstream ss;
   ss.clear();

   //TODO, set play to its value, right now if play set to zero the prog crash
   // during the initialization

   ss << MAX_VEHICLES                        << "=" << max_vehicles               << endl
      << ROAD_LENGTH                         << "=" << road_length                << endl
      << ROAD_WIDTH                          << "=" << road_width                 << endl
      << ROAD_A_X                            << "=" << roadPoints[3].x            << endl
      << ROAD_A_Y                            << "=" << roadPoints[3].y            << endl
      << ROAD_B_X                            << "=" << roadPoints[0].x            << endl
      << ROAD_B_Y                            << "=" << roadPoints[0].y            << endl
      << ROAD_C_X                            << "=" << roadPoints[1].x            << endl
      << ROAD_C_Y                            << "=" << roadPoints[1].y            << endl
      << ROAD_D_X                            << "=" << roadPoints[2].x            << endl
      << ROAD_D_Y                            << "=" << roadPoints[2].y            << endl
      << CONTROL_PLAY                        << "=" << 1                          << endl
      << CONTROL_CLASIFY                     << "=" << classify                   << endl
      << CONTROL_USE_STATIC_ROAD             << "=" << useStaticRoad              << endl
      << CONTROL_SWITCH_DETECTION_ZONE       << "=" << switchDetectionZone        << endl
      << CONTROL_DETECTION_ZONE_PERCENTAGE   << "=" << detectionZonePercentage    << endl
      << CONTROL_KLT_TRACKING                << "=" << kltTrackingActive          << endl
      << CONTROL_PROXIMITY_TRACKING          << "=" << proximityTrackingActive     << endl
      << CONTROL_PIXELTRACK_TRACKING         << "=" << pixelTrackTrackingActive   << endl
      << CONTROL_ADVANCED_DETECTION          << "=" << advancedDetection          << endl
      << CONTROL_SHADOW_DETECTION            << "=" << shadowDetectionActive      << endl
      << CONTROL_CAMERA_AUTO_CALIB           << "=" << cameraAutoCalibration      << endl
      << CONTROL_GUI                         << "=" << gui                        << endl
      << CONTROL_GUI_SHOW_TRACKING_ZONE      << "=" << showTrackingZone           << endl
      << CONTROL_GUI_SHOW_DETECTION_ZONE     << "=" << showDetectionZone          << endl
      << CONTROL_GUI_SHOW_TRACKING_INFO      << "=" << showTrackingInfo           << endl
      << CONTROL_GUI_SHOW_CATEGORIES         << "=" << showCategories             << endl
      << CONTROL_GUI_SHOW_BBOX               << "=" << showBoundingBox            << endl
      << CONTROL_GUI_SHOW_OCLUSION           << "=" << showOclusion               << endl
      << CONTROL_GUI_SHOW_KLT_POINTS         << "=" << showKltPoints              << endl
      << CONTROL_GUI_SHOW_BG_MASK            << "=" << showBgMask                 << endl
      << CONTROL_GUI_SHOW_ORIG_BG_MASK       << "=" << showOrigBgMask             << endl
      << CONTROL_GUI_SHOW_PROJECTIONS        << "=" << showProjections            << endl
      << CONTROL_GUI_SHOW_TMP_BLOBS          << "=" << showTmpBlobs               << endl
      << CONTROL_GUI_SHOW_EXTENDED_INFO      << "=" << showExtendedInfo           << endl
      << CONTROL_BGALB                       << "=" << bgalg                      << endl
      << CONTROL_CAMERA_CALIB_FILE           << "=" << camera_conf_file           << endl
      << TM_CONF_PATH                        << "=" << tm_conf_path               << endl
      << CONTROL_BG_GUI_SHOW_BLOB_BG         << "=" << showBlobBg                 << endl
      << CONTROL_BG_GUI_SHOW_BLOB_FG         << "=" << showBlobFg                 << endl
      << GLADE_FILE_NAME                     << "=" << glade_file_name            << endl;

   return ss.str();
}

/**
 *
 */
bool TrafficMonitorAlgorithmConfig::isValid()
{
   bool conf_is_ok = true;
   std::string glade_path = get_glade_filename();
   std::string camera_path = get_camera_filename();

   ifstream glade_file(glade_path);
   ifstream camera_file(camera_path);

   if (!glade_file.good())
   {
      cout << endl << "** Error: Can't read the glade file '" << glade_path <<"'" << endl;
      conf_is_ok = false;
   }
   else if (!camera_file.good())
   {
      cout << endl << "** Error: Can't read the camera file '" << camera_path <<"'" << endl;
      conf_is_ok = false;
   }

   return conf_is_ok;
}

/**
 *
 */
void TrafficMonitorAlgorithmConfig::show()
{
   std::cout << " Loading Config: " << std::endl;
   std::cout << " ------------------------------ " << std::endl;
   std::cout << dump();
   std::cout << " ------------------------------ " << std::endl;
}

/**
 *
 */
void TrafficMonitorAlgorithmConfig::save()
{
  std::string cfg_file_name = "trafficmonitor.cfg";
  ofstream out(cfg_file_name.c_str(), ios::out | ios::binary);
  if (out)
  {
    out << dump();
    out.close();
  }
  else
  {
    cout << "Cannot open output file" << endl;
  }
}

}
