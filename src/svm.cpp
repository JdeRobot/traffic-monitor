#include <opencv2/objdetect/objdetect.hpp>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>

#include <colorspacesmm.h>
#include "svm.h"
#include "planar_geom.h"
#include "rectify.h"

#define WITDH  96
#define HEIGHT 128

#define IN_IMAGE(p, image) (p.x > 0 && p.x<image.cols && p.y > 0 && p.y<image.rows)

namespace trafficmonitor
{

bool TrafficMonitorSVM::init(std::string path)
{
  if (access( path.c_str(), F_OK ) != -1 )
  {
    return true;
  }

  return false;
}

tvehicle_category TrafficMonitorSVM::classify_vehicle(Vehicle* vehicle, colorspaces::Image& inputImage)
{
  Tpoint2D e,f,g,h;
  Tpoint2D inputPoints[4];
  Tpoint2D rectifiedPoints[4];
  const Tpolygon* polygon = vehicle->get_projection();
  return TRUCK;

  e = polygon->segments[4].orig;
  f = polygon->segments[4].end;
  g = polygon->segments[5].end;
  h = polygon->segments[6].end;

  inputPoints[0] = e;
  inputPoints[1] = f;
  inputPoints[2] = g;
  inputPoints[3] = h;

  if (!IN_IMAGE(e, inputImage)
      || !IN_IMAGE(f, inputImage)
      || !IN_IMAGE(g, inputImage)
      || !IN_IMAGE(h, inputImage))
  {
    return INVALID_VEHICLE_CLASS;
  }

  rectifiedPoints[0] = Tpoint2D(0,0);
  rectifiedPoints[1] = Tpoint2D(0,WITDH);
  rectifiedPoints[2] = Tpoint2D(HEIGHT,WITDH);
  rectifiedPoints[3] = Tpoint2D(HEIGHT,0);

  Rectifier rectifier;
  colorspaces::Image output_image (cv::Mat::zeros(HEIGHT,WITDH, CV_8UC3), colorspaces::ImageRGB8::FORMAT_RGB8);
  rectifier.rectify(inputPoints, rectifiedPoints);
  rectifier.rectify(inputImage, output_image);

  HOGDescriptor d (Size(96,128)
                   ,Size(16,16)
                   ,Size(8,8)
                   ,Size(8,8)
                   ,16
                   ,0
                   ,-1
                   ,0
                   ,0.2
                   ,1
                   ,64);

  d.winSize = Size(WITDH, HEIGHT);
  // d.block_size only 16x16 is supported
  // d.block_stride;
  // d.cell_size only Size(8x8) is supported
  // d.nbins  only 9 bins are supported
  // d.win_sigma;
  // d.threshold_L2hys;
  // d.gamma_correction;
  // d.nlevels;

  vector<float> descriptorsValues;
  vector<Point> locations;
  d.compute( output_image, descriptorsValues);
  m_SVM = Algorithm::load<ml::SVM>("./svm.yml");
  float result;
  if (m_SVM)
    result = m_SVM->predict(cv::Mat(descriptorsValues));

  switch ((int)result)
  {
  case 0:
    printf("BUS\n");
    return BUS;
  case 1:
    printf("TRUCK\n");
    return TRUCK;
  case 2:
    printf("TANK TRUCK\n");
    return TRUCK;
  }

  return INVALID_VEHICLE_CLASS;
}

}
