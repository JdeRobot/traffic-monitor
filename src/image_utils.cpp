
#include "image_utils.h"
#include "background_model.h"

using namespace cv;

namespace trafficmonitor
{

void ImageUtils::MyCanny(Mat&src, Mat& dst)
{
   Mat src_gray;
   Mat detected_edges;
   int edgeThresh = 1;
   int lowThreshold = 50;
   int const max_lowThreshold = 100;
   int ratio = 3;
   int kernel_size = 3;

   /// Convert the image to grayscale
   cvtColor( src, src_gray, CV_BGR2GRAY);

   /// Reduce noise with a kernel 3x3
   blur( src_gray, detected_edges, Size(3,3) );

   /// Canny detector
   Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

   /// Using Canny's output as a mask, we display our result
   dst = Scalar::all(0);
   src.copyTo( dst, detected_edges);

   Background* bg = Background::Instance();

   for( size_t i = 0; i < dst.rows; i++ )
   {
      for( size_t j = 0; j < dst.cols; j++ )
      {
         if(!bg->isFg(i,j))
         {
            dst.at<cv::Vec3b>(i,j)[0] = 0;
            dst.at<cv::Vec3b>(i,j)[1] = 0;
            dst.at<cv::Vec3b>(i,j)[2] = 0;
         }
      }
   }
}

} // namespace
