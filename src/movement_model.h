#ifndef _MOVEMENT_MODEL_
#define _MOVEMENT_MODEL_

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <list>
#include <vector>

#include <colorspacesmm.h>
#include "blob.h"
#include "singleton.h"

using namespace cv;
using std::vector;
using std::list;

namespace trafficmonitor{

/*
 * Default minimu "blob" size in pixels, movements under this value will be ignored.
 */
#define MIN_PIXELS_IN_BLOB_DEFAULT 40

/*
 * The maximum with of a valid blob in meters
 */
#define TM_MAX_BLOB_WIDTH 4

/** Those constants determine if a pixel has moved. We choose those values
 *  to avoid colision with blobs classes that are stored on the movement
 *  matrix when exploring moving regions
 */
#define STATIC 0
#define MOVED 255

class MovModel: public CSingleton<MovModel>{
   
public:

   /**
    *
    */
   void resize(unsigned int height_val, unsigned int width_val);

   /**
    *
    */
   int find_blobs(colorspaces::Image& new_frame, std::vector<Blob*>& new_blobs, bool advanced_detection);

   /**
    *
    */
   int FindBlobs(const cv::Mat &binary, std::vector<Blob*>& new_blobs);
   int cv_find_blobs(colorspaces::Image& new_frame, std::vector<Blob*>& new_blobs);

   /**
    *
    */
   int get_min_pixels_in_blob(){return min_pixels_in_blob;};

   /**
    *
    */
   void set_min_pixels_in_blob(int new_value){min_pixels_in_blob=new_value;};

   /**
    *
    */
   bool movement_on(Tpoint2D p);
   bool movement_on(int row, int col);

   /**
    *
    */
   Mat& get_blob_fg() {return m_refined_blob_fg;}
   Mat& get_blob_bg() {return m_refined_blob_bg;}
   
   /** This array has an entr for each pixel in the image. It represents that pixel
    *  that haven been moved in the last  iteration. This is used by the classifier
    *  module to determine the moved pixels
    */
   Mat_<uchar> movement_matrix;

protected:

   friend class CSingleton<MovModel>;

private:

   /**
    *
    */
   MovModel();

   /**
    *
    */
   inline bool movement_detected_on(const Tpoint2D& p);
   inline bool movement_detected_on(int i, int j);
   inline bool is_internal_point(Tpoint2D point, colorspaces::Image& new_frame);
   int opt_growing(colorspaces::Image& new_frame, Rect roi, std::vector<Blob*>& new_blobs);

   /**
    * 
    */
   void blobs_fusion(std::vector<Blob*>& blobs, int num_blobs);

   /**
    *
    */
   void build_movement_matrix(colorspaces::Image& new_frame, int* max_idx);

   /**
    *
    */
   void optContour(Blob* blob);

   int recursive_growing_region(std::vector<Blob*>& new_blobs, int idx);
   int iterative_region_growing(std::vector<Blob*>& new_blobs, int idx);
   
   void get_neighbors(const Tpoint2D& p, list<Tpoint2D>& neighbors);
   void get_neighbors(const Tpoint2D& p, list<Tpoint2D>& neighbors, Blob* curr_blob);
   bool is_valid_blob(Blob* blob);
   bool pre_process_blobs(colorspaces::Image& new_frame, std::vector<Blob*>& new_blobs);
   bool refine_blobs(Mat image, Mat blob_area, Mat bg_img, Rect roi, Mat mask);
   bool is_refined_blob(Blob* blob);
      
   /**
    *
    */
   int explore(int i, int j, Blob* blobs);
   
   /**
    *
    */
   int min_pixels_in_blob;
   vector<Tpoint2D> moved_pixels;

   bool initialized;
   unsigned int width;
   unsigned int height;

   /**
    * This list contains all the blobs that have been refined in the current iteration
    * 
    */
   vector<Rect> m_refined_zones;

   /**
    * This is the a temporal buffers used to process a blob with shadows. All the morphological operations 
    * are applied on them buffer this way we avoid affecting the original background. The first one stores
    * the blob foreground meanwhile the second one hold the blob background.
    */
   Mat m_refined_blob_fg;
   Mat m_refined_blob_bg;
};
}

#endif

