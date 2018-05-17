#include <sys/time.h>
#include <string.h>
#include <limits>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include "camera_model.h"
#include "planar_geom.h"
#include "blob.h"

#define DISTANCE(a,b) sqrt((a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y))
#define MIN_TRACKING_DISTANCE 20

using namespace std;

namespace trafficmonitor{

/**
 *
 */
Blob::Blob(){
   memset(points, 0, MAX_COUNT*sizeof(CvPoint2D32f));
   tracking_blobs_id.resize(50);
   contour.resize(MAX_CONTOUR_SIZE);
   m_idx = 0;
   m_upadte_interval = 3;
   m_locationHistory.resize(MIN_LOCATION_HISTORY);
   m_blobWitdh.resize(MIN_LOCATION_HISTORY);
   m_blobHeigth.resize(MIN_LOCATION_HISTORY);
   init();
}

/**
 *
 */
bool Blob::compare_size(const Blob* first, const Blob* second)
{
   CvRect first_rect = first->get_rect();
   CvRect second_rect = second->get_rect();
   return ((first_rect.width*first_rect.height) < (second_rect.width*second_rect.height));
}

/**
 *
 */
void Blob::init(){

   id = -1;
   free = true;
   is_tracked = false;
   in_opp_direction = false;
   matched = 0;
   num_points = 0;
   radius = 0;
   num_features = 0;
   m_life = 0;

   // direction = 0;
   direction.a = 0;
   direction.b = 0;

   prev_2d_center.x = 0;
   prev_2d_center.y = 0;

   current_2d_center.x = 0;
   current_2d_center.y = 0;

   center_of_mass.x = 0;
   center_of_mass.y = 0;

   first_3d_center.X = 0;
   first_3d_center.Y = 0;
   first_3d_center.Z = 0;

   current_3d_center.X = 0;
   current_3d_center.Y = 0;
   current_3d_center.Z = 0;

   prev_3d_center=current_3d_center;

   left_corner.x = INT_MAX;
   left_corner.y = INT_MAX;

   right_corner.x = 0;
   right_corner.y = 0;

   ocluded = false;
   first_time = true;

   tracking_blobs_id.clear();
   contour.clear();
   m_blobWitdh.clear();
   m_blobHeigth.clear();

   for(int i=0; i<m_locationHistory.size(); i++)
   {
      m_locationHistory[i] = Point(-1,-1);
   }

   obsIsUsed = false;
}

/**
 *
 */
void Blob::join(Blob& v2)
{
   num_points += v2.num_points;

   left_corner.x = min(left_corner.x, v2.left_corner.x);
   left_corner.y = min(left_corner.y, v2.left_corner.y);

   right_corner.x = max(right_corner.x, v2.right_corner.x);
   right_corner.y = max(right_corner.y, v2.right_corner.y);
}

/**
 *
 */
void Blob::start_tracking(timeval timestamp)
{
   HPoint2D hp_tmp;
   HPoint3D hppoint;
   PinHoleCamera* camera = PinHoleCamera::Instance();
   m_life = 0;

   m_blobWitdh.clear();
   m_blobHeigth.clear();
   for(int i=0; i<m_locationHistory.size(); i++)
   {
      m_locationHistory[i] = Point(-1,-1);
   }

   set_free(false);
   is_tracked = true;

   hp_tmp.y = current_2d_center.y;
   hp_tmp.x = current_2d_center.x;
   camera->reproject(hp_tmp, &hppoint, 0);

   first_2d_center = current_2d_center;
   first_3d_center = hppoint;
   current_3d_center = first_3d_center;
   ingress_timestamp = timestamp;
}

/**
 *
 */
void Blob::end_of_tracking(timeval timestamp)
{
   HPoint3D hppoint;
   HPoint2D hp_tmp;
   PinHoleCamera* camera = PinHoleCamera::Instance();

   is_tracked = false;
   egress_timestamp = timestamp;

   hp_tmp.x = current_2d_center.x;
   hp_tmp.y = current_2d_center.y;
   camera->reproject(hp_tmp, &hppoint, 0);

   current_3d_center = hppoint;
}

/**
 *
 */
void Blob::update_3d_center()
{
   PinHoleCamera* camera = PinHoleCamera::Instance();
   HPoint2D hp_tmp;
   HPoint3D hppoint;

   hp_tmp.x = current_2d_center.x;
   hp_tmp.y = current_2d_center.y;
   camera->reproject(hp_tmp, &hppoint, 0);
   current_3d_center = hppoint;
}

/**
 *
 */
bool Blob::GetDirection(Vec4f& direction) const
{
   return fit_line(m_locationHistory, direction, MIN_POINTS_FOR_LINE_FIT);
}

/**
 *
 */
bool Blob::IsBlobInProximityElipse(Blob* blob) const
{
   Tpoint2D ec = get_2d_center(); // Ellipse center
   CvRect rect = get_rect();
   Tpoint2D tmp_blob_center = blob->get_2d_center();
   float theta = M_PI;
   Vec4f direction;
   float distance = -1;

   //TODO: Comment
   if (GetDirection(direction))
   {
      float w = min(max(rect.height/2, 4), 15);
      float h = w/2;
      float theta = atanf(direction[1]/direction[0]);
      float u = cos(theta)*(tmp_blob_center.x - ec.x) + sin(theta)*(tmp_blob_center.y - ec.y);
      float v = cos(theta)*(tmp_blob_center.y - ec.y) - sin(theta)*(tmp_blob_center.x - ec.x);
      distance = (u/w)*(u/w) + (v/h)*(v/h);
      // printf("ellipse distance is %f \n",distance);
   }
   else
   {
      // (x - center_x)^2 + (y - center_y)^2 < radius^2
      float w = min(max(rect.height, 4), 20);
      float h = (w*w)/4; // (w/2) ^ 2
      float u = (tmp_blob_center.y - ec.y);
      float v = (tmp_blob_center.x - ec.x);
      distance = (u*u)/h + (v*v)/h;
      // printf("circle distance is %f radius=%f (%d:%d) -> (%d:%d)\n"
      //        ,distance
      //        ,w/2
      //        ,tmp_blob_center.x
      //        ,tmp_blob_center.y
      //        ,ec.x
      //        ,ec.y);
   }

   return (distance <= 1);
}

void Blob::UpdateDirection()
{
   if ((m_life % m_upadte_interval) == 0)
   {
      Point p;
      p.x = current_2d_center.x;
      p.y = current_2d_center.y;
      m_locationHistory[m_idx] = p;
      m_idx = (m_idx + 1) % MIN_LOCATION_HISTORY;
   }
}

/**
 *
 */
void Blob::track(CvRect& newRect)
{
   PinHoleCamera* camera = PinHoleCamera::Instance();
   HPoint2D hp_tmp;
   HPoint3D hppoint;
   Tvector vtmp_1, vtmp_2;
   Tpoint2D orig_center;

   struct timeval time;
   gettimeofday(&time, NULL);
   unsigned int now  = (time.tv_sec*1000+time.tv_usec/1000);

   inc_life();

   UpdateDirection();

   set_prev_2d_center(get_2d_center());
   set_rect(newRect);

   get_vector(&direction, prev_2d_center, current_2d_center);

   hp_tmp.x = current_2d_center.x;
   hp_tmp.y = current_2d_center.y;
   camera->reproject(hp_tmp, &hppoint, 0);
   prev_3d_center = current_3d_center;
   current_3d_center = hppoint;

   if (m_life > MIN_LIFE)
   {
      m_blobWitdh.push_back(Point(m_life, newRect.width));
      m_blobHeigth.push_back(Point(m_life, newRect.height));
   }
}

/**
 *
 */
bool Blob::get_estimated_size(int& w, int& h)
{
   w = -1;
   h = -1;

   if (m_life < MIN_POINTS_FOR_LINE_FIT)
   {
      return false;
   }

   Vec4f witdh_curve;
   Vec4f heigth_curve;
   if (fit_line(m_blobWitdh, witdh_curve, MIN_POINTS_FOR_LINE_FIT)
       && fit_line(m_blobHeigth, heigth_curve, MIN_POINTS_FOR_LINE_FIT))
   {
      for(int i=0; i<m_blobHeigth.size(); i++)
      {
         printf("%d: %d, ",m_blobHeigth[i].x, m_blobHeigth[i].y);
      }

      // Y = A.X + B =>  witdh = A*life + B
      float A = witdh_curve[1]/witdh_curve[0];
      float B = witdh_curve[3] - (A*witdh_curve[2]);
      w = (int)((A*(m_life))+B);

      float A2 = heigth_curve[1]/heigth_curve[0];
      float B2 = heigth_curve[3] - (A2*heigth_curve[2]);
      h = (int)((A2*(m_life))+B2);

      return true;
   }
   else
   {
      false;
   }
}

/**
 *
 */
bool Blob::get_metric_size(double& w, double& h)
{
   HPoint3D lc_3d, rc_3d;
   HPoint2D lc,rc;
   PinHoleCamera* camera = PinHoleCamera::Instance();

   lc.x = left_corner.x;
   lc.y = left_corner.y;
   camera->reproject(lc, &lc_3d, 1.6);

   rc.x = right_corner.x;
   rc.y = right_corner.y;
   camera->reproject(rc, &rc_3d, 0);

   w = rc_3d.X - lc_3d.X;
   h = lc_3d.Y - rc_3d.Y;

   // printf("** get size: lc(%.2f , %.2f) 3dlc(%.2f , %.2f) rc(%.2f , %.2f) 3drc(%.2f , %.2f) w=%.2f h=%.2f \n"
   //        ,lc.x
   //        ,lc.y
   //        ,lc_3d.X
   //        ,lc_3d.Y
   //        ,rc.x
   //        ,rc.y
   //        ,rc_3d.X
   //        ,rc_3d.Y
   //        ,w
   //        ,h);
}

/**
 *
 */
void Blob::track(Blob& blob)
{
   PinHoleCamera* camera = PinHoleCamera::Instance();
   HPoint2D hp_tmp;
   HPoint3D hppoint;
   Tvector vtmp_1, vtmp_2;
   Tpoint2D orig_center;

   struct timeval time;
   gettimeofday(&time, NULL);
   unsigned int now  = (time.tv_sec*1000+time.tv_usec/1000);
   // unsigned int time_delta = now-egress_timestamp;
   // egress_timestamp = now;

   inc_life();

   // orig_center = get_prev_2d_center();

   // //
   // get_vector(&vtmp_1, orig_center, get_2d_center());

   UpdateDirection();

   set_prev_2d_center(get_2d_center());
   set_2d_center(blob.get_2d_center());
   set_left_corner(blob.get_left_corner());
   set_right_corner(blob.get_right_corner());
   num_points = blob.get_num_points();
   center_of_mass = blob.get_center_of_mass();

   if (m_life > MIN_LIFE)
   {
      m_blobWitdh.push_back(Point(m_life, get_rect().width));
      m_blobHeigth.push_back(Point(m_life, get_rect().height));
   }

   get_vector(&direction, prev_2d_center, current_2d_center);

   //
   // get_vector(&vtmp_2, orig_center, get_2d_center());
   // direction = angle_vectors(vtmp_1, vtmp_2);

   contour = blob.contour;

   hp_tmp.x = current_2d_center.x;
   hp_tmp.y = current_2d_center.y;
   camera->reproject(hp_tmp, &hppoint, 0);
   prev_3d_center = current_3d_center;
   current_3d_center = hppoint;
}

Tpoint2D Blob::predict_next_position()
{
   Tpoint2D nxt_pos = current_2d_center;

   return nxt_pos;
}

/**
 *
 */
void Blob::update_blob_box_center(){

   current_2d_center.x = (int)(right_corner.x + left_corner.x)/2;
   current_2d_center.y = (int)(right_corner.y + left_corner.y)/2;
   calculate_center_of_mass();
}

/**
 *
 */
void Blob::print(){

   printf("Blob(%d) free=%s tracked=%s life=%d matched=%s num_points=%d r=%.2f\n",
          id,
          free?"true":"false",
          is_tracked?"true":"false",
          m_life,
          matched?"true":"false",
          num_points,
          radius);

   printf("lc(%d:%d) cc(%d:%d) f3dc(%.2f:%.2f:%.2f) c3dc(%.2f:%.2f:%.2f)\n",
          prev_2d_center.x ,
          prev_2d_center.y ,
          current_2d_center.x ,
          current_2d_center.y ,
          first_3d_center.X ,
          first_3d_center.Y ,
          first_3d_center.Z ,
          current_3d_center.X ,
          current_3d_center.Y ,
          current_3d_center.Z);

   printf("left-c(%d:%d) right-c(%d:%d)\n",
          left_corner.x,
          left_corner.y,
          right_corner.x,
          right_corner.y);
}

float Blob::get_covered_distance()
{
    return DISTANCE(get_first_3d_center(), get_current_3d_center());
}

bool Blob::is_valid_tracking()
{
  return (m_life > MIN_LOCATION_HISTORY) && (get_covered_distance() >= MIN_TRACKING_DISTANCE);
}

} // trafficomonitor Namespace
