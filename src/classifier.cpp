#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <algorithm>

#include <colorspacesmm.h>
#include "classifier.h"
#include "camera_model.h"
#include "road_detection.h"
#include "movement_model.h"

using namespace std;

namespace trafficmonitor{

#define DEBUG 0
#define CLASS_DEBUG 0

#define LOG(args...){if (DEBUG) printf(args);}
#define CATEGORY_LOG(args...){if (CLASS_DEBUG) printf(args);}
#define DEG2RAD(angle) ((float)(angle*PI)/180)
#define PI 3.1415926535

/****************************************************************/
/****************************************************************/
/****************************************************************/
#define MAX_LOOKUP_ITER 20
#define MIN_SEP_DISTANCE 1

Tpoint3D Classifier::sun_pos;
float Classifier::sun_radius=100;

/**
 *
 */
Classifier::Classifier()
{
   if (m_svm_classifier.init("./svm.yml"))
   {
      cout << "SVM model has been loaded correctly ..." << endl;
   }
   else
   {
      cout << "** Error while loading SVM model ..." << endl;
   }
}

/**
 *
 */
void Classifier::update_sun_pos(float theta_angle, float psi_angle)
{
   sun_pos.x = sun_radius*sin(DEG2RAD(theta_angle))*cos(DEG2RAD(psi_angle));
   sun_pos.y = sun_radius*sin(DEG2RAD(theta_angle))*sin(DEG2RAD(psi_angle));
   sun_pos.z = sun_radius*cos(DEG2RAD(theta_angle));
}

/**
 *
 */
void Classifier::set_orig_end(Tsegment* s, HPoint2D a, HPoint2D b){
   
   s->orig.x = a.x;
   s->orig.y = a.y;
   s->end.x = b.x;
   s->end.y = b.y;
   
   /** Update the segment line params*/
   calculate_line(s);
}

/**
 *
 */
int Classifier::shadows_are_present()
{
   return (sun_pos.x != 0 && sun_pos.y != 0);
}

/**
 *
 */
void Classifier::get_shadow(HPoint3D A, HPoint3D* shadow){

   if (shadows_are_present())
   {
      shadow->X = (A.X+(A.Z*sun_pos.x)/sun_pos.z);
      shadow->Y = (A.Y+(A.Z*sun_pos.y)/sun_pos.z);
      shadow->Z = 0;
      shadow->H = 1;
   }
   else
   {
      /** There's no shadow (the sun light is vertical) in this case
       *  we return the same point! TODO: this may lead to some problem!
       */
      shadow->X = A.X;
      shadow->Y = A.Y;
      shadow->Z = A.Z;
      shadow->H = 1;
   }

   LOG(" shadow of (%.1f,%.1f,%.1f) is  (%.1f,%.1f,%.1f)\n",A.X,A.Y,A.Z,shadow->X,shadow->Y,shadow->Z);
}

/** Received the model and its center and calculate its projection polygon and the its shadow proejction also.
 *  The folloiwng polygon is used for the projection:
 *
 *    A ______ B
 *     |\     |\
 *     | \    | \
 *     |  \   |  \
 *    D|___\__| C \
 *      \   \  \   \
 *       \  E\__\___\F
 *        \   |  \   |
 *         \  |   \  |
 *          \ |    \ |
 *          G\|_____\| H
 *
 */
void Classifier::project_model_and_shadow(tvehicle_category category, 
                                          HPoint3D center, 
                                          Tpolygon* polygon){

   int i=0;
   int center_x=0;
   int center_y=0;
   float x_offset, y_offset, z_offset;
   HPoint3D A,B,C,D,E,F,G,H;
   HPoint2D a,b,c,d,e,f,g,h;
   HPoint2D center_proj;
   PinHoleCamera* camera = PinHoleCamera::Instance();

   /** shadows points*/
   HPoint3D As,Bs,Es,Fs;
   HPoint2D as,bs,es,fs;

   /** Object segments*/
   Tsegment s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12;

   /** shadow segments*/
   Tsegment sh1,sh2,sh3,sh4,sh5,sh6,sh7,sh8,sh9,sh10,sh11,sh12;

   LOG("model_3d_center (%.2f::%.2f::%.2f)\n", center.X, center.Y,center.Z);

   x_offset = VehicleModel::get_category_dimension_y(category)/2;
   y_offset = VehicleModel::get_category_dimension_z(category)/2;
   z_offset = VehicleModel::get_category_dimension_x(category)/2;

   A.X = center.X-x_offset;
   A.Y = center.Y+y_offset;
   A.Z = center.Z+z_offset;
   A.H = 1;

   D.X = center.X-x_offset;
   D.Y = center.Y+y_offset;
   D.Z = center.Z-z_offset;
   D.H = 1;

   B.X = center.X+x_offset;
   B.Y = center.Y+y_offset;
   B.Z = center.Z+z_offset;
   B.H = 1;
  
   C.X = center.X+x_offset;
   C.Y = center.Y+y_offset;
   C.Z = center.Z-z_offset;
   C.H = 1;
  
   E.X = center.X-x_offset;
   E.Y = center.Y-y_offset;
   E.Z = center.Z+z_offset;
   E.H = 1;
  
   F.X = center.X+x_offset;
   F.Y = center.Y-y_offset;
   F.Z = center.Z+z_offset;
   F.H = 1;
  
   G.X = center.X+x_offset;
   G.Y = center.Y-y_offset;
   G.Z = center.Z-z_offset;
   G.H = 1;
  
   H.X = center.X-x_offset;
   H.Y = center.Y-y_offset;
   H.Z = center.Z-z_offset;
   H.H = 1;

   camera->myproject(A, &a);
   camera->myproject(B, &b);
   camera->myproject(C, &c);
   camera->myproject(D, &d);
   camera->myproject(E, &e);
   camera->myproject(F, &f);
   camera->myproject(G, &g);
   camera->myproject(H, &h);
   camera->myproject(center, &center_proj);

   camera->optic2grafic(&a);
   camera->optic2grafic(&b);
   camera->optic2grafic(&c);
   camera->optic2grafic(&d);
   camera->optic2grafic(&e);
   camera->optic2grafic(&f);
   camera->optic2grafic(&g);
   camera->optic2grafic(&h);
   camera->optic2grafic(&center_proj);

   LOG(" A(%.2f,%.2f,%.2f) B(%.2f,%.2f,%.2f) C(%.2f,%.2f,%.2f) D(%.2f,%.2f,%.2f)\n",
       A.X,A.Y,A.Z,
       B.X,B.Y,B.Z,
       C.X,C.Y,C.Z,
       D.X,D.Y,D.Z);
   
   LOG(" E(%.2f,%.2f,%.2f) F(%.2f,%.2f,%.2f) G(%.2f,%.2f,%.2f) H(%.2f,%.2f,%.2f)\n",
       E.X,E.Y,E.Z,
       F.X,F.Y,F.Z,
       G.X,G.Y,G.Z,
       H.X,H.Y,H.Z);
        
   LOG(" a(%.2f,%.2f) b(%.2f,%.2f) c(%.2f,%.2f) d(%.2f,%.2f) e(%.2f,%.2f) f(%.2f,%.2f) g(%.2f,%.2f) h(%.2f,%.2f)\n",
       a.y,a.x,
       b.y,b.x,
       c.y,c.x,
       d.y,d.x,
       e.y,e.x,
       f.y,f.x,
       g.y,g.x,
       h.y,h.x);

   /** Prepare object segments*/
   set_orig_end(&s1,a,b);
   set_orig_end(&s2,b,c);
   set_orig_end(&s3,c,d);
   set_orig_end(&s4,d,a);

   set_orig_end(&s5,e,f);
   set_orig_end(&s6,f,g);
   set_orig_end(&s7,g,h);
   set_orig_end(&s8,h,e);

   set_orig_end(&s9,e,a);
   set_orig_end(&s10,f,b);
   set_orig_end(&s11,g,c);
   set_orig_end(&s12,d,h);
  
   polygon->num_seg=12;
   polygon->segments[0]=s1;
   polygon->segments[1]=s2;
   polygon->segments[2]=s3;
   polygon->segments[3]=s4;
   polygon->segments[4]=s5;
   polygon->segments[5]=s6;
   polygon->segments[6]=s7;
   polygon->segments[7]=s8;
   polygon->segments[8]=s9;
   polygon->segments[9]=s10;
   polygon->segments[10]=s11;
   polygon->segments[11]=s12;

   get_shadow(A,&As);
   get_shadow(B,&Bs);
   get_shadow(E,&Es);
   get_shadow(F,&Fs);

   camera->myproject(As, &as);
   camera->myproject(Bs, &bs);
   camera->myproject(Es, &es);
   camera->myproject(Fs, &fs);

   camera->optic2grafic(&as);
   camera->optic2grafic(&bs);
   camera->optic2grafic(&es);
   camera->optic2grafic(&fs);

   LOG(" as(%.2f,%.2f) bs(%.2f,%.2f) es(%.2f,%.2f) fs(%.2f,%.2f)\n",
       as.y,as.x,
       bs.y,bs.x,
       es.y,es.x,
       fs.y,fs.x);

   if (shadows_are_present())
   {
      /** Prepare shadow segments*/
      set_orig_end(&sh1,as,bs);
      set_orig_end(&sh2,bs,c);
      set_orig_end(&sh3,c,d);
      set_orig_end(&sh4,d,as);

      set_orig_end(&sh5,es,fs);
      set_orig_end(&sh6,fs,g);
      set_orig_end(&sh7,g,h);
      set_orig_end(&sh8,h,es);

      set_orig_end(&sh9,es,as);
      set_orig_end(&sh10,fs,bs);
      set_orig_end(&sh11,g,c);
      set_orig_end(&sh12,d,h);

      polygon->num_shadow_seg=12;
      polygon->shadow_segments[0]=sh1;
      polygon->shadow_segments[1]=sh2;
      polygon->shadow_segments[2]=sh3;
      polygon->shadow_segments[3]=sh4;
      polygon->shadow_segments[4]=sh5;
      polygon->shadow_segments[5]=sh6;
      polygon->shadow_segments[6]=sh7;
      polygon->shadow_segments[7]=sh8;
      polygon->shadow_segments[8]=sh9;
      polygon->shadow_segments[9]=sh10;
      polygon->shadow_segments[10]=sh11;
      polygon->shadow_segments[11]=sh12;
      
      for (i=0; i<polygon->num_seg; i++)
      {
         center_x += polygon->segments[i].orig.x;
         center_y += polygon->segments[i].orig.y;
         center_x += polygon->shadow_segments[i].orig.x;
         center_y += polygon->shadow_segments[i].orig.y;
      }

      // Calculate the geometric center of the polygon
      polygon->center.x = (float)center_x/24;
      polygon->center.y = (float)center_y/24;
   }
   else
   {
      polygon->num_shadow_seg = 0;
      for (i=0; i<polygon->num_seg; i++)
      {
         center_x += polygon->segments[i].orig.x;
         center_y += polygon->segments[i].orig.y;
      }

      // Calculate the geometric center of the polygon
      polygon->center.x = (float)center_x/12;
      polygon->center.y = (float)center_y/12;
   }
  
   // Set the body center
   polygon->body_center.x = center_proj.x;
   polygon->body_center.y = center_proj.y;
}

/** Received the model and its center and calculate the projection
 *  Polygon
 */
void Classifier::project_model(T3dmodel model, HPoint3D center, Tpolygon* polygon){

   float x_offset, y_offset, z_offset;
   HPoint3D A,B,C,D,E,F,G,H;
   HPoint2D a,b,c,d,e,f,g,h;
   Tsegment s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12;
   PinHoleCamera* camera = PinHoleCamera::Instance();
   
   x_offset = model.y/2;
   y_offset = model.z/2;
   z_offset = model.x/2;

   A.X = center.X-x_offset;
   A.Y = center.Y+y_offset;
   A.Z = center.Z+z_offset;
   A.H = 1;

   D.X = center.X-x_offset;
   D.Y = center.Y+y_offset;
   D.Z = center.Z-z_offset;
   D.H = 1;

   B.X = center.X+x_offset;
   B.Y = center.Y+y_offset;
   B.Z = center.Z+z_offset;
   B.H = 1;
  
   C.X = center.X+x_offset;
   C.Y = center.Y+y_offset;
   C.Z = center.Z-z_offset;
   C.H = 1;
  
   E.X = center.X-x_offset;
   E.Y = center.Y-y_offset;
   E.Z = center.Z+z_offset;
   E.H = 1;
  
   F.X = center.X+x_offset;
   F.Y = center.Y-y_offset;
   F.Z = center.Z+z_offset;
   F.H = 1;
  
   G.X = center.X+x_offset;
   G.Y = center.Y-y_offset;
   G.Z = center.Z-z_offset;
   G.H = 1;
  
   H.X = center.X-x_offset;
   H.Y = center.Y-y_offset;
   H.Z = center.Z-z_offset;
   H.H = 1;

   camera->myproject(A, &a);
   camera->myproject(B, &b);
   camera->myproject(C, &c);
   camera->myproject(D, &d);
   camera->myproject(E, &e);
   camera->myproject(F, &f);
   camera->myproject(G, &g);
   camera->myproject(H, &h);

   camera->optic2grafic(&a);
   camera->optic2grafic(&b);
   camera->optic2grafic(&c);
   camera->optic2grafic(&d);
   camera->optic2grafic(&e);
   camera->optic2grafic(&f);
   camera->optic2grafic(&g);
   camera->optic2grafic(&h);

   set_orig_end(&s1,a,b);
   set_orig_end(&s2,b,c);
   set_orig_end(&s3,c,d);
   set_orig_end(&s4,d,a);

   set_orig_end(&s5,e,f);
   set_orig_end(&s6,f,g);
   set_orig_end(&s7,g,h);
   set_orig_end(&s8,h,e);

   set_orig_end(&s9,e,a);
   set_orig_end(&s10,f,b);
   set_orig_end(&s11,g,c);
   set_orig_end(&s12,h,d);

   polygon->num_seg=12;
  
   polygon->segments[0]=s1;
   polygon->segments[1]=s2;
   polygon->segments[2]=s3;
   polygon->segments[3]=s4;
   polygon->segments[4]=s5;
   polygon->segments[5]=s6;
   polygon->segments[6]=s7;
   polygon->segments[7]=s8;
   polygon->segments[8]=s9;
   polygon->segments[9]=s10;
   polygon->segments[10]=s11;
   polygon->segments[11]=s12;
}

/**
 *
 */
bool Classifier::cv_point_in_model_projection(Tpolygon *projection, Tpoint2D p){
   // return (pointPolygonTest(get_points(projection), p, false) >= 0);
}

/**
 *
 */
bool Classifier::point_in_model_projection(Tpolygon *projection, Tpoint2D p){

   /** for object*/
   Tpolygon p1,p2,p3,p4,p5;
   
   /** for shadows*/
   Tpolygon s1,s2,s3,s4,s5;

   p1.num_seg = p2.num_seg = p3.num_seg = p4.num_seg = p5.num_seg = 4;
   s1.num_seg = s2.num_seg = s3.num_seg = s4.num_seg = s5.num_seg = 4;
  

   /** We check if the poin exists in all the faces but ABCD, since this is always
    *  hiden by the other faces. In the case of shadow we look in all the faces but
    *  the DCGH face since it's already checked in the object part.
    */ 

   /******** OBJECT FACES **********/
   /** EFGH Face*/ 
   p1.segments[0] = projection->segments[4];
   p1.segments[1] = projection->segments[5];
   p1.segments[2] = projection->segments[6];
   p1.segments[3] = projection->segments[7];

   /** ABFE Face*/ 
   p2.segments[0] = projection->segments[0];
   p2.segments[1] = projection->segments[9];
   p2.segments[2] = projection->segments[4];
   p2.segments[3] = projection->segments[8];

   /** DCGH Face*/ 
   p3.segments[0] = projection->segments[2];
   p3.segments[1] = projection->segments[10];
   p3.segments[2] = projection->segments[6];
   p3.segments[3] = projection->segments[11];

   /** EADH Face*/ 
   p4.segments[0] = projection->segments[8];
   p4.segments[1] = projection->segments[3];
   p4.segments[2] = projection->segments[11];
   p4.segments[3] = projection->segments[7];

   /** FBCG Face*/ 
   p5.segments[0] = projection->segments[9];
   p5.segments[1] = projection->segments[1];
   p5.segments[2] = projection->segments[10];
   p5.segments[3] = projection->segments[5];

   /******** SHADOW FACES **********/
   /** EFGH Face*/ 
   s1.segments[0] = projection->shadow_segments[4];
   s1.segments[1] = projection->shadow_segments[5];
   s1.segments[2] = projection->shadow_segments[6];
   s1.segments[3] = projection->shadow_segments[7];

   /** ABFE Face*/ 
   s2.segments[0] = projection->shadow_segments[0];
   s2.segments[1] = projection->shadow_segments[9];
   s2.segments[2] = projection->shadow_segments[4];
   s2.segments[3] = projection->shadow_segments[8];

   /** DCGH Face*/ 
   s3.segments[0] = projection->shadow_segments[2];
   s3.segments[1] = projection->shadow_segments[10];
   s3.segments[2] = projection->shadow_segments[6];
   s3.segments[3] = projection->shadow_segments[11];

   /** EADH Face*/ 
   s4.segments[0] = projection->shadow_segments[8];
   s4.segments[1] = projection->shadow_segments[3];
   s4.segments[2] = projection->shadow_segments[11];
   s4.segments[3] = projection->shadow_segments[7];

   /** ABCD Face*/ 
   s5.segments[0] = projection->shadow_segments[0];
   s5.segments[1] = projection->shadow_segments[1];
   s5.segments[2] = projection->shadow_segments[2];
   s5.segments[3] = projection->shadow_segments[3];

   return (pip(&p1,p) 
           || pip(&p2,p)
           || pip(&p3,p)
           || pip(&p4,p)
           || pip(&p5,p)
           || pip(&s1,p)
           || pip(&s2,p)
           || pip(&s4,p)
           || pip(&s5,p)
           || pip(&s3,p));
}

/**
 *
 */
int Classifier::point_in_model_projection_no_shadows(Tpolygon *projection, Tpoint2D p){

   Tpolygon p1,p2,p3,p4,p5;

   p1.num_seg = p2.num_seg = p3.num_seg = p4.num_seg = p5.num_seg = 4;

   /** EFGH Face*/ 
   p1.segments[0] = projection->segments[4];
   p1.segments[1] = projection->segments[5];
   p1.segments[2] = projection->segments[6];
   p1.segments[3] = projection->segments[7];

   /** FBCG Face*/ 
   p5.segments[0] = projection->segments[9];
   p5.segments[1] = projection->segments[1];
   p5.segments[2] = projection->segments[10];
   p5.segments[3] = projection->segments[5];

   /** FBCG Face*/ 
   p2.segments[0] = projection->segments[9];
   p2.segments[1] = projection->segments[1];
   p2.segments[2] = projection->segments[10];
   p2.segments[3] = projection->segments[5];

   /** EADH Face*/ 
   p4.segments[0] = projection->segments[8];
   p4.segments[1] = projection->segments[3];
   p4.segments[2] = projection->segments[11];
   p4.segments[3] = projection->segments[7];

   /** ABFE Face*/ 
   p2.segments[0] = projection->segments[0];
   p2.segments[1] = projection->segments[9];
   p2.segments[2] = projection->segments[4];
   p2.segments[3] = projection->segments[8];

   /** DCGH Face*/ 
   p3.segments[0] = projection->segments[2];
   p3.segments[1] = projection->segments[10];
   p3.segments[2] = projection->segments[6];
   p3.segments[3] = projection->segments[11];
  
   return (pip(&p1,p) 
           || pip(&p2,p) 
           || pip(&p3,p)
           || pip(&p4,p)
           || pip(&p5,p));
}

/**
 *
 */
void Classifier::match_model_projection_with_blob(Vehicle *vehicle, 
                                                  Tpolygon* projection,
                                                  int* blob_minos_hyp,
                                                  int* blob_and_hyp,
                                                  int* hyp_minos_blob,
                                                  int* hyp,
                                                  int* blob){
   int a,b,c,hypotesis,moved;
   int i,j;
   Tpoint2D p, left_corner, right_corner, left_most_corner, right_most_corner;
   int step_u=0,step_v=0;

   a = b = c = hypotesis = moved = 0;

   /** Calculate the left most and right most corners*/ 
   get_polygon_corners(projection, shadows_are_present(), &left_corner, &right_corner);

   /** get the left_most_corner*/
   left_most_corner.x = floor(min(vehicle->get_left_corner().x, left_corner.x));
   left_most_corner.y = floor(min(vehicle->get_left_corner().y, left_corner.y));

   /** get the left_most_corner*/
   right_most_corner.x = floor(max(vehicle->get_right_corner().x, right_corner.x));
   right_most_corner.y = floor(max(vehicle->get_right_corner().y, right_corner.y));

   vehicle->set_left_most_corner(left_most_corner);
   vehicle->set_right_most_corner(right_most_corner);

   LOG("%d:%d -- %d:%d \n",
       left_most_corner.y,
       left_most_corner.x,
       right_most_corner.y,
       right_most_corner.x);
      
   LOG("polygon %d:%d -- %d:%d \n",
       left_corner.y,
       left_corner.x,
       right_corner.y,
       right_corner.x);
      
   LOG("vehicle %d:%d -- %d:%d \n",
       vehicle->get_left_corner().y,
       vehicle->get_left_corner().x,
       vehicle->get_right_corner().y,
       vehicle->get_right_corner().x);
      
   LOG("%d -- %d \n",
       right_most_corner.x-left_most_corner.x,
       right_most_corner.y-left_most_corner.y);
  
   step_u=(right_most_corner.x-left_most_corner.x)/10;
   if (0 == step_u)
      step_u=1;

   step_v=(right_most_corner.x-left_most_corner.x)/10;
   if (0 == step_v)
      step_v=1;

   cv::Rect rect = cv::Rect(vehicle->get_rect());

   for (i=left_most_corner.y; i<right_most_corner.y; i += step_u)
      for (j=left_most_corner.x; j<right_most_corner.x; j += step_v)
      {
         p.y = i;
         p.x = j;
         bool point_in_hypotesis = point_in_model_projection(projection,p);

         if (point_in_hypotesis)
            hypotesis++;

         if (MovModel::Instance()->movement_on(p) && rect.contains(cv::Point(j,i)))
         {
           moved++;
            
           if (point_in_hypotesis)
           {
             b++;
           }
           else
           {
             a++;
           }
         }
         else if (point_in_hypotesis)
         {
            c++;
         }
      }

   *blob_minos_hyp = a;
   *blob_and_hyp = b;
   *hyp_minos_blob = c;
   *hyp = hypotesis;
   *blob = moved;
}


/**
 *
 */
float _div_(int a, int b){
   return (float)a/b;
}

/**
 *
 */
float exp_prob(int a, int b, int c){

   float e,B;
   e = ((float)(b+1))/((float)(a+c+1));
   B = pow(b,e);
   return (float)B/(float)(a+c+B+1);
}

/**
 *
 */
float mult_prob(int a, int b, int c)
{
   float e,B;
   e = ((float)(b+1))/((float)(a+c+1));
   B = b*e;
   return (float)B/(float)(a+c+B+1);
}

/**
 *
 */
float Classifier::calculate_model_prob(Vehicle* vehicle, Tpoint2D center, tvehicle_category category){

   int a,b,c,hyp,blob;

   HPoint3D vehicle_3d_center;
   HPoint2D centro;
   Tpolygon model_proj;
   float class_prob=0;
   PinHoleCamera* camera = PinHoleCamera::Instance();

   if (category == INVALID_VEHICLE_CLASS)
   {
      printf("Received an invalid model \n");
      return 0;
   }
 
   /** Get the vehicle 3d center*/
   centro.x = center.x;
   centro.y = center.y;
   camera->reproject(centro, &vehicle_3d_center, VehicleModel::get_category_dimension_x(category)/2);

   /** Project model*/
   memset(&model_proj, 0, sizeof(Tpolygon));
   project_model_and_shadow(category, vehicle_3d_center, &model_proj);
   match_model_projection_with_blob(vehicle, &model_proj, &a, &b, &c, &hyp, &blob);

   LOG(" a=%d b=%d c=%d hyp=%d blob=%d\n\n",a,b,c,hyp,blob);
  
   class_prob = mult_prob(a,b,c);
  
   return class_prob;
}

/**
 *
 */
bool Classifier::project_vehicle(Tpoint2D center, tvehicle_category category, Tpolygon* model_proj){

   HPoint3D vehicle_3d_center;
   HPoint2D centro;
   PinHoleCamera* camera = PinHoleCamera::Instance();

   if (category == INVALID_VEHICLE_CLASS)
   {
      printf("Received an invalid model \n");
      return false;
   }
 
   /** Get the vehicle 3d center*/
   centro.x = center.x;
   centro.y = center.y;
   camera->reproject(centro, &vehicle_3d_center, VehicleModel::get_category_dimension_x(category)/2);

   /** Project model*/
   memset(model_proj, 0, sizeof(Tpolygon));
   project_model_and_shadow(category, vehicle_3d_center, model_proj);
  
   return true;
}

/**
 *
 */
void Classifier::shift_model(Tpolygon *polygon, int delta_u, int delta_v){

   int i;
   
   /** Shift the projection (including the shadow) to match with the blob center*/
   for (i=0; i<polygon->num_seg; i++)
   {
      polygon->segments[i].orig.x += delta_u;
      polygon->segments[i].orig.y += delta_v;
      polygon->segments[i].end.x += delta_u;
      polygon->segments[i].end.y += delta_v;
      
      polygon->shadow_segments[i].orig.x += delta_u;
      polygon->shadow_segments[i].orig.y += delta_v;
      polygon->shadow_segments[i].end.x += delta_u;
      polygon->shadow_segments[i].end.y += delta_v;
   }
   
   polygon->center.x += delta_u;
   polygon->center.y += delta_v;

   polygon->body_center.x += delta_u;
   polygon->body_center.y += delta_v;

}

/**
 *
 */
void Classifier::classify_vehicle(Vehicle* vehicle, colorspaces::Image& inputImage){

   HPoint3D vehicle_3d_center;
   HPoint2D centro;

   tvehicle_category category,best_matched_class=INVALID_VEHICLE_CLASS;
   int a,b,c,hyp,blob;
   float max_prob=0,class_prob=0;
   float distance=0;
   int counter=0;
   int delta_u,delta_v;
   PinHoleCamera* camera = PinHoleCamera::Instance();

   if (vehicle->ocluded)
   {
     CATEGORY_LOG("(o) Results of vehicle %d life=%d \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \n"
                  ,vehicle->get_id()
                  ,vehicle->get_life()
                  ,vehicle->get_class_prob(MOTORCYCLE)
                  ,vehicle->get_class_prob(CAR)
                  ,vehicle->get_class_prob(VAN)
                  ,vehicle->get_class_prob(TRUCK)
                  ,vehicle->get_class_prob(BUS));
     return;
   }
   
   CATEGORY_LOG("Classification of vehicle %d life=%d\n", vehicle->get_id(), vehicle->get_life());
  
   for (category=MOTORCYCLE; category<MAX_MODELS; category++)
   {
      centro.x = vehicle->get_center_of_mass().x;
      centro.y = vehicle->get_center_of_mass().y;
      camera->reproject(centro, &vehicle_3d_center, VehicleModel::get_category_dimension_x(category)/2);
      memset(&models_projection[category], 0, sizeof(Tpolygon));

      project_model_and_shadow(category, vehicle_3d_center, &models_projection[category]);
      distance = DISTANCE_2D(models_projection[category].center, vehicle->get_center_of_mass());

      LOG(" %s distance is -- %.2f \n",VehicleModel::get_model_desc(category),distance);

      counter = 0;
      while (distance>MIN_SEP_DISTANCE && counter<MAX_LOOKUP_ITER)
      {
         delta_u = vehicle->get_center_of_mass().x-models_projection[category].center.x;
         delta_v = vehicle->get_center_of_mass().y-models_projection[category].center.y;

         LOG(" delta is (%d:%d)\n",delta_u,delta_v);

         centro.x = models_projection[category].body_center.x+delta_u;
         centro.y = models_projection[category].body_center.y+delta_v;
         camera->reproject(centro, &vehicle_3d_center, VehicleModel::get_category_dimension_x(category)/2);
         
         project_model_and_shadow(category, vehicle_3d_center, &models_projection[category]);
         distance = DISTANCE_2D(models_projection[category].center, vehicle->get_center_of_mass());

         LOG("iter -- %d \n",counter);
         LOG(" using 2d center (%.2f:%.2f)\n", centro.x, centro.y);
         LOG(" distance is -- %.2f \n",distance);

         counter++;
      }

      // if (out_of_image(&models_projection[category]))
      // {
      //    continue;
      // }
      
      /**************/
      match_model_projection_with_blob(vehicle, &models_projection[category], &a, &b, &c, &hyp, &blob);
      
      /** Update the class probability*/
      class_prob = mult_prob(a,b,c) * ((float)b/(float)blob);
      
      
      if (class_prob > max_prob)
      {
         max_prob = class_prob;
         best_matched_class = category;
         vehicle->set_current_3d_center(vehicle_3d_center);
      }

      CATEGORY_LOG("prob of %s p=%.2f acm=%.2f (%d : %d : %d -- %d %d %.2f)\n\n",
                   VehicleModel::get_model_desc(category),
                   class_prob,
                   vehicle->get_class_prob(category),
                   a,
                   b,
                   c,
                   blob,
                   hyp,
                   (float)b/(float)blob);
   }

   CATEGORY_LOG("Results of vehicle %d life=%d \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \n"
                ,vehicle->get_id()
                ,vehicle->get_life()
                ,vehicle->get_class_prob(MOTORCYCLE)
                ,vehicle->get_class_prob(CAR)
                ,vehicle->get_class_prob(VAN)
                ,vehicle->get_class_prob(TRUCK)
                ,vehicle->get_class_prob(BUS));

   // if (CLASS_DEBUG)
   //    vehicle->projection = models_projection[Graphics::get_selected_model_for_debuging()];
   // else

   // Second layer classification (to distinguish between trucks and buses)
   vehicle->set_category(best_matched_class);
   vehicle->set_projection(&models_projection[best_matched_class]);
   
   if ((best_matched_class == TRUCK) || (best_matched_class == BUS))
   {
      vehicle->inc_class_prob(m_svm_classifier.classify_vehicle(vehicle, inputImage), max_prob);
   }
   else
   {
      vehicle->inc_class_prob(best_matched_class, max_prob);
   }

   
   CATEGORY_LOG("---------\n");
}

/**
 *
 */
void Classifier::test_pip_projection(Tpoint2D p){
   if (point_in_model_projection(&models_projection[CAR],p))
      printf("dentro\n");
   else
      printf("fuera\n");
}

/**
 *
 */
void Classifier::get_smallest_vehicle_dimension(unsigned int* dimensions, float* length, Tpoint2D center)
{

   PinHoleCamera* camera = PinHoleCamera::Instance();   
   Tpolygon smallest_obj_projection;
   Tpoint2D p, left_corner, right_corner;
   HPoint3D vehicle_3d_center;
   unsigned int pixels_sum=0;
   Road* road = Road::Instance();
   int i,j;
   HPoint2D hcenter;

   /*** Init to null values*/
   *dimensions=-1;
   *length=-1;

   hcenter.x = center.x;
   hcenter.y = center.y;
   
   camera->reproject(hcenter, &vehicle_3d_center, VehicleModel::get_category_dimension_x(MOTORCYCLE)/2); 
   memset(&smallest_obj_projection, 0, sizeof(Tpolygon));
   project_model_and_shadow(MOTORCYCLE, vehicle_3d_center, &smallest_obj_projection);

   /** Calculate the left most and right most corners*/ 
   get_polygon_corners(&smallest_obj_projection, shadows_are_present(), &left_corner, &right_corner);

   if (left_corner.x >= 0
       && left_corner.y >= 0 
       && right_corner.x >= 0 
       && right_corner.y >= 0)
     {
       for (i=left_corner.y; i<right_corner.y; i ++)
       {
         for (j=left_corner.x; j<right_corner.x; j++)
         {
           p.y = i;
           p.x = j;
           if (point_in_model_projection(&smallest_obj_projection, p))
           {
             pixels_sum++;
           }
         }
       }
   
       *dimensions=pixels_sum;
       *length = (float)(right_corner.y-left_corner.y)/2;
     }
}
}
