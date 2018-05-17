#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "camera_model.h"
#include "progeo.h"


#define DEBUG 0

#define MAX_POINTS 4
#define DEFAULT_CAMERA_HEIGTH 8.5
#define DEFAULT_CAMERA_FOCAL_DISTANCE 500

using namespace std;

/**
 *
 */
PinHoleCamera::PinHoleCamera()
{
   calibration_rectangle.resize(MAX_POINTS);

   camera_cfg_file = "camera.cfg";

   //TODO: make it configurable
   strcpy(camera.name,"TrafficMonitorCamera");

   m_camera_pos.X = 0;
   m_camera_pos.Y = 0;
   m_camera_pos.Z = DEFAULT_CAMERA_HEIGTH;
   m_camera_pos.H = 1;

   m_camera_foa.X = 0;
   m_camera_foa.Y = 0;
   m_camera_foa.Z = 0;
   m_camera_foa.H = 1;

   m_focal_distance_x = DEFAULT_CAMERA_FOCAL_DISTANCE;
   m_focal_distance_y = DEFAULT_CAMERA_FOCAL_DISTANCE;

   m_tilt = CV_PI/4;
   m_pan = 0;
   m_roll = 0;
}

/**
 *
 */
void PinHoleCamera::init(const std::vector<Tpoint2D>& road_points)
{
   for(unsigned int i=0; i<calibration_rectangle.size(); i++)
   {
      calibration_rectangle[i] = road_points[i];
   }
}

/**
 * gets the calibration of the camera from a file
 */
int PinHoleCamera::load_cam_line(FILE *myfile)
{

#define limit 256
#define MM_TO_METERS(a) (a/1000)
#define METERS_TO_MM(a) (a*1000)

   char word1[limit],word2[limit];
   int i=0;
   char buffer_file[limit];

   buffer_file[0]=fgetc(myfile);

   if (feof(myfile)) return EOF;

   if (buffer_file[0]==(char)255) return EOF;

   if (buffer_file[0]=='#')
   {
      while(fgetc(myfile)!='\n'); return 0;
   }

   if (buffer_file[0]==' ')
   {
      while(buffer_file[0]==' ') buffer_file[0]=fgetc(myfile);
   }

   if (buffer_file[0]=='\t')
   {
      while(buffer_file[0]=='\t') buffer_file[0]=fgetc(myfile);
   }

   /* Captures a line and then we will process it with sscanf checking that the last character is \n.
    *   We can't doit with fscanf because this function does not difference \n from blank space.
    */
   while((buffer_file[i]!='\n')
         && (buffer_file[i] != (char)255)
         &&  (i<limit-1) )
   {
      buffer_file[++i]=fgetc(myfile);
   }

   if (i >= limit-1)
   {
      printf("%s...\n", buffer_file);
      printf ("Line too long in config file!\n");
      exit(-1);
   }
   buffer_file[++i]='\0';

   if (sscanf(buffer_file,"%s",word1)!=1)
   {
      return 0;
      /* return EOF; empty line*/
   }
   else
   {
      if (strcmp(word1,"positionX")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_pos.X=MM_TO_METERS((float)atof(word2));
      }
      else if (strcmp(word1,"positionY")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_pos.Y=MM_TO_METERS((float)atof(word2));
      }
      else if (strcmp(word1,"positionZ")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_pos.Z=MM_TO_METERS((float)atof(word2));
      }
      else if (strcmp(word1,"positionH")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_pos.H=(float)atof(word2);
      }
      else if (strcmp(word1,"FOApositionX")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_foa.X=MM_TO_METERS((float)atof(word2));
      }
      else if (strcmp(word1,"FOApositionY")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_foa.Y=MM_TO_METERS((float)atof(word2));
      }
      else if (strcmp(word1,"FOApositionZ")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_foa.Z=MM_TO_METERS((float)atof(word2));
      }
      else if (strcmp(word1,"FOApositionH")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_camera_foa.H=(float)atof(word2);
      }
      else if (strcmp(word1,"roll")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_roll=(float)atof(word2);
      }
      else if (strcmp(word1,"f")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_focal_distance_x=(float)atof(word2);
         m_focal_distance_y=(float)atof(word2);
      }
      else if (strcmp(word1,"fx")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_focal_distance_x=(float)atof(word2);
      }
      else if (strcmp(word1,"fy")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         m_focal_distance_y=(float)atof(word2);
      }
      else if (strcmp(word1,"skew")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         camera.skew=(float)atof(word2);
      }
      else if (strcmp(word1,"u0")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         camera.u0=(float)atof(word2);
      }
      else if (strcmp(word1,"v0")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         camera.v0=(float)atof(word2);
      }
      else if (strcmp(word1,"columns")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         camera.columns=(int)atof(word2);
      }
      else if (strcmp(word1,"rows")==0)
      {
         sscanf(buffer_file,"%s %s",word1,word2);
         camera.rows=(int)atof(word2);
      }
   }
   return 1;
}


/**
 * gets the calibration of the camera from a file
 */
void PinHoleCamera::load_cam(string fich_in)
{
   FILE *entrada;
   int i;

   entrada=fopen(fich_in.c_str(),"r");

   if(entrada!=NULL)
   {
      do
      {
         i=load_cam_line(entrada);
      }
      while(i!=EOF);

      fclose(entrada);
   }
}

/**
 *
 */
void PinHoleCamera::reproject(HPoint2D in, HPoint3D *out, float Z){

   HPoint2D tmp;
   HPoint3D A;
   float mx,my,mz,lambda;

   /** Calculate the 3d point of the 2d point "in"*/
   tmp.x = in.x;
   tmp.y = in.y;
   tmp.h = 1;
   grafic2optic(&tmp);
   backproject(&A, tmp, camera);

   A.X = A.X/A.H;
   A.Y = A.Y/A.H;
   A.Z = A.Z/A.H;

   /** Using the parametric 3d line equation we connect the point A with
    *  the camera position and then we calculate the intersetion with the
    *  plane Z = vehile model center
    *
    *  camera_pos(0,0,camaera.position.Z)
    */
   mx = 0-A.X;
   my = 0-A.Y;
   mz = m_camera_pos.Z-A.Z;

   /** lambda is calculated using the Z coordinate*/
   out->Z = Z;
   lambda = (out->Z-m_camera_pos.Z)/mz;
   out->X = 0 + mx*lambda;
   out->Y = 0 + my*lambda;
   out->H = 1;

   if (DEBUG)
      printf(" Z=%.2f repr in(%.2f,%.2f) tmp(%.2f,%.2f) A(%.2f,%.2f,%.2f) out(%.2f,%.2f,%.2f)\n",
             Z,
             in.x,in.y,
             tmp.x,
             tmp.y,
             A.X,A.Y,A.Z,
             out->X,out->Y,out->Z);
}

/**
 *
 */
int PinHoleCamera::myproject(HPoint3D in, HPoint2D *out)
{
   return project(in, out, camera);
}

/**
 *
 */
void PinHoleCamera::config(const string camera_cfg_file)
{
   load_cam(camera_cfg_file);
   update_camera_config();
   display_camerainfo(camera);
   /** Store original values, just in case we want to restore them.*/
   camera_orig = camera;
}

/**
 *
 */
void PinHoleCamera::restore_config()
{
   camera = camera_orig;
   update_camera_matrix(&camera);
   display_camerainfo(camera);
}

/**
 *
 */
void PinHoleCamera::optic2grafic(HPoint2D *in){
   float tmp;

   tmp = in->y;
   in->y = camera.rows - 1 - in->x;
   in->x = tmp;
}

/**
 *
 */
void PinHoleCamera::grafic2optic(HPoint2D *in){

   float tmp;
   tmp = in->y;
   in->y = in->x;
   in->x = camera.rows - 1 - tmp;
}

/**
 *
 */
void PinHoleCamera::move_point(int i, int j)
{
   Tpoint2D tmp;
   float dist = INT_MAX;
   float tmp_dist=0;
   int pos=-1;

   tmp.y = i;
   tmp.x = j;

   /** Find the nearst point*/
   for(unsigned int i=0; i<calibration_rectangle.size(); i++)
   {
      tmp_dist = DISTANCE_2D(calibration_rectangle[i], tmp);
      if (tmp_dist<dist)
      {
         dist = tmp_dist;
         pos = i;
      }
   }

   /** If the nearst point then we move the current point to this location*/
   if (pos>=0)
   {
      calibration_rectangle[pos] = tmp;
   }
}

/**
 *
 */
bool PinHoleCamera::autocalib()
{
   printf("******** AUTOCALIB ********\n");

   /** Object segments*/
   Tsegment s1,s2,s3,s4,s5;
   Tpoint2D ptmp;

   find_convex_hull(calibration_rectangle);
   order_points(calibration_rectangle);

   s1.orig = calibration_rectangle[0];
   s1.end = calibration_rectangle[1];

   s2.orig = calibration_rectangle[1];
   s2.end = calibration_rectangle[2];

   s3.orig = calibration_rectangle[2];
   s3.end = calibration_rectangle[3];

   s4.orig = calibration_rectangle[3];
   s4.end = calibration_rectangle[0];

   s4.end = calibration_rectangle[0];

   calculate_line(&s1);
   calculate_line(&s2);
   calculate_line(&s3);
   calculate_line(&s4);

   printf("======================== \n");

   get_intersec(s2,s4,m_vanishing_p1);

   s5.orig = m_vanishing_p1;
   ptmp = m_vanishing_p1;
   ptmp.x = ptmp.x-10;
   s5.end = ptmp;
   calculate_line(&s5);

   get_intersec(s3,s5,m_vanishing_p2);

   // do coordinates conversion
   m_vanishing_p1.x -= (camera.columns/2);
   m_vanishing_p1.y -= (camera.rows/2);
   m_vanishing_p2.x -= (camera.columns/2);
   m_vanishing_p2.y -= (camera.rows/2);

   float focal_distance = sqrt(-((m_vanishing_p1.y*m_vanishing_p1.y)+(m_vanishing_p1.x*m_vanishing_p2.x)));

   if (!isnan(focal_distance))
   {
      m_focal_distance_x = focal_distance;
      m_focal_distance_y = focal_distance;

      m_tilt = atan(-m_vanishing_p1.y/focal_distance);
      m_pan = atan(-m_vanishing_p1.x*cos(m_tilt)/focal_distance);
      // We are supposing a 0 roll
      m_roll = 0;

      m_camera_foa.Y = m_camera_pos.Z/tan(m_tilt);
      m_camera_foa.X = m_camera_foa.Y*cos(m_pan)*sin(m_pan);
      m_camera_foa.Z = 0;
      m_camera_foa.H = 1;

      update_camera_config();
   }
}

/**
 *
 */
void PinHoleCamera::update_camera_config()
{
   camera.position=m_camera_pos;
   camera.foa=m_camera_foa;

   camera.roll=m_roll;

   camera.fdistx = m_focal_distance_x;
   camera.fdisty = m_focal_distance_y;
   camera.skew=0;
   camera.u0 = camera.rows/2;
   camera.v0 = camera.columns/2;

   update_camera_matrix(&camera);
}

/**
 *
 */
void PinHoleCamera::update_camera_heigth(float new_value)
{
   printf("******** updating camera hight ********\n");
   m_camera_pos.Z=new_value;
   update_camera_config();
};

/**
 *
 */
void PinHoleCamera::save_config()
{
   std::string filename = camera_cfg_file;
   std::stringstream ss;
   ss.clear();

   ss << "#extrinsics, position"                                        << endl
      << "positionX "             << int (METERS_TO_MM(m_camera_pos.X)) << endl
      << "positionY "             << int (METERS_TO_MM(m_camera_pos.Y)) << endl
      << "positionZ "             << int (METERS_TO_MM(m_camera_pos.Z)) << endl
      << "positionH "             << int (m_camera_pos.H) << endl
      << ""                       << endl
      << "#orientation"           << endl
      << "FOApositionX "          << int (METERS_TO_MM(m_camera_foa.X)) << endl
      << "FOApositionY "          << int (METERS_TO_MM(m_camera_foa.Y)) << endl
      << "FOApositionZ "          << int (METERS_TO_MM(m_camera_foa.Z)) << endl
      << "FOApositionH "          << int (m_camera_foa.H) << endl
      << "roll "                  << int (m_roll)                       << endl
      << ""                                                             << endl
      << "#Intrensics"                                                  << endl
      << "fx "                    << int(m_focal_distance_x)            << endl
      << "fy "                    << int(m_focal_distance_y)            << endl
      << "skew "                  << 0                                  << endl
      << "u0 "                    << int(camera.rows/2)                 << endl
      << "v0 "                    << int(camera.columns/2)              << endl
      << "columns "               << camera.columns                     << endl
      << "rows "                  << camera.rows                        << endl;

   cout << endl
        << "Saving Camera configuration:"
        << endl << endl
        << ss.str()  << endl;

   ofstream out(filename.c_str(), ios::out | ios::binary);
   if(!out)
   {
      cout << "Cannot open output file.\n";
   }
   else
   {
      out << ss.str();
      out.close();
   }
}
