#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <algorithm>
#include <limits>

#include <opencv2/imgproc/imgproc.hpp>
#include "planar_geom.h"


using namespace std;

#define DEBUG 0

#define MAX_SEGMENTS_PER_POLYGON  4
#define INC_IDX(i) ((i+1)<MAX_SEGMENTS_PER_POLYGON?(i+1):0)
#define DEC_IDX(i) ((i-1)>=0?(i-1):MAX_SEGMENTS_PER_POLYGON-1)
#define PRECISION  0.05

//compares if the float f1 is equal with f2 and returns 1 if true and 0 if false
#define COMPARE_FLOAT(f1,f2) ((f1 - PRECISION < f2) && (f1 + PRECISION > f2))?1:0

/**
 * Obtiene los coeficientes a y b de la ecuacion de la recta que pasa por el segmento
 * s y = a*x +b 
 */
void calculate_line(Tsegment *s){
  
   float x1,x2,y1,y2;

   x1= s->orig.x;
   y1= s->orig.y;
   x2= s->end.x;
   y2= s->end.y;

   if (x2-x1 == 0)
   {
      x1=x1-0.1;
      x2=x2+0.1;
      s->a = (y2-y1)/(x2-x1);
      s->b = (y1*x2 - y2*x1)/(x2-x1);
   }
   else if (y2-y1 == 0)
   {
      s->a = 0;
      s->b = y2;
   }
   else
   {
      s->a = (y2-y1)/(x2-x1);
      s->b = (y1*x2 - y2*x1)/(x2-x1);
   }
}

/**
 * Obtiene los coeficientes a y b de la ecuacion de la recta que pasa por el segmento
 * s y = a*x +b 
 */
void get_polar_repr(Tsegment *s, float* r, float* theta){

   if (s && r && theta)
   {
      /** First we get the y = a.x + b format.*/
      calculate_line(s);

      /** Then we estimate r and theta.*/
      float tmp_theta = atan(-1/s->a);
      float tmp_r = s->b*sin(tmp_theta);

      /** Convert to deg */
      tmp_theta = (tmp_theta*180)/CV_PI;

      *theta = tmp_theta;
      *r = tmp_r;
   }
}


/**
 * Devuelve true si el el segmento horizontal que parte desde el punto
 * hazia la derecha podria intersectarse con el segmento s
 *
 */
int point_may_intersec(Tsegment s, Tpoint2D p){

   return ((p.y >= s.orig.y && p.y <= s.end.y)
           ||
           (p.y <= s.orig.y && p.y >= s.end.y)
      );
}


/**
 *  comprobar si el punto pertenecer al segmento horizontal
 */
int point_in_h_segment(Tsegment s, Tpoint2D p){

   return ((p.y == s.orig.y && p.y == s.end.y)
           &&
           (
              (p.x <= s.orig.x && p.x >= s.end.x)
              ||
              (p.x >= s.orig.x && p.x <= s.end.x)
              )
      );
}

/**
 *  comprobar si el punto pertenecer al segmento vertical
 */
int point_in_v_segment(Tsegment s, Tpoint2D p){

   return ((p.x == s.orig.x && p.x == s.end.x)
           &&
           (
              (p.y <= s.orig.y && p.y >= s.end.y)
              ||
              (p.y >= s.orig.y && p.y <= s.end.y)
              )
      );
}

/**
 * comprobamos si el punto pertenece al segmento
 */
int point_in_segment(Tsegment s, Tpoint2D p){
   return (point_in_v_segment(s,p) || point_in_h_segment(s,p));
}


/**
 *  Dados dos segmentos s1 y s2 devuelve true si intersectan, false
 *  en caso contrario
 */
int get_segments_intersec(Tsegment s1, Tsegment s2, Tpoint2D *intersec){

   float temp=s1.a-s2.a;

   if (s2.a-s1.a != 0)
   {
      intersec->x = (s2.b-s1.b)/temp;
      intersec->y = (s2.b*s1.a-s1.b*s2.a)/temp;
      return 1;
   }
  
   /** caso de rectas paralelas*/
   return 0;
}

/**
 *
 */
int line_traverse_segments(Tpoint2D q, Tsegment s1, Tsegment s2)
{

   if (DEBUG)
      printf("line_traves (%d:%d)  s1(%d:%d ---> %d:%d)  s2(%d:%d ---> %d:%d) %d\n",
             q.y,q.x,
             s1.orig.y, s1.orig.x, s1.end.y, s1.end.x, 
             s2.orig.y, s2.orig.x, s2.end.y, s2.end.x,
             ((q.y>s1.orig.y) && (q.y<s2.end.y))
             ||
             ((q.y>s2.end.y) && (q.y<s1.orig.y)));
  
   return ( 
      ((q.y>s1.orig.y) && (q.y<s2.end.y))
      ||
      ((q.y>s2.end.y) && (q.y<s1.orig.y))
      );
}

/**
 * Esta funcion comprueba si el punto de interseccion q es una esquina. Si es el caso entonces
 * puede haber dos posibilidades:
 *
 *
 * 1) La linea horizontal intersecta en una esquina "aguda" formada por los segmentos s1,s2
 *    En este caso se cuenta como se fuera dos intersecciones (el campo matched se deja a false 
 *    entonces se contara para ambos segmentos)
 * 
 *    s1\   /s2
 * p.....\q/.........
 *
 *
 * 2) La linea horizontal "atraviesa" la interseccion, en este caso solo cuenta una vez (el campo
 *    matched se pone a true para uno de los segmentos)
 *
 *   s1\  
 *      \
 * p.....q.........
 *        \
 *         \s2
 *
 */
void check_corners(Tpolygon *polygon, int i, Tpoint2D p, Tpoint2D q)
{

   Tsegment stmp;

   if (COMPARE_FLOAT(polygon->segments[i].orig.x,q.x)
       &&
       COMPARE_FLOAT(polygon->segments[i].orig.y,q.y)
      )
   {
      /** Si la esquina correspond con el origen, entonces cogemos el segmento anterior*/
      stmp = polygon->segments[DEC_IDX(i)];
      if (line_traverse_segments(p,stmp,polygon->segments[i]))
      {
         if (DEBUG)
            printf("the line traverse orig .. ignoring segment %d:%d ---> %d:%d\n",
                   polygon->segments[DEC_IDX(i)].orig.y,
                   polygon->segments[DEC_IDX(i)].orig.x,
                   polygon->segments[DEC_IDX(i)].end.y,
                   polygon->segments[DEC_IDX(i)].end.x);
                    
         polygon->segments[DEC_IDX(i)].checked=1;
      }
   }
   else if (COMPARE_FLOAT(polygon->segments[i].end.x,q.x)
            &&
            COMPARE_FLOAT(polygon->segments[i].end.y,q.y)
      )
   {
      /** Si la esquina correspond con el final, entonces cogemos el segmento posterior*/
      stmp = polygon->segments[INC_IDX(i)];
      if (line_traverse_segments(p, polygon->segments[i], stmp))
      {
         if (DEBUG)
            printf("the line traverse end .. ignoring segment %d:%d ---> %d:%d\n",
                   polygon->segments[INC_IDX(i)].orig.y,
                   polygon->segments[INC_IDX(i)].orig.x,
                   polygon->segments[INC_IDX(i)].end.y,
                   polygon->segments[INC_IDX(i)].end.x);
                    
         polygon->segments[INC_IDX(i)].checked=1;
      }
   }


}

/**
 * Poin in Polygon, devuelve true si el punto p esta DENTRO del poligono
 * false si esta fuera
 */
bool pip(Tpolygon *polygon, Tpoint2D p){

   int i;
   int num_intersec=0;
   Tsegment horiz;
   Tpoint2D q;

   /** hallar el segmento horizontal formado desde el punto p */
   horiz.a = 0;
   horiz.b = p.y;
   horiz.checked = false;

   for (i=0; i<polygon->num_seg; i++)
   {
      polygon->segments[i].checked = 0;
   } 
  
   for (i=0; i<polygon->num_seg; i++)
   {
      
      if(polygon->segments[i].checked)
      {
         if (DEBUG)
            printf("already checked segment %d:%d ---> %d:%d \n",
                   polygon->segments[i].orig.y,
                   polygon->segments[i].orig.x,
                   polygon->segments[i].end.y,
                   polygon->segments[i].end.x);
         continue;
      }

      /** Comprobamos si el punto esta dentro del segmento, si esta entonces
       *  paramos la busqueda forzando el numero de intersecciones a 1 (impar)
       *  ya que el punto pertenece al poligono
       */
      if(point_in_segment(polygon->segments[i],p))
      {
         if(DEBUG)
            printf("in segment %d:%d ---> %d:%d \n",
                   polygon->segments[i].orig.y,
                   polygon->segments[i].orig.x,
                   polygon->segments[i].end.y,
                   polygon->segments[i].end.x);
          
         num_intersec=1;
         break;
          
      }
      else if (DEBUG)
      {
         printf("out of segment %d:%d ---> %d:%d \n",
                polygon->segments[i].orig.y,
                polygon->segments[i].orig.x,
                polygon->segments[i].end.y,
                polygon->segments[i].end.x);
      }

      /** comprobar si el segmento intersecta con el segmento horizontal
       *  formado desde el punto recibido como argumento
       */
      if (point_may_intersec(polygon->segments[i],p)
          &&
          get_segments_intersec(polygon->segments[i], horiz, &q)
         )
      {
         if (DEBUG)
         {
            printf("checking intersection with segment %d:%d ---> %d:%d \n",
                   polygon->segments[i].orig.y,
                   polygon->segments[i].orig.x,
                   polygon->segments[i].end.y,
                   polygon->segments[i].end.x);
         }
          
         /** Los dos segmentos se intersectan si la intersección de sus rectas se 
          *  encuentra a la derecha del punto p
          */
         if (q.x > p.x)
         {
            /** Cuando la intersección es una esquina hay que tratarla de forma distinta*/
            check_corners(polygon,i,p,q);

            if (DEBUG)
               printf("esquina en %d:%d\n",(int)q.y,(int)q.x);
            
            num_intersec++;

         }
         else if (DEBUG)
            printf(" miss\n");
          
         if (DEBUG) 
            printf("\n");
      }
   }
  
   /** Si el numero de interseccion es IMPAR entonces el punto esta dentro del poligno*/
   return ((num_intersec % 2) != 0);
}

/**
 *
 */
void  get_polygon_corners(Tpolygon* pol, bool shadow, Tpoint2D* left_corner, Tpoint2D* right_corner){

   int i=0;

   // left_corner->x = INT_MAX;
   // left_corner->y = INT_MAX;
   left_corner->x = 3000;
   left_corner->y = 3000;
   right_corner->x = 0;
   right_corner->y = 0;

   /** Iterate over object segments*/
   for (i=0; i<pol->num_seg; i++)
   {
      left_corner->x = min(left_corner->x, min(pol->segments[i].orig.x, pol->segments[i].end.x));
      left_corner->y = min(left_corner->y, min(pol->segments[i].orig.y, pol->segments[i].end.y));
      right_corner->x = max(right_corner->x, max(pol->segments[i].orig.x, pol->segments[i].end.x));
      right_corner->y = max(right_corner->y, max(pol->segments[i].orig.y, pol->segments[i].end.y));
   }

   if (shadow)
   {
      /** Iterate over shadow segments*/
      for (i=0; i<pol->num_shadow_seg; i++)
      {
         left_corner->x = min(left_corner->x, min(pol->shadow_segments[i].orig.x, pol->shadow_segments[i].end.x));
         left_corner->y = min(left_corner->y, min(pol->shadow_segments[i].orig.y, pol->shadow_segments[i].end.y));
         right_corner->x = max(right_corner->x, max(pol->shadow_segments[i].orig.x, pol->shadow_segments[i].end.x));
         right_corner->y = max(right_corner->y, max(pol->shadow_segments[i].orig.y, pol->shadow_segments[i].end.y));
      }
   }
}

/**
 *
 */
void  get_lr_corners(Tpolygon* pol, Tpoint2D* left_corner, Tpoint2D* right_corner){

   int i=0;
   int lc_pos=-1;
   int rc_pos=-1;

   left_corner->x = INT_MAX;
   left_corner->y = INT_MAX;
   right_corner->x = 0;
   right_corner->y = 0;

   /** Iterate over object segments*/
   for (i=0; i<pol->num_seg; i++)
   {
      if (pol->segments[i].orig.x < left_corner->x && pol->segments[i].orig.y < left_corner->y)
      {
         left_corner->x = pol->segments[i].orig.x;
         left_corner->y = pol->segments[i].orig.y;
      }

      if (pol->segments[i].end.x < left_corner->x && pol->segments[i].end.y < left_corner->y)
      {
         left_corner->x = pol->segments[i].end.x;
         left_corner->y = pol->segments[i].end.y;
      }
      

      if (pol->segments[i].orig.x > right_corner->x && pol->segments[i].orig.y > right_corner->y)
      {
         right_corner->x = pol->segments[i].orig.x;
         right_corner->y = pol->segments[i].orig.y;
      }

      if (pol->segments[i].end.x > right_corner->x && pol->segments[i].end.y > right_corner->y)
      {
         right_corner->x = pol->segments[i].end.x;
         right_corner->y = pol->segments[i].end.y;
      }
   }
}

/**
 *
 */
void find_convex_hull(vector<Tpoint2D>& points){
   
   /** Copy the points to a temporal vector.*/
   vector<cv::Point> input_points(points.size());
   for(unsigned int i=0; i<points.size(); i++)
   {
      input_points[i].x = points[i].x;
      input_points[i].y = points[i].y;
   }

   cv::Mat_<cv::Point> tmp(input_points);
   vector<cv::Point> output_points;
   output_points.resize(input_points.size());
   convexHull(tmp, output_points, true);

   for(unsigned int i=0; i<output_points.size(); i++)
   {
      points[i].x = output_points[i].x;
      points[i].y = output_points[i].y;
   }

   // printf("input=%d (%d:%d) (%d:%d) (%d:%d) (%d:%d)\n",
   //        input_points.size(),
   //        input_points[0].x, input_points[0].y,
   //        input_points[1].x, input_points[1].y,
   //        input_points[2].x, input_points[2].y,
   //        input_points[3].x, input_points[3].y);
   
   // printf("output=%d (%d:%d) (%d:%d) (%d:%d) (%d:%d)\n",
   //        output_points.size(),
   //        output_points[0].x, output_points[0].y,
   //        output_points[1].x, output_points[1].y,
   //        output_points[2].x, output_points[2].y,
   //        output_points[3].x, output_points[3].y);
}


/**
 *
 */
void get_vector(Tvector *v, Tpoint2D p1, Tpoint2D p2){
   v->a = p2.x - p1.x;
   v->b = p2.y - p1.y;
}

/**
 *
 */
void add_vector(const Tvector *v1, Tvector *v2){
   v2->a += v1->a;
   v2->b += v1->b;
   v2->a = v2->a/2;
   v2->b = v2->b/2;
}

/**
 *
 */
void get_vector(Tvector *v, CvPoint2D32f p1, CvPoint2D32f p2){
   v->a = p2.x - p1.x;
   v->b = p2.y - p1.y;
}

/**
 *
 */
void get_vector(Tvector *v, HPoint3D p1, HPoint3D p2){
   v->a = p2.X - p1.X;
   v->b = p2.Y - p1.Y;
} 


/**
 *
 */
Tvector norm_vector(Tvector v1)
{
   Tvector norm_v1;
   
   float modulo_v1 = MODULO(v1);
   norm_v1.a = (float)v1.a/modulo_v1;
   norm_v1.b = (float)v1.b/modulo_v1;
   return norm_v1;
}

/**
 *
 */
float angle_vectors(Tvector v1, Tvector v2)
{
   Tvector norm_v1,norm_v2;

   float modulo_v1 = MODULO(v1);
   norm_v1.a = (float)v1.a/modulo_v1;
   norm_v1.b = (float)v1.b/modulo_v1;

   float modulo_v2 = MODULO(v2);
   norm_v2.a = (float)v2.a/modulo_v2;
   norm_v2.b = (float)v2.b/modulo_v2;
   
   float dot = SCALAR_PROD(norm_v1,norm_v2)/(MODULO(norm_v1)*MODULO(norm_v2));

   if (dot>1) dot=1;
   float angle=acos(dot);

   if (DEBUG)  
   {
      printf(" angle=%.2f dot=%.2f v1(%.3f:%.3f) v2(%.3f:%.3f)\n",angle,dot,v1.a,v1.b,v2.a,v2.b);
      
      printf(" modulo_v1=%.2f modulo_v2=%.2f norm_v1(%.3f:%.3f) norm_v2(%.3f:%.3f)\n",
             modulo_v1,
             modulo_v2,
             norm_v1.a,
             norm_v1.b,
             norm_v2.a,
             norm_v2.b);
      
      printf("angle is %.3f \n",angle*180/3.14);
   }
      
   return angle;
}

/**
 *
 */
float angle_vectors_2(Tvector v1, Tvector v2)
{
   Tvector norm_v1,norm_v2;

   float modulo_v1 = MODULO(v1);
   norm_v1.a = (float)v1.a/modulo_v1;
   norm_v1.b = (float)v1.b/modulo_v1;

   float modulo_v2 = MODULO(v2);
   norm_v2.a = (float)v2.a/modulo_v2;
   norm_v2.b = (float)v2.b/modulo_v2;
   
   float tmp=atan2(norm_v2.b,norm_v2.a) - atan2(norm_v1.b,norm_v1.a);

   if (DEBUG)  
   {
      printf(" modulo_v1=%.2f modulo_v2=%.2f norm_v1(%.3f:%.3f) norm_v2(%.3f:%.3f)\n",
             modulo_v1,
             modulo_v2,
             norm_v1.a,
             norm_v1.b,
             norm_v2.a,
             norm_v2.b);
      
      printf("angle is %.3f \n",tmp*180/3.14);
   }
      
   return tmp;
}


/**
 *
 */
void get_seg_center(Tsegment tmp_finish_line, Tpoint2D* seg_center)
{
   seg_center->x = (tmp_finish_line.orig.x + tmp_finish_line.end.x)/2;
   seg_center->y = (tmp_finish_line.orig.y + tmp_finish_line.end.y)/2;
}


/**
 *
 */
cv::Mat_<Tpoint2D> get_points(Tpolygon* pol)
{
   vector<Tpoint2D> tmp_points;

   for(int i=0; i<MAX_SEGMENTS; i++)
   {
      tmp_points.push_back(pol->segments[i].orig);
      tmp_points.push_back(pol->segments[i].end);
      tmp_points.push_back(pol->shadow_segments[i].orig);
      tmp_points.push_back(pol->shadow_segments[i].orig);
   }

   return (cv::Mat_<Tpoint2D>)tmp_points;
}

/**
 *
 */
int fast_pip(Tpolygon *polygon, const Tpoint2D& p){
   Tpoint2D top_lf_corner,top_r_corner;
   get_polygon_corners(polygon, false, &top_lf_corner, &top_r_corner);
   return (p.y >= top_lf_corner.y && p.y <= top_r_corner.y);
}

/**
 *
 */
void order_points(vector<Tpoint2D>& points)
{
   int i;
   int idx=-1,post_idx,pre_idx;
   Tpoint2D orig;
   vector<Tpoint2D> tmp_vector = points;
   int corner;

   /** Init to the max possible values*/
   orig.x = INT_MAX;
   orig.y = INT_MAX;
   
   /** look for the point with the minor Y coordinate*/
   for (i=0; i<points.size(); i++)
   {
      if (points[i].y <= orig.y)
      {
         idx = i;
         orig.y = points[i].y;
      }
   }

   if (idx>0)
   {
      pre_idx = idx-1;
      if (pre_idx == -1)
         pre_idx = 3;

      post_idx = (idx+1) % 4;

      if (points[pre_idx].y < points[post_idx].y)
         corner = pre_idx;
      else
         corner = idx;

      /** Rotate all the points to get them in the correct order*/
      int k=corner;
      for(i=0; i<points.size(); i++)
      {
         tmp_vector[i] = points[k];
         k = (k+1) % points.size();
      }

      points = tmp_vector;
   }
}

void line2seg(Vec4f& line, Tsegment& seg)
{
   Tpoint2D orig, end;
   orig.x = line[2];
   orig.y = line[3];
   end.x = line[2]+line[0]*20;
   end.y = line[2]+line[0]*20;

   seg.orig = orig;
   seg.end = end;

   calculate_line(&seg);
}

bool PointInRectangle(CvPoint2D32f p, CvRect rect)
{
   return ((p.x >= rect.x) && (p.x <= rect.x + rect.width)
           &&
           (p.y >= rect.y) && (p.y <= rect.y + rect.height));
}

bool fit_line(vector<Point> points, Vec4f& line, int MIN_POINTS)
{
   vector<Point> history;
   history.resize(points.size());
   history.clear();
   
   for(int i=0; i<points.size(); i++)
   {
      if (points[i].x>=0 && points[i].y>=0)
      {
         history.push_back(points[i]);
      }
   }

   if (history.size() >= MIN_POINTS)
   {
      cv::fitLine(history
                  ,line
                  ,2
                  ,0
                  ,0.01
                  ,0.01);
      return true;
   }

   return false;
}


/**
 *
 */
bool get_intersec(Tsegment s1, Tsegment s2, Tpoint2D& intersec){

   if ((s1.a-s2.a) != 0)
   {
      float delta = (float)(s2.b-s1.b)/(float)(s1.a-s2.a);
      intersec.x = delta;
      intersec.y = s1.a*delta + s1.b;
      printf(" delta=%.2f y=%.2f s1(%.2f,%.2f) s2(%.2f,%.2f)\n",
             delta,
             s1.a*delta + s1.b,
             s1.a,s1.b,
             s2.a,s2.b);
   }
   else
   {
      // paralel lines
      intersec.x = 0;
      intersec.y = 0;
      return false;
   }
}
