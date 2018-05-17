/*
 *  Copyright (C) 1997-2008 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Authors : Redouane Kachach <redo.robot at gmail.com>
 *
 */
#ifndef _PLANAR_GEOM_
#define _PLANAR_GEOM_

#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "progeo.h"

using namespace cv;
using namespace std;

#define fsqr(a)(a*a)
#define MAX_SEGMENTS 12
#define DISTANCE_2D(a,b) sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) )
#define DISTANCE_3D_HPOINTS(a,b) sqrt( (a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y)  + (a.Z-b.Z)*(a.Z-b.Z))
#define MODULO(v) sqrt(v.a*v.a+v.b*v.b)
#define GET_LINE_POINT_DISTANCE(s,p)(abs(s.a*p.x - p.y + s.b)/sqrt(fsqr(s.a)+1))
#define SCALAR_PROD(v1,v2) ((v1.a*v2.a)+(v1.b*v2.b))

typedef Rect_<int> Rect;
//typedef cv::Point Tpoint2D;

struct Tpoint2D
{
Tpoint2D(int a, int b):y(a),x(b){};
Tpoint2D():y(0),x(0){};
   bool operator==(Tpoint2D a) const
      {
         return (a.x==x && a.y== y);
      }   
   int x;
   int y;
};

typedef Point3f Tpoint3D;

typedef struct{
   Tpoint2D orig;
   Tpoint2D end;
   float a;
   float b;
   int checked;
} Tsegment;

typedef struct{
   Tsegment segments[MAX_SEGMENTS];
   int num_seg;
   Tsegment shadow_segments[MAX_SEGMENTS];
   int num_shadow_seg;
   Tpoint2D center;
   Tpoint2D body_center;
} Tpolygon;

typedef struct myvector{
   float a;
   float b;
} Tvector;

typedef struct motiont_info{
   CvPoint2D32f orig;
   CvPoint2D32f end;
   Tvector vector;
   int    id;
} TmotionInfo;

typedef struct{
   float a;
   float b;
   float c;
} Tvector3D;

/**
 *
 */
void  get_polygon_corners(Tpolygon* pol, bool shadow, Tpoint2D* left_corner, Tpoint2D* right_corner);

/**
 *
 */
bool fit_line(vector<Point> points, Vec4f& line, int MIN_POINTS);

bool PointInRectangle(CvPoint2D32f p, CvRect rect);



/**
 *
 */
void line2seg(Vec4f& line, Tsegment& seg);

/**
 *
 */
bool pip(Tpolygon *polygon, Tpoint2D p);

/**
 *
 */
cv::Mat_<Tpoint2D> get_points(Tpolygon* pol);

/**
 *
 */
int fast_pip(Tpolygon *polygon, const Tpoint2D& p);

/**
 *
 */
bool get_intersec(Tsegment s1, Tsegment s2, Tpoint2D& intersec);

/**
 *
 */
void calculate_line(Tsegment *s);
void get_polar_repr(Tsegment *s, float* r, float* theta);

/**
 *
 */
void  get_lr_corners(Tpolygon* pol, Tpoint2D* left_corner, Tpoint2D* right_corner);

/**
 *
 */
void get_vector(Tvector *v, Tpoint2D p1, Tpoint2D p2);
void get_vector(Tvector *v, HPoint3D p1, HPoint3D p2);
void get_vector(Tvector *v, CvPoint2D32f p1, CvPoint2D32f p2);
void add_vector(const Tvector *v1, Tvector *v2);
Tvector norm_vector(Tvector v1);

/**
 *
 */
float angle_vectors(Tvector v1, Tvector v2);
float angle_vectors_2(Tvector v1, Tvector v2);
void find_convex_hull(vector<Tpoint2D>& points);


/**
 *
 */
void get_seg_center(Tsegment tmp_finish_line, Tpoint2D* seg_center);
void sort_4_points_clockwise(vector<Tpoint2D>& points);
void order_points(vector<Tpoint2D>& points);

#endif
