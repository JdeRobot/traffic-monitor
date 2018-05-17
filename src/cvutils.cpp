/*
 *
 *  Copyright (C) 1997-2010 JDERobot Developers Team
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
 *  Authors : David Lobato Bravo <dav.lobato@gmail.com>
 *
 */

#include "cvutils.h"

namespace trafficmonitor {
//HELPER FUNCTIONS
//With this functions we can get the coordinates of a rectangle starting in
//different corners, always supposing 0,0 is Top-Left corner.

std::vector<cv::Point> pointsFromCornersTLBCW(const cv::Rect& rect) {
   std::vector<cv::Point> v(4);
   //top-left
   v[0] = cv::Point(rect.x, rect.y);
   v[1] = cv::Point(rect.width + rect.x, rect.y);
   v[2] = cv::Point(rect.width + rect.x, rect.height + rect.y);
   v[3] = cv::Point(rect.x, rect.height + rect.y);
   return v;
}

//build a vector with corner points from img Top-Rigth->Bottom Clockwise
std::vector<cv::Point> pointsFromCornersTRBCW(const cv::Rect& rect) {
   std::vector<cv::Point> v(4);
   //top-right
   v[0] = cv::Point(rect.width + rect.x, rect.y);
   v[1] = cv::Point(rect.width + rect.x, rect.height + rect.y);
   v[2] = cv::Point(rect.x, rect.height + rect.y);
   v[3] = cv::Point(rect.x, rect.y);
   return v;
}

//build a vector with corner points from img Top-Rigth->Bottom AntiClockwise
std::vector<cv::Point> pointsFromCornersBLTACW(const cv::Rect& rect) {
   std::vector<cv::Point> v(4);
   //top-right
   v[0] = cv::Point(rect.x, rect.height + rect.y);
   v[1] = cv::Point(rect.width + rect.x, rect.height + rect.y);
   v[2] = cv::Point(rect.width + rect.x, rect.y);
   v[3] = cv::Point(rect.x, rect.y);
   return v;
}

cv::Rect roiFromPoints(const std::vector<cv::Point>& points) {
      
   if (points.size() > 1)
   {
      cv::Point tl(points[0].x, points[0].y); //top-left corner min x, min y
      cv::Point br(tl); //bottom-right corner max x, max y

      std::vector<cv::Point>::const_iterator p_it;
      for (p_it = points.begin(); p_it != points.end(); p_it++)
      {
         tl = cv::Point(std::min(tl.x, p_it->x), std::min(tl.y, p_it->y));
         br = cv::Point(std::max(br.x, p_it->x), std::max(br.y, p_it->y));
      }
      return cv::Rect(tl, br);
   } else
      return cv::Rect();
}
}
