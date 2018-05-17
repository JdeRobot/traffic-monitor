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

#ifndef CARSPEED_CVUTILS_H
#define	CARSPEED_CVUTILS_H

#include <opencv2/highgui/highgui.hpp>
#include <vector>

namespace trafficmonitor {
//build a vector with corner points from rect Top-Left->Bottom Clockwise
std::vector<cv::Point> pointsFromCornersTLBCW(const cv::Rect& rect);
//build a vector with corner points from img Bottom-Left->Top Anti-Clockwise
std::vector<cv::Point> pointsFromCornersBLTACW(const cv::Rect& rect);
//build the minimum rectangular area enclosing points
cv::Rect roiFromPoints(const std::vector<cv::Point>& points);
}

#endif	/* CARSPEED_CVUTILS_H */

