/*
 *
 *  Copyright (C) 1997-2010 JDE Developers Team
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
 *  Author : David Lobato Bravo <redouane.kachach@gmail.com>
 *
 */


#ifndef TRAFFIC_MONITOR_ICE
#define TRAFFIC_MONITOR_ICE

#include <slice/image.ice>

module trafficmonitor{

  /**
   * Camera interface
   */
  interface TrafficMonitorI extends jderobot::ImageProvider
  {
    void onMouseMov(int xCoor, int yCoor);
  };
                     
}; /*module*/

#endif /*TRAFFIC_MONITOR_ICE*/

