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
#ifndef _TRAFFICMONITOR_SVM_
#define _TRAFFICMONITOR_SVM_

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include "vehicle_model.h"
#include "vehicle.h"

namespace trafficmonitor
{
  using std::string;

  class TrafficMonitorSVM
  {
  public:
    bool init(std::string path);
    tvehicle_category classify_vehicle(Vehicle* vehicle, colorspaces::Image& inputImage);

  private:

    Ptr<ml::SVM> m_SVM;
  };

}

#endif
