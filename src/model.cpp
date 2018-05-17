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
 *          : Redouane Kachach <redouane.kachach@gmail.com>
 *
 */
#include "model.h"
#include <cmath>
#include <algorithm>
#include <tr1/memory>

namespace trafficmonitor
{

/**
 *
 */
Model::Model(const colorspaces::Image& initialImg, const timeval timeStamp) throw () :timeStamp(timeStamp), m_firstFrame(true) {}

/**
 *
 */
void Model::setImage(const colorspaces::Image& img, const timeval timeStamp ) throw ()
{
  if (m_algorithm)
  {
    if (m_firstFrame)
    {
      m_algorithm->processFirstFrame(img, timeStamp);
      m_firstFrame = false;
    }
    else if (m_algorithm->iteration(img, timeStamp))
    {
      notifyObservers();
    }
  }
}

} // trafficmonitor namespace
