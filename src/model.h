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
#ifndef TRAFFICMONITOR_MODEL_H
#define TRAFFICMONITOR_MODEL_H

#include <observer.h>
#include <colorspacesmm.h>
#include <memory>

#include "trafficmonitor_algorithm.h"

namespace trafficmonitor {

class Model : public jderobotutil::Subject{

public:
   /**
    *
    */
   Model(const colorspaces::Image& initialImg, const timeval timeStamp) throw();

   /**
    * model input data
    */
   void setImage(const colorspaces::Image& img, const timeval timeStamp) throw();
   const colorspaces::Image& getImage() const throw() { return m_algorithm->get_state().current_frame;}
   const colorspaces::Image& getBackgroundImage() const throw() {return m_algorithm->get_state().background_image;};
   const TrafficMonitorAlgorithmState& getState() const throw() { return m_algorithm->get_state();}

   /**
    *
    */
   void setAlgorithm(TrafficMonitorAlgorithmPtr newAlg) throw() {if (m_algorithm = newAlg) notifyObservers();}
   const TrafficMonitorAlgorithmPtr& getAlgorithm() {return m_algorithm;};

   /**
    *
    */
   void setAlgorithmCfg(const TrafficMonitorAlgorithmConfig& newCfg) throw() {m_algorithm->set_cfg(newCfg);};
   const TrafficMonitorAlgorithmConfig& getAlgorithmCfg() throw() {return m_algorithm->get_cfg();};
   void updateAlgorithmCfg() throw() {m_algorithm->update_cfg();};
   void saveAlgorithmCfg() throw() {m_algorithm->save_cfg();};
   void init() {m_algorithm->init();};

   /**
    *
    */
   void processMouseMovement(int xCoor, int yCoor) throw() {m_algorithm->processMouseMovement(xCoor, yCoor);};

   /**
    * return a null pointer until algorithm is set
    */
   const TrafficMonitorAlgorithmPtr algorithm() const throw() { return m_algorithm;};

private:
   timeval timeStamp;
   TrafficMonitorAlgorithmPtr m_algorithm;
   bool m_firstFrame;
};
}//namespace
#endif /*TRAFFICMONITOR_MODEL_H*/
