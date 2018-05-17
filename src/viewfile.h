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
 *  Authors : Redouane Kacahch <redouane.kachach@gmail.com>
 *
 */

#ifndef VIEWFILE_H
#define VIEWFILE_H

#include <colorspacesmm.h>
#include "model.h"
#include "vehicle.h"
#include "view.h"

namespace trafficmonitor
{
class ViewFile: public View
{
public:
   ViewFile(Model& _model) throw();
   virtual ~ViewFile() throw() {}
   void update(const jderobotutil::Subject* o, jderobotutil::ObserverArg* arg = 0);

private:

   //algorithm selection dialog
   Gtk::Main gtkmain;
};

}//namespace

#endif /*VIEWFILE_H*/
