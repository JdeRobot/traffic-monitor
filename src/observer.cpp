/*
 *  Copyright (C) 2016 David Lobato Bravo
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License v3
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *  Authors : David Lobato Bravo
 */
#include "observer.h"
#include <algorithm>

namespace jderobotutil {


  ObserverArg::ObserverArg(void *arg)
    : arg_(arg) {}

  Subject::Subject()
    : observers(0) {}

  void Subject::addObserver(ObserverPtr o){
    observers.push_back(o);
  }

  int Subject::countObservers() const{
    return observers.size();
  }

  void Subject::deleteObserver(ObserverPtr o){
    ObserverList::iterator it;

    if (observers.size() > 0){
      it = std::find(observers.begin(),observers.end(),o);
      if (it != observers.end())
        observers.erase(it);
    }
  }

  void Subject::deleteObservers(){
    observers.clear();
  }

  bool Subject::hasChanged() const{
    return changed;
  }

  void Subject::notifyObservers(){
    notifyObservers(0);
  }

  void Subject::notifyObservers(ObserverArg* arg){
    ObserverList::iterator it;

    for (it=observers.begin(); it != observers.end(); it++)
      (*it)->update(this,arg);
  }

  void Subject::clearChanged(){
    changed = false;
  }

  void Subject::setChanged(){
    changed = true;
  }

} //namespace
