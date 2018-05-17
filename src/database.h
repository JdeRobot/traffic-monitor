#ifndef DATABASE_H
#define DATABASE_H

#include "vehicle.h"

namespace trafficmonitor{

class DataBase
{
public:

   /**
    *
    */
   DataBase();
   virtual ~DataBase(){};

   /**
    *
    */
   bool connect();

   /**
    *
    */
   bool disconnect();

   /**
    *
    */
   bool store (const Vehicle& vehicle);

private:

};

extern DataBase database;

}

#endif
