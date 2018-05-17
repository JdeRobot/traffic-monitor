#ifndef STATS_H_H
#define STATS_H_H

#include <vector>

#include "singleton.h"
#include "circular_buffer.h"
#include "vehicle.h"

namespace trafficmonitor{

#define MAX_VEHICLES_HISTORY 15
#define MIN_VEHICLES_TRAJECTORIES 100

class Stats: public CSingleton<Stats>{

public:

   /**
    *
    */
   void init(bool classification_status);
   
   /**
    *
    */
   int get_model_count(tvehicle_category category);

   /**
    *
    */
   unsigned int get_last_elem_pos(){return last_vehicles.get_last_elem_pos();};

   /**
    *
    */
   const std::vector<Vec2f>& get_vehicles_trajectories(){return vehicles_trajectories;};

   /**
    *
    */
   int get_num_trajectories(){return num_trajectories;};

   /**
    *
    */
   void rewind(){return last_vehicles.rewind();};
      
   /**
    *
    */
   void reset_stats();

   /**
    *
    */
   Vehicle* get_next_vehicle();

   /**
    *
    */
   void collect_stats(Vehicle& vehicle);
   void collect_invalid_vehicle(Vehicle& vehicle);

   /**
    *
    */
  void set_classification_status(bool status){m_classification_active = status;}

protected:

   friend class CSingleton<Stats>;
   
private:

   Stats();

   /**
    *
    */
   void inc_model_count(tvehicle_category category);

   /**
    *
    */
   std::vector<Vec2f> vehicles_trajectories;
   int num_trajectories;

   /**
    *
    */
   int models_counts[MAX_MODELS];

   /** This is a circular array that stores the last vehicles that have left 
    *  the tracking zone it useed mainly to update the vehicle stats/counts like 
    *  the speed.
    */
   CircularBuffer<Vehicle> last_vehicles;

   /**
    *
    */
   unsigned int last_vehicles_head;
  bool m_classification_active;
};
}

#endif
