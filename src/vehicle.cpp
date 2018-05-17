
#include <string.h>
#include <stdio.h>

#include "vehicle.h"
#include "stats.h"

namespace trafficmonitor{

/**
 *
 */
Vehicle::Vehicle():Blob(){
   init();
}

/**
 *
 */
Vehicle::Vehicle(const Blob &obj): Blob(){}

/**
 *
 */
void Vehicle::init(){
   Blob::init();
   category = INVALID_VEHICLE_CLASS;
   memset(categories_prob_acm,0,sizeof(categories_prob_acm));
   memset(&projection,0,sizeof(projection));
}

/**
 *
 */
void Vehicle::set_projection(Tpolygon* p_projection){

   if (p_projection)
      projection = *p_projection;
}

/**
 *
 */
tvehicle_category Vehicle::get_matched_class() const
{
   tvehicle_category veh=INVALID_VEHICLE_CLASS;
   tvehicle_category matched_class=INVALID_VEHICLE_CLASS;
   float score=-1;

   for (veh=MOTORCYCLE; veh<MAX_MODELS; veh++)
   {
      if (categories_prob_acm[veh] != 0 && categories_prob_acm[veh] > score)
      {
         matched_class=veh;
         score = categories_prob_acm[veh];
      }
   }
  
   return matched_class;
}

/**
 *
 */
void Vehicle::start_tracking(timeval timestamp)
{
   Blob::start_tracking(timestamp);
   category = INVALID_VEHICLE_CLASS;
   memset(categories_prob_acm, 0.0, sizeof(categories_prob_acm));
   memset(&projection, 0, sizeof(projection));
}

/**
 *
 */
void Vehicle::end_of_tracking(timeval timestamp)
{
   Stats* stats = Stats::Instance();
   
   /** NOTE: end of tracking MUST be called before
    *  collecting stats
    */
   Blob::end_of_tracking(timestamp);
   if (stats)
   {
     if (is_valid_tracking())
     {
       stats->collect_stats(*this);
     }
     else
     {
       stats->collect_invalid_vehicle(*this);
     }
   }
   
   init();
}

/**
 *
 */
void Vehicle::inc_class_prob(tvehicle_category veh, float class_prob)
{
   if (VALID_CATEGORY(veh))
   {
      categories_prob_acm[veh] += class_prob;
   }
   else
   {
      printf("** Trying to increment an invalid category %d\n", veh);
   }
}

/**
 *
 */
float Vehicle::get_class_prob(tvehicle_category category){
   if (VALID_CATEGORY(category))
      return categories_prob_acm[category];
   else
      return 0;
}
}
