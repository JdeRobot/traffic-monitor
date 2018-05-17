#include <iostream>
#include <stdlib.h>

#include "database.h"
#include "vehicle_model.h"

using namespace std;

namespace trafficmonitor{

DataBase database;

/**
 *
 */
DataBase::DataBase()
{
}

/**
 *
 */
bool DataBase::connect()
{
  // try
  // {
  //   if (m_conn->connect("trafficmonitor_db", "localhost", "trafficmonitor", "trafficmonitor123"))
  //   {
  //     cout << "Database connected successfully" << endl;
  //   }
  //   else
  //   {
  //     cerr << "Error: Can't connect to the database" << endl;
  //   }
  // }
  // catch (const std::exception& e)
  // {
  //   cerr << "Error while connecting to the database. Details: " << e.what() << endl;
  //   exit(1);
  // }
  return true;
}

/**
 *
 */
bool DataBase::disconnect()
{
  return true;
}

/**
 *
 */
bool DataBase::store (const Vehicle& vehicle)
{
   std::stringstream sql_insert;

   // Query query = m_conn->query();

   // query << "INSERT INTO traffic_stats(vehicle_id, category, speed) VALUES("
   //       << vehicle.get_id()
   //       << ","
   //       << "\""
   //       << VehicleModel::get_model_desc(vehicle.get_matched_class())
   //       << "\""
   //       << ","
   //       << vehicle.get_speed()
   //       << ");";

   // cout << query;

   // const std::string& ins_query = sql_insert.str();

   // try
   // {
   //    query.execute();
   // }
   // catch (const std::exception &e)
   // {

   // }
}

} // trafficmonitor namespace
