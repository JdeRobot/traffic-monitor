/*
 *  Copyright (C) 2016 Kachach Redouane
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
 *  Authors : Kachach Redouane <redouane.kachach@gmail.com>
 */
#include <stdio.h>
#include <string.h>
#include "vehicle_model.h"

const char* VehicleModel::models_names[MAX_MODELS] = {
   "Invalid",
   "Motocycle",
   "Car",
   "Van",
   "Truck",
   "Bus",
};

const char* VehicleModel::models_description[MAX_MODELS] = {
   "INV",
   "Motorcycle",
   "Car",
   "Van",
   "Truck",
   "Bus",
};

/**
 *  The units are in meteres
 *
 *   z / 
 *    /  
 *   |  / 
 * x | / 
 *   |/____ 
 *      y 
 *
 */
T3dmodel VehicleModel::models_dimensions[MAX_MODELS] = {
  
   /** invalid vehicle*/
   {0, 0, 0},
   
   /** MOTO CYCLE*/
   {1.2, 0.5, 2},
   
   /** CAR **/
   {1.3, 1.6, 4},

   /** VAN **/
   {2, 2, 7},
   
   /** TRUCK **/
   {4, 2.5, 14},

   /** BUS **/
   {3, 2.5, 12.0},
};

/**
 *
 */
const char* VehicleModel::get_model_name(tvehicle_category category){
   
   if (VALID_CATEGORY(category))
      return models_names[category];
   else
      return models_names[INVALID_VEHICLE_CLASS];
}

/**
 *
 */
float VehicleModel::get_category_dimension_x(tvehicle_category category){
   
   if (VALID_CATEGORY(category))
      return models_dimensions[category].x;
   else
      return -1;
}

/**
 *
 */
float VehicleModel::get_category_dimension_y(tvehicle_category category){
   
   if (VALID_CATEGORY(category))
      return models_dimensions[category].y;
   else
      return -1;
}

/**
 *
 */
float VehicleModel::get_category_dimension_z(tvehicle_category category){
   
   if (VALID_CATEGORY(category))
      return models_dimensions[category].z;
   else
      return -1;
}

/**
 *
 */
const char* VehicleModel::get_model_desc(tvehicle_category category){
   
   if (VALID_CATEGORY(category))
      return models_description[category];
   else
   {
      return models_description[INVALID_VEHICLE_CLASS];
   }
}

/**
 *
 */
tvehicle_category VehicleModel::get_model_id(const std::string model_name){

   tvehicle_category i;
  
   for (i=MOTORCYCLE; i<MAX_MODELS; i++)
   {
      if (strcmp(model_name.c_str(), models_names[i]) == 0)
      {
         printf(" %s has been selected\n",models_names[i]);
         return i;
      }
   }
  
   return INVALID_VEHICLE_CLASS;
}
