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
 *  Authors : Victor Manuel Hidalgo Blazquez <hbmhidalgo@gmail.com>
 *            David Lobato Bravo <dav.lobato@gmail.com>
 *
 */

#ifndef CARSPEED_DATATYPES_H
#define CARSPEED_DATATYPES_H

#include "defs.h"
#include "planar_geom.h"

namespace trafficmonitor{
//enums and constants
enum mode { PRE_RACE, RACE };
  
//types
class Car{
public:
   int width;
   int bodyLength;
   int skirtLength;
};

typedef Tpoint2D Tpoint;

/*Displays para los marcadores de velocidad*/
typedef struct display{
   int free;
   float time;
   Tpoint unidad_pos;
   Tpoint decena_pos;
   Tpoint centena_pos;
   int velocidad;
   int r;
   int g;
   int b;
}Tdisplay;

typedef struct marcador{
   Tdisplay display1;
   Tdisplay display2;
   Tdisplay display3;
}Tmarcador;

/*Estructura para almacenar los datos de 
  la grafica*/

typedef struct grafico{
   float poblacion[50];
   float pose[50];
   int total;
}Tgrafico;

/*Estrucutras que he usado para poder sacar las graficas de la salud
  para la memoria*/
typedef struct grafica_salud{
   int pos_x;
   float salud_i;
   float salud_a;
   int ocupada;
}Tgrafica_salud;

typedef struct saludG{
   Tgrafica_salud salud[40];
   float velocidad;
   int total;
}TsaludG;
}//namespace

#endif //CARSPEED_DATATYPES_H
