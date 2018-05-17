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

#ifndef CARSPEED_DEFS_H
#define CARSPEED_DEFS_H

/*******************CONSTANTES*********************/

#define PUSHED 1
#define RELEASED 0
#define carSpeedVER "Car Speed"

/*Constantes para el algortimo de generacion y seguimiento*/

#define RANGO_PREDICCIONES 6
#define RANGO_VELOCIDADES 200
#define NUM_EXPLOTADORAS 200
#define MAX_RAZAS 50
#define HUE_INTERVALOS 30
/* Numero de particulas explotadores por raza */
#define MAX_PARTICLES 150


/*Numero máximo de particulas prometedoras*/
#define TOPE_PROMETEDORAS 500

/*Constantes para el control de los coches de gazebo*/
#define KMHTOMMS     (1000000.0 / 3600.0)
#define MMSTOKMH     (3600.0/1000000.0)
#define KMHTOCMS     (100000.0/3600)
#define MAX_VEHICLES 10

/* constantes de configuración 
   para el modo de ejecución del algoritmo*/

#define ENTRANTES 0
#define SALIENTES 1
/*************************FIN CONSTANTES****************************/

#endif // CARSPEED_DEFS_H
