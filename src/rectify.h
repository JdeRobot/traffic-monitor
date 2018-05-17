/*
 *  Copyright (C) 1997-2008 JDE Developers Team
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
 *  Authors : Redouane Kachach <redo.robot at gmail.com>
 *
 */

#ifndef __RECTIFIER__
#define __RECTIFIER__

#include <colorspacesmm.h>
#include "planar_geom.h"

/* cada punto da lugar a dos ecuaciones */
#define NUMPUNTOS 4


class Rectifier{

public:
   
   /**
    *
    */
   Rectifier(){};

   /**
    *
    */
   void rectify(Tpoint2D *orignal_points, Tpoint2D *rectified_points);
   void rectify(const colorspaces::Image& image, colorspaces::Image& outimage); 

   /**
    * Dado un punto en la imagen origen, calcula su correpondiente en la imagen rectificada 
    */
   Tpoint2D calcular_correspondencia(Tpoint2D in);
   // Tpoint2D calcular_correspondencia(cv::Point in);

   /**
    * Dado un punto en la imagen rectificada, calcula su correpondiente en la imagen original
    */
   Tpoint2D calcular_correspondencia_inv(Tpoint2D in);
   // Tpoint2D calcular_correspondencia_inv(cv::Point in);

private:
   
   /**
    *
    */
   void invert();
   
   /**
    *
    */
   void regresion_lineal_multiple(double a_data[], double b_data[]);

   /**
    *
    */
   void obtener_ecuacion(Tpoint2D p , 
                         Tpoint2D p_prima,
                         int **ecuacion);
   /**
    *
    */
   void resolver_sistema_de_ecuaciones();

   double matriz_solucion[9];
   double matriz_inversa[9];

   /* almacena los 4 puntos que han sido elegidos */
   Tpoint2D pnts_elegidos_en_img_entrada[NUMPUNTOS];
   Tpoint2D pnts_elegidos_en_img_rectificada[NUMPUNTOS];
};

#endif 
