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
#include <string.h>
#include "rectify.h"

/** rectification algoritms will use GSL **/
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_multifit.h>

#define NUMEC  NUMPUNTOS*2

void Rectifier::invert(){

   int i,j,k;

   memcpy(matriz_inversa, matriz_solucion, sizeof(double)*9);

   gsl_matrix_view m = gsl_matrix_view_array (matriz_inversa, 3, 3);
   gsl_permutation * p = gsl_permutation_alloc (3);
   gsl_matrix* sol = gsl_matrix_alloc(3,3);

   int s;
   gsl_linalg_LU_decomp (&m.matrix, p, &s);
   gsl_linalg_LU_invert (&m.matrix, p, sol);

   k=0;
   for (i=0; i<3; i++)
   {
      for (j=0; j<3; j++)
      {
         matriz_inversa[k++] = gsl_matrix_get(sol,i,j);
      }
   }
}

void Rectifier::regresion_lineal_multiple(double a_data[NUMEC*8], double b_data[NUMEC]){

   int i,j,k,aux;
   double chisq;
   gsl_matrix *X, *cov;
   gsl_vector *y, *c;
   gsl_multifit_linear_workspace * work;

   X = gsl_matrix_alloc(NUMEC,8);
   y = gsl_vector_alloc(NUMEC);
   c = gsl_vector_alloc (8);
   cov = gsl_matrix_alloc (8,8);

   /* Prepramos la matriz de muestras

      NOTA: El sistema de ecuaciones a resolver contiene
      8 incognitas en vez de nueve. La novena incognita es
      igual a alfa (constante), y hay que inicializarla sino
      La solucion siempre sera el vector nulo
   */

   for (i=0; i<NUMEC; i++)
   {
      for (j=0; j<8; j++)
      {
         aux = i*8+j;
         gsl_matrix_set(X,i,j,a_data[aux]);
      }
   }

   /* Inicializamos el verctor de muestras */
   for (k=0; k<NUMEC; k++)
   {
      gsl_vector_set(y,k,b_data[k]);
   }

   /* Inicializamos y resolver el sistema sobredimensioando */
   work = gsl_multifit_linear_alloc (NUMEC,8);
   gsl_multifit_linear (X,  y, c, cov, &chisq, work);
   gsl_multifit_linear_free (work);

   /* sacamor por pantalla la suma de los cuadrados del error cometido
      a la hora de optimizar la matriz solución. */

   // printf(" Error cometido en la rectificacion = %g\n",chisq);

   /** copiamos la solución */
   for (i=0; i<8; i++)
   {
      matriz_solucion[i] = gsl_vector_get(c,i);
      /*printf(" -> %g\t",matriz_solucion[i]);*/
   }
}

/* Dados dos puntos p(x,y) y p'(x',y') obtiene dos
   ecuaciones fruto de la correspondencia entre los dos pnts
*/
void Rectifier::obtener_ecuacion(Tpoint2D p ,
                                 Tpoint2D p_prima,
                                 int **ecuacion){

   /* alfa es el C9 de la matriz resultado, de momento es constante !!!*/
   int alfa = 1;

   /**
    * Rellenamos la primera fila con los siguientes coeficientes :
    *
    * Ecuacion1 -> x.x'.C7 + y.x'.C8 - C1.x -C2.y + x' -C3 -C9.x'  = 0
    * Ecuacion2 -> x.y'.C7 + y.y'.C8 + y' - C4.x -C5.y - C6 -C9.y' = 0
    */

   /* Ecuación 1 */
   ecuacion[0][0] = -p.x;
   ecuacion[0][1] = -p.y;
   ecuacion[0][2] = -1;
   ecuacion[0][3] = 0;
   ecuacion[0][4] = 0;
   ecuacion[0][5] = 0;
   ecuacion[0][6] = p.x*p_prima.x;
   ecuacion[0][7] = p.y*p_prima.x;
   ecuacion[0][8] = -p_prima.x*alfa;

   /* Ecuación 1 */
   ecuacion[1][0] = 0;
   ecuacion[1][1] = 0;
   ecuacion[1][2] = 0;
   ecuacion[1][3] = -p.x;
   ecuacion[1][4] = -p.y;
   ecuacion[1][5] = -1;
   ecuacion[1][6] = p_prima.y*p.x;
   ecuacion[1][7] = p_prima.y*p.y;
   ecuacion[1][8] = -p_prima.y*alfa;
}

void Rectifier::resolver_sistema_de_ecuaciones(){

   int k,i,j;
   int **ecuacion_lineal;
   double a_data[8*NUMEC];
   double b_data[NUMEC];
   int sistema_lineal_ecuaciones[NUMEC][9];

   ecuacion_lineal = (int**) malloc(2*sizeof(int*));
   ecuacion_lineal[0] = (int*) malloc(9*sizeof(int));
   ecuacion_lineal[1] = (int*) malloc(9*sizeof(int));

   /* recoremos los dos arrays con los putnos almacenados, y vamos obteniendo
      las ecuaciones para cada par de puntos. Cada par de pnts da lugar a dos ecuaciones.
   */
   for (i=0; i<NUMPUNTOS; i++)
   {
      obtener_ecuacion(pnts_elegidos_en_img_rectificada[i], pnts_elegidos_en_img_entrada[i], ecuacion_lineal);

      /** copiamos la ecuacion obtenida al sistema lineal sobre dimensionado*/
      for (j=0; j<9; j++)
      {
         sistema_lineal_ecuaciones[i*2][j] = ecuacion_lineal[0][j];
         sistema_lineal_ecuaciones[i*2+1][j] = ecuacion_lineal[1][j];
      }
   }

   /** vamos a resolver Ax=b*/

   /** copiamos la matriz "A" */
   k = 0;
   for (i=0; i<NUMEC; i++)
   {
      for (j=0; j<8; j++)
      {
         a_data[k++] = sistema_lineal_ecuaciones[i][j];
      }
   }

   /** Copiamos el vector "b" (la ultima columna)*/
   for (j=0; j<NUMEC; j++)
      b_data[j] = sistema_lineal_ecuaciones[j][8];

   /*resuelve_sistema_lineal(a_data, b_data);*/
   regresion_lineal_multiple(a_data, b_data);

}

/* Dado un punto en la imagen origen, calcula su correpondiente
   en la imagen rectificada
*/
Tpoint2D Rectifier::calcular_correspondencia(Tpoint2D in){


   /*
     Formulas Base para pasar de un punto (x,y) -> (x',y'). Nuestro H(3x3) en este
     caso es "matriz_solucion" que es un array plano de 9 posiciones

     x' = (h1*x + h2*y + h3)/(h7*x + h8*y + h9)
     y' = (h4*x + h5*y + h6)/(h7*x + h8*y + h9)

   */
   Tpoint2D pout;
   pout.x = (int) (matriz_solucion[0]*in.x + matriz_solucion[1]*in.y + matriz_solucion[2])/
      (matriz_solucion[6]*in.x + matriz_solucion[7]*in.y + matriz_solucion[8]);

   pout.y = (int) (matriz_solucion[3]*in.x + matriz_solucion[4]*in.y + matriz_solucion[5])/
      (matriz_solucion[6]*in.x + matriz_solucion[7]*in.y + matriz_solucion[8]);

   return pout;
}


/* Dado un punto en la imagen rectificada, calcula su correpondiente
   en la imagen original
*/
Tpoint2D Rectifier::calcular_correspondencia_inv(Tpoint2D in){

   /*
     Formulas Base para pasar de un punto (x,y) -> (x',y'). Nuestro H(3x3) en este
     caso es "matriz_solucion" que es un array plano de 9 posiciones

     x' = (h1*x + h2*y + h3)/(h7*x + h8*y + h9)
     y' = (h4*x + h5*y + h6)/(h7*x + h8*y + h9)

   */
   Tpoint2D pout;
   pout.x = (int) (matriz_inversa[0]*in.x + matriz_inversa[1]*in.y + matriz_inversa[2])/
      (matriz_inversa[6]*in.x + matriz_inversa[7]*in.y + matriz_inversa[8]);

   pout.y = (int) (matriz_inversa[3]*in.x + matriz_inversa[4]*in.y + matriz_inversa[5])/
      (matriz_inversa[6]*in.x + matriz_inversa[7]*in.y + matriz_inversa[8]);

   return pout;
}

/**
 *
 */
void Rectifier::rectify(Tpoint2D *original_points, Tpoint2D *rectified_points){

   int i=0;

   for (i=0; i<4; i++)
   {
      pnts_elegidos_en_img_entrada[i].x = original_points[i].x;
      pnts_elegidos_en_img_entrada[i].y = original_points[i].y;
      pnts_elegidos_en_img_rectificada[i].x = rectified_points[i].x;
      pnts_elegidos_en_img_rectificada[i].y = rectified_points[i].y;

      // printf(" -- %d-%d ** %d-%d -- \n",
      //        pnts_elegidos_en_img_entrada[i].x,pnts_elegidos_en_img_entrada[i].y,
      //        pnts_elegidos_en_img_rectificada[i].x,pnts_elegidos_en_img_rectificada[i].y);
   }

   matriz_solucion[8]=1.0;
   resolver_sistema_de_ecuaciones();
   invert();
}

/**
 *
 */
void Rectifier::rectify(const colorspaces::Image& image, colorspaces::Image& outimage){

   Tpoint2D p_in, p;
   int i,j,in_idx,out_idx;
   unsigned char r,g,b;

   // printf("in(%d:%d) out(%d:%d)\n",image.width,image.height,outimage.width,outimage.height);

   /**
    * Construimos la imagen recitificada para ello:
    * Recoremos la imagen rectificada, para cada punto buscamos su correspondiente
    * en la imagen de entrada, si este cae fuera lo pintamos en blanco. Si cae dentro
    * lo copiamos (pintamos un pixel del mismo color en la imagen rectificada).
    */
   for (i=0; i<outimage.height; i++)
   {
      for (j=0; j<outimage.width; j++)
      {
         p_in.y = i;
         p_in.x = j;
         p = calcular_correspondencia(p_in);
         // printf("corres(%d:%d)\n",p.x,p.y);

         /**
          * Si el punto resultante cae dentro de la imagen lo dibujamos
          * sino lo pintamos en blando
          */
         out_idx = i*outimage.width+j;

         if ((p.x>=0) && (p.y>=0) && (p.x<=image.width) && (p.y<=image.height))
         {
            in_idx = p.y*image.width+p.x;
            r = image.data[in_idx*3] ;
            g = image.data[in_idx*3+1];
            b = image.data[in_idx*3+2];

            outimage.data[out_idx*3] = r ;
            outimage.data[out_idx*3+1] = g;
            outimage.data[out_idx*3+2] = b;
         }
         else
         {
            outimage.data[out_idx*3] = 255;
            outimage.data[out_idx*3+1] = 255;
            outimage.data[out_idx*3+2] = 255;
         }
      }
   }

   // printf("EXIT: in(%d:%d) out(%d:%d)\n",image.width,image.height,outimage.width,outimage.height);
}
