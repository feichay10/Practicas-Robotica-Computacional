#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Universidad de La Laguna
# Escuela Superior de Ingeniería y Tecnología
# Grado en Ingeniería Informática
# Asignatura: Robótica Computacional
# Curso: 4º
# Práctica: Cinemática directa mediante Denavit-Hartenberg
# Author Cheuk Kelly Ng Pante (alu0101364544@ull.edu.es)

# Ejemplo:
# ./cin_dir_5_bien.py 0 0 0 0

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ******************************************************************************
# Declaración de funciones

def ramal(I,prev=[],base=0):
  # Convierte el robot a una secuencia de puntos para representar
  O = []
  if I:
    if isinstance(I[0][0],list):
      for j in range(len(I[0])):
        O.extend(ramal(I[0][j], prev, base or j < len(I[0])-1))
    else:
      O = [I[0]]
      O.extend(ramal(I[1:],I[0],base))
      if base:
        O.append(prev)
  return O

def muestra_robot(O,ef=[]):
  # Pinta en 3D
  OR = ramal(O)
  OT = np.array(OR).T
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  # Bounding box cúbico para simular el ratio de aspecto correcto
  max_range = np.array([OT[0].max()-OT[0].min()
                       ,OT[1].max()-OT[1].min()
                       ,OT[2].max()-OT[2].min()
                       ]).max()
  Xb = (0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten()
     + 0.5*(OT[0].max()+OT[0].min()))
  Yb = (0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten()
     + 0.5*(OT[1].max()+OT[1].min()))
  Zb = (0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten()
     + 0.5*(OT[2].max()+OT[2].min()))
  for xb, yb, zb in zip(Xb, Yb, Zb):
     ax.plot([xb], [yb], [zb], 'w')
  ax.plot3D(OT[0],OT[1],OT[2],marker='s')
  ax.plot3D([0],[0],[0],marker='o',color='k',ms=10)
  if not ef:
    ef = OR[-1]
  ax.plot3D([ef[0]],[ef[1]],[ef[2]],marker='s',color='r')
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.show()
  return

def arbol_origenes(O,base=0,sufijo=''):
  # Da formato a los origenes de coordenadas para mostrarlos por pantalla
  if isinstance(O[0],list):
    for i in range(len(O)):
      if isinstance(O[i][0],list):
        for j in range(len(O[i])):
          arbol_origenes(O[i][j],i+base,sufijo+str(j+1))
      else:
        print('(O'+str(i+base)+sufijo+')0\t= '+str([round(j,3) for j in O[i]]))
  else:
    print('(O'+str(base)+sufijo+')0\t= '+str([round(j,3) for j in O]))

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Orígenes de coordenadas:')
  arbol_origenes(O)
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def matriz_T(d,theta,a,alpha):
  # Calcula la matriz T (ángulos de entrada en grados)
  th=theta*pi/180; # Convertir de grados a radianes
  al=alpha*pi/180;
  # Matriz de transformación homogénea 4x4
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1] # Perspectiva (0) y escala (1)
         ]
# ******************************************************************************

plt.ion() # Modo interactivo
# Introducción de los valores de las articulaciones
nvar=5 # Número de variables
if len(sys.argv) != nvar+1:
  sys.exit('El número de articulaciones no es el correcto ('+str(nvar)+')')
p=[float(i) for i in sys.argv[1:nvar+1]]

# Parámetros D-H:
# Esta es la tabla del pdf
#        1     2      3          4           51     52         EF  
d  = [ p[0], -p[1],   2,        -1,          0,      0,        0]
th = [  270,   90, 90+p[2], -(90-p[3]), (90-p[4]), (90+p[4]),  90]
a  = [    0,    0,    0,        0,          1,    1,            1]
al = [   90,  180,   90,       -90,        -180,   -180,     -180]

# DH
# d  = [Oi-1          Zi-1 ^ Xi-1   Zi-1]
# th = [Xi-1          Xi            Zi-1]
# a  = [Zi-1 ^ Xi-1   Oi            Xi]
# al = [Zi-1          Zi            Xi]

# Orígenes para cada articulación. Con respecto a sigo mismos.
o00=[0,0,0,1]
o11=[0,0,0,1]
o22=[0,0,0,1]
o33=[0,0,0,1]
o44=[0,0,0,1]
o55_1=[0,0,0,1]
o55_2=[0,0,0,1]
oEF=[0,0,0,1]

# Cálculo matrices transformación
# Nos tenemos que construir dos matrices
# De 1 a 0
T10=matriz_T(d[0],th[0],a[0],al[0]) # De O1 a O0. Elementos de la primera columna

# De 2 a 1
T21=matriz_T(d[1],th[1],a[1],al[1]) # De O1' a O1. Elementos de la segunda columna
# De 2 a 0
T20=np.dot(T10,T21) # De O2 a O0, multiplicar ambas matrices

# De 3 a 2
T23=matriz_T(d[2],th[2],a[2],al[2]) # De O3 a O2. Elementos de la tercera columna
T30=np.dot(T20,T23) # De O3 a O0, multiplicar ambas matrices

# De 4 a 3
T34=matriz_T(d[3],th[3],a[3],al[3]) # De O4 a O3. Elementos de la cuarta columna
T40=np.dot(T30,T34) # De O4 a O0, multiplicar ambas matrices

# De 5_1 a 4
T5_14=matriz_T(d[4],th[4],a[4],al[4]) # De O4 a O3. Elementos de la quinta columna
T5_10=np.dot(T40,T5_14) # De O5_1 a O0, multiplicar ambas matrices

# De 5_2 a 4
T5_24=matriz_T(d[5],th[5],a[5],al[5]) # De O4 a O3. Elementos de la sexta columna
T5_20=np.dot(T40,T5_24) # De O5_1 a O0, multiplicar ambas matrices

# De EF a 4
Tef=matriz_T(d[6],th[6],a[6],al[6]) # De Oef a O4. Elementos de la octava columna
Tef0=np.dot(T40,Tef) # De Oef a O0, multiplicar ambas matrices

# Transformación de cada articulación
# Coordenadas de los puntos O1 y O2 con respecto a O0
o10 =np.dot(T10, o11).tolist() # Multiplicar el punto O11 por la matriz T01
o20 =np.dot(T20, o22).tolist() # Multiplicar el punto O22 por la matriz T02
o30 =np.dot(T30, o33).tolist() # Multiplicar el punto O33 por la matriz T30
o40 =np.dot(T40, o44).tolist() # Multiplicar el punto O33 por la matriz T30
o50_1 =np.dot(T5_10, o55_1).tolist() # Multiplicar el punto O33 por la matriz T30
o50_2 =np.dot(T5_20, o55_2).tolist() # Multiplicar el punto O33 por la matriz T30
oEF=np.dot(Tef0, oEF).tolist() # Multiplicar el punto O44 por la matriz T40

# Mostrar resultado de la cinemática directa
muestra_origenes([o00,o10,o20, o30, o40, [[o50_1], [o50_2]]], oEF)
muestra_robot   ([o00,o10,o20, o30, o40, [[o50_1], [o50_2]]], oEF)
input()
