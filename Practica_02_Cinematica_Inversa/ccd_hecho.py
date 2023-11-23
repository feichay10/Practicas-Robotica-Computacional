#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - Curso 2014/2015
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
from math import *
import math
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure(1)
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.show()
  # raw_input()
  plt.clf()

def matriz_T(d,th,a,al):
  # Calcula la matriz T (ángulos de entrada en grados)
  
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# Valores articulares arbitrarios para la cinemática directa inicial
th = [0.,0.,0.]               # Ángulos de las articulaciones
a = [5.,5.,5.]                # Longitud de las articulaciones
type_of_joint = [0, 1, 0]     # Tipo de articulación - (0) rotacional, (1) prismática
limits = [20.,20.,20]         # Límites de las articulaciones

print("th =", th)
print("a =", a)
print("type=", type)
print("limits=", limits)

L = 0 # variable para representación gráfica

# Bucle para calcular la longitud total del robot
for i in range(len(type_of_joint)):
  if type_of_joint[i] == 0 :
    L += a[i]
  else :
    L += limits[i][1]
EPSILON = .01

plt.ion() # modo interactivo

# Introducción del punto para la cinemática inversa
if len(sys.argv) != 3:
  sys.exit("python3 " + sys.argv[0] + " x y")
objetivo=[float(i) for i in sys.argv[1:]]

# Cálculos ********************************************************

O = [None] * (len(th) + 1)        # Reservamos estructura en memoria
O[0] = cin_dir(th,a)              # Calculamos la posicion inicial
print( "- Posicion inicial:")
muestra_origenes(O[0])

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  # Almacenamos la distancia para poder compararla en cada iteración y ver
  # si el desplazamiento es menor al umbral (epsilon)
  prev = dist

  # para cada desplazamiento (1 por articulación)
  for i in range(len(th)):
    # Escogemos el siguiente elemento por el final
    j = len(th) - i - 1
    # Comprobamos si es una articulación rotacional (0) o prismática (1)

    # --- Caso articulación rotacional
    if (type_of_joint[j] == 0):
      # Obtenemos los 2 puntos para formar el triángulo y tener un sistema de referencias.
      punto_final_robot = O[i][len(th)]
      punto_referencia = O[i][j]
      # Calculamos los ángulos desde el punto de referencia a el punto objetivo
      # y el último punto del robot. Calculamos la diferencia.
      alfa1 = atan2(objetivo[1] - punto_referencia[1], objetivo[0] - punto_referencia[0]) 
      alfa2 = atan2(punto_final_robot[1]  - punto_referencia[1], punto_final_robot[0] - punto_referencia[0])
      theta = alfa1 - alfa2
      # Movemos el ángulo ajustándonos a los valores umbrales.
      if (th[j] + theta <= limits[j][0]):
        th[j] = limits[j][0]
      elif (th[j] + theta >= limits[j][1]) :
        th[j] = limits[j][1]
      else:
        th[j] += theta
    
    # --- Caso articulación prismática
    else :
      summ_of_thetas = 0
      # Sumamos los ángulos de todos los componentes anteriores para saber el ángulo
      # en el que nos desplazamos
      for k in range(j) :
        summ_of_thetas += th[j]
      punto_final_robot = O[i][len(th)]
      # Calculamos la diferencia entre el punto final y el punto final del robot.
      final_eof_x = objetivo[0] - punto_final_robot[0]
      final_eof_y = objetivo[1] - punto_final_robot[1]
      # Aplicamos la fórmula dada en el PDF (que no acabo de entender pero funciona =^))
      # y vemos si al recalcular la distancia nos salimos de los valores umbrales.
      # Si no lo hacemos simplemente sumamos
      d = np.dot([cos(summ_of_thetas), sin(summ_of_thetas)], [final_eof_x, final_eof_y])
      if (a[j] + d <= limits[j][0]) :
        a[j] = limits[j][0]
      elif (a[j] + d >= limits[j][1]) :
        a[j] = limits[j][1]
      else :
        a[j] += d
    
    O[i + 1] = cin_dir(th,a)
    
  dist = np.linalg.norm(np.subtract(objetivo, O[-1][-1]))
  print("\n- Iteracion " + str(iteracion) + ':')
  muestra_origenes(O[-1])
  muestra_robot(O, objetivo)
  print("Distancia al objetivo = " + str(round(dist, 5)))
  iteracion += 1
  O[0] = O[-1]

if dist <= EPSILON:
    print("\n" + str(iteracion) + " iteraciones para converger.")
else:
    print("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print("- Umbral de convergencia epsilon: " + str(EPSILON))
print("- Distancia al objetivo:          " + str(round(dist, 5)))
print("- Valores finales de las articulaciones:")
for i in range(len(th)):
    print("  theta" + str(i+1) + " = " + str(round(th[i], 3)))
for i in range(len(th)):
    print("  L" + str(i+1) + "     = " + str(round(a[i], 3)))