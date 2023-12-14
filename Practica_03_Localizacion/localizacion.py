#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Universidad de La Laguna
# Escuela Superior de Ingeniería y Tecnología
# Grado en Ingeniería Informática
# Asignatura: Robótica Computacional
# Curso: 4º
# Práctica 3: Simulación de robots móviles holonómicos y no holonómicos
# Author Cheuk Kelly Ng Pante (alu0101364544@ull.edu.es)

# localizacion.py

import sys
from math import *
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
# ******************************************************************************
# Declaraci�n de funciones


def distancia(a, b):
    # Distancia entre dos puntos (admite poses)
    return np.linalg.norm(np.subtract(a[:2], b[:2]))


def angulo_rel(pose, p):
    # Diferencia angular entre una pose y un punto objetivo (baliza) 'p'
    w = atan2(p[1]-pose[1], p[0]-pose[0])-pose[2]
    while w > pi:
        w -= 2*pi
    while w < -pi:
        w += 2*pi
    return w


def mostrar(objetivos, ideal, trayectoria):
    # Mostrar objetivos y trayectoria:
    # plt.ion() # modo interactivo
    # Fijar los bordes del gr�fico
    objT = np.array(objetivos).T.tolist()
    trayT = np.array(trayectoria).T.tolist()
    ideT = np.array(ideal).T.tolist()
    bordes = [min(trayT[0]+objT[0]+ideT[0]), max(trayT[0]+objT[0]+ideT[0]),
              min(trayT[1]+objT[1]+ideT[1]), max(trayT[1]+objT[1]+ideT[1])]
    centro = [(bordes[0]+bordes[1])/2., (bordes[2]+bordes[3])/2.]
    radio = max(bordes[1]-bordes[0], bordes[3]-bordes[2])*.75
    plt.xlim(centro[0]-radio, centro[0]+radio)
    plt.ylim(centro[1]-radio, centro[1]+radio)
    # Representar objetivos y trayectoria
    idealT = np.array(ideal).T.tolist()
    plt.plot(idealT[0], idealT[1], '-g')
    plt.plot(trayectoria[0][0], trayectoria[0][1], 'or')
    r = radio * .1
    for p in trayectoria:
        plt.plot([p[0], p[0]+r*cos(p[2])], [p[1], p[1]+r*sin(p[2])], '-r')
        # plt.plot(p[0],p[1],'or')
    objT = np.array(objetivos).T.tolist()
    plt.plot(objT[0], objT[1], '-.o')
    plt.show()
    input()
    plt.clf()


def localizacion(balizas, real, ideal, centro, radio, mostrar=0):
    # Buscar la localizaci�n m�s probable del robot, a partir de su sistema
    # sensorial, dentro de una regi�n cuadrada de centro "centro" y lado "2*radio".
    
    # La imagen que almacenará todos los errores dados para todos los puntos en el radioo
    imagen = []
    # El error más pequeño encontrado que corresponde al punto más probable en el radio
    min_error = sys.maxsize
    # El punto más probable en el radio donde se encuentra el robot real
    mejor_punto = []
    # Incremento para recorrer todo el radio
    incremento = 0.05

    for i in np.arange(-radio, radio, incremento):
        imagen.append([])
        for j in np.arange(-radio, radio, incremento):
            # Obtenemos las componentes del punto actual
            x = centro[0] + j
            y = centro[1] + i
            # Movemos el robot ideal al punto actual del radio
            ideal.set(x, y, ideal.orientation)
            # Comprobamos la diferencia entre las medidas que tiene el nuevo robot ideal y las medidas del real
            error = real.measurement_prob(ideal.sense(balizas), balizas)
            # Guardamos el error en la matriz de imagen
            imagen[-1].append(error)
            # Comprobamos errores
            # Si el error dado es mejor que el error mínimo actual, actualizamos nuestro mejor_punto
            # y el valor del mejor error ya que significa que es más probable que nuestro robot esté
            # en el punto actual que estamos comprobando que en el último punto almacenado
            if (error < min_error):
                min_error = error
                mejor_punto = [x, y]
    # Colocamos el robot ideal en el nuevo punto donde pensamos que está ahora el robot real
    ideal.set(mejor_punto[0], mejor_punto[1], real.orientation)
    print("Mejor:", mejor_punto, min_error)

    if mostrar:
        plt.ion() # modo interactivo
        plt.xlim(centro[0]-radio, centro[0]+radio)
        plt.ylim(centro[1]-radio, centro[1]+radio)
        imagen.reverse()
        plt.imshow(imagen, extent=[centro[0]-radio, centro[0]+radio,
                                   centro[1]-radio, centro[1]+radio])
        balT = np.array(balizas).T.tolist()
        plt.plot(balT[0], balT[1], 'or', ms=10)
        plt.plot(ideal.x, ideal.y, 'D', c='#ff00ff', ms=10, mew=2)
        plt.plot(real.x, real.y, 'D', c='#00ff00', ms=10, mew=2)
        plt.show()
        input()
        plt.clf()

# ******************************************************************************


# Definici�n del robot:
P_INICIAL = [0., 4., 0.]  # Pose inicial (posici�n y orientacion)
V_LINEAL = .7         # Velocidad lineal    (m/s)
V_ANGULAR = 140.       # Velocidad angular   (�/s)
FPS = 10.        # Resoluci�n temporal (fps)

HOLONOMICO = 1
GIROPARADO = 0
LONGITUD = .2

# Definici�n de trayectorias:
trayectorias = [
    [[1, 3]],
    [[0, 2], [4, 2]],
    [[2, 4], [4, 0], [0, 0]],
    [[2, 4], [2, 0], [0, 2], [4, 2]],
    [[2+2*sin(.8*pi*i), 2+2*cos(.8*pi*i)] for i in range(5)]
]

# Definici�n de los puntos objetivo:
if len(sys.argv) < 2 or int(sys.argv[1]) < 0 or int(sys.argv[1]) >= len(trayectorias):
    sys.exit(sys.argv[0]+" <indice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definici�n de constantes:
EPSILON = .05               # Umbral de distancia
V = V_LINEAL/FPS            # Metros por fotograma
W = V_ANGULAR*pi/(180*FPS)  # Radianes por fotograma

ideal = robot()
ideal.set_noise(0, 0, .03)   # Ruido lineal / radial / de sensado
ideal.set(*P_INICIAL)     # operador 'splat'

real = robot()
real.set_noise(.01, .01, .01)  # Ruido lineal / radial / de sensado
real.set(*P_INICIAL)

random.seed(0)
tray_ideal = [ideal.pose()]  # Trayectoria percibida
tray_real = [real.pose()]     # Trayectoria seguida

tiempo = 0.
espacio = 0.
# random.seed(0)
# random.seed(datetime.now())
random.seed(int(datetime.now().timestamp()))

### localizacion() ###
localizacion(objetivos, real, ideal, ideal.pose(), 5, mostrar=1)
######################

for punto in objetivos:
    while distancia(tray_ideal[-1], punto) > EPSILON and len(tray_ideal) <= 1000:
        pose = ideal.pose()

        w = angulo_rel(pose, punto)
        if w > W:
            w = W
        if w < -W:
            w = -W
        v = distancia(pose, punto)
        if (v > V):
            v = V
        if (v < 0):
            v = 0

        if HOLONOMICO:
            if GIROPARADO and abs(w) > .01:
                v = 0
            ideal.move(w, v)
            real.move(w, v)
        else:
            ideal.move_triciclo(w, v, LONGITUD)
            real.move_triciclo(w, v, LONGITUD)
        tray_ideal.append(ideal.pose())
        tray_real.append(real.pose())
        
        # Relocalizar, que va a calcular cual es la posicion mas adecuada
        # para el robot idea en funcion de la posicion del robot real
        if (real.measurement_prob(ideal.sense(objetivos), objetivos) > EPSILON):
            localizacion(objetivos, real, ideal, ideal.pose(), 0.2, mostrar=0)

        espacio += v
        tiempo += 1

if len(tray_ideal) > 1000:
    print("<!> Trayectoria muy larga - puede que no se haya alcanzado la posicion final.")
print("Recorrido: "+str(round(espacio, 3))+"m / "+str(tiempo/FPS)+"s")
print("Distancia real al objetivo: " +
      str(round(distancia(tray_real[-1], objetivos[-1]), 3))+"m")
mostrar(objetivos, tray_ideal, tray_real)  # Representaci�n gr�fica
