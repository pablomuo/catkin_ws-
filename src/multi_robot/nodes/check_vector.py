import rospy
import time
import numpy as np
import math
import random
import json
from math import pi
import os
import sys



class Check_vectors(object):
    """docstring for ."""

    def __init__(self):
        #aqui inicializo si tengo que inicializar algo, creo que si los pongio aqui, en agenst lo debo referenciar que son de aqui y poner como entrada
        self.dist_rec        = 0
        self.dist_lineal     = 0
        self.ang2            = 0

    def check_dist_angle(self, diff_time, vel_lineal, vel_ang):
        if vel_ang != 0:
            radio = abs(vel_lineal/vel_ang)
            dist_ang = abs(vel_ang*diff_time)                    # desplazamiento angular en radianes
            ang1 = (math.radians(180)-dist_ang)/2               #el triangulo isoceles
            self.ang2 = (math.radians(90)-ang1)                 # buscamos el angulo complementario del ang1
            # self.dist_lineal = self.get_point_position(radio, radio, dist_ang, 0)
            self.dist_lineal = np.sqrt(math.pow(radio,2)+math.pow(radio,2)-(2*radio*radio*math.cos(dist_ang)))

        else:
            self.dist_rec = diff_time*abs(vel_lineal)           #distancia que recorrge en recto tiempo x velocidad lineal
            self.ang2 = 0
            pass


    def get_dist_angulo_izq(self, ladocont1, ladocont2, indice_haz, ang_giro, ang_vect):
        #formula para calcular el lado de un triangulo no rectangulo, sabiendo dos lados y un angulo
        dist_calc = np.sqrt(math.pow(ladocont1,2)+math.pow(ladocont2,2)-(2*ladocont1*ladocont2*math.cos(math.radians(ang_vect[indice_haz])+ang_giro)))
        # calcula en angulo que forma el valor calculado con dist_rect
        ang_calc = math.acos((-math.pow(ladocont1,2)+math.pow(ladocont2,2)+math.pow(dist_calc,2))/(2*ladocont2*dist_calc))
        ang_haz = 180-math.degrees(ang_calc)       # el complementario de angulo calculado en el paso anterior, es el angulo del haz que se modifica
        ang_vect_ar = np.asarray(ang_vect)    #para poder operar con una lista
        haz = (np.abs(ang_vect_ar-ang_haz)).argmin()     #se busca cual es ese haz
        return dist_calc, haz

    def get_dist_angulo_der(self, ladocont1, ladocont2, indice_haz, ang_giro, ang_vect):
        #formula para calcular el lado de un triangulo no rectangulo, sabiendo dos lados y un angulo
        dist_calc = np.sqrt(math.pow(ladocont1,2)+math.pow(ladocont2,2)-(2*ladocont1*ladocont2*math.cos(math.radians(360-ang_vect[indice_haz])+ang_giro)))
        # calcula en angulo que forma el valor calculado con dist_rect
        ang_calc = math.acos((-math.pow(ladocont1,2)+math.pow(ladocont2,2)+math.pow(dist_calc,2))/(2*ladocont2*dist_calc))
        ang_haz = 180-math.degrees(ang_calc)       # el complementario de angulo calculado en el paso anterior, es el angulo del haz que se modifica
        ang_haz = 360 - ang_haz      #al estar en el lado derecho
        ang_vect_ar = np.asarray(ang_vect)    #para poder operar con una lista
        haz = (np.abs(ang_vect_ar-ang_haz)).argmin()     #se busca cual es ese haz
        return dist_calc, haz


    def act_2(self, scan_data, scan_data_past, ang_vect, n_izq, n_der, val_cam, action_done_past, action_done):
        ang_giro = 0

        if action_done_past == action_done:   #primero se modifican los valores traseros cuando hay mas de una accion 2
            for j in range(4):
                scan_data[10+j] = scan_data_past[10+j] + (self.dist_rec/math.cos(math.radians(abs(180-ang_vect[10+j]))))
        else:
            pass

        n_izq_temp = n_izq[:val_cam+1]
        n_izq = n_izq[1:]     # la posicion 0 no se tiene en cuenta cuando se va en recto
        for i in n_izq:
            dist, haz = self.get_dist_angulo_izq(scan_data_past[i], self.dist_rec, i, ang_giro, ang_vect)
            if haz <= val_cam or haz >= 12:
                pass
            else:
                scan_data[haz] = dist
                if haz not in n_izq_temp:
                    n_izq_temp.append(haz)

        n_der_temp = n_der[:val_cam+1]
        n_der = n_der[1:]      # la posicion 23 no se tiene en cuenta cuando se va en recto
        for i in n_der:
            dist, haz = self.get_dist_angulo_der(scan_data_past[i], self.dist_rec, i, ang_giro, ang_vect)
            if haz >= 23-val_cam or haz <= 12:
                pass
            else:
                scan_data[haz] = dist
                if haz not in n_der_temp:
                    n_der_temp.append(haz)


        return scan_data, n_izq_temp, n_der_temp



    def act_0_1_izq(self, scan_data, scan_data_past, ang_vect, n_izq, n_der, val_cam):
        n_izq_temp = n_izq[:val_cam+1]   #los haces que da la camara originalmente en el lado izquierdo
        n_der_temp = n_der[:val_cam+1]
        for i in n_der:
            dist, haz = self.get_dist_angulo_der(scan_data_past[i], self.dist_lineal, i, self.ang2, ang_vect)
            if haz >= 23-val_cam:
                pass
            elif haz < 23-val_cam and haz >= 12:
                scan_data[haz] = dist
                if haz not in n_der_temp:
                    n_der_temp.append(haz)
            elif haz < 12 and haz > val_cam:
                scan_data[haz] = dist
            else:
                pass

        return scan_data, n_izq_temp, n_der_temp

    def act_3_4_der(self, scan_data, scan_data_past, ang_vect, n_izq, n_der, val_cam):
        n_der_temp = n_der[:val_cam+1]    #los haces que da la camara originalmente en el lado derecho
        n_izq_temp = n_izq[:val_cam+1]
        for i in n_der:
            dist, haz = self.get_dist_angulo_izq(scan_data_past[i], self.dist_lineal, i, self.ang2, ang_vect)
            if haz <= val_cam:
                pass
            elif haz > val_cam and haz < 12:
                scan_data[haz] = dist
                if haz not in n_izq_temp:
                    n_izq_temp.append(haz)
            elif haz >= 12 and haz < 23-val_cam:
                scan_data[haz] = dist
            else:
                pass

        return scan_data, n_izq_temp, n_der_temp

    def act_5(self, scan_data, scan_data_past, n_izq, n_der, val_cam):
        n_izq_temp = n_izq[:val_cam+1]
        n_der_temp = n_der[:val_cam+1]
        for j in range(4):
            scan_data[10+j] = scan_data_past[10+j] - (self.dist_rec/math.cos(math.radians(abs(180-ang_vect[10+j]))))

        return scan_data, n_izq_temp, n_der_temp


# n_izq
# n_izq_90 = [0, 1, 2, 3]    #igual el 0 no poner, porque es imposible que sea otra posicion en la siguiente accion
# n_izq_120 = [0, 1, 2, 3, 4]
# n_izq_150 = [0, 1, 2, 3, 4, 5]
# n_izq_180 = [0, 1, 2, 3, 4, 5, 6]

# n_der
# n_der_90 = [23, 22,21,20]    #igual el 23 no poner, porque es imposible que sea otra posiconn en la siguiente accion
# n_der_120 = [23, 22,21,20,19]
# n_der_150 = [23, 22,21,20,19,18]
# n_der_180 = [23, 22,21,20,19,18,17]

# ang_vect = [alguno de los 4 de abajo]
# ang_90 = [0, 12.86, 25.72, 38.58, 55.22, 71.86, 88.5, 105.14, 121.78, 138.42, 155.06, 171.7, 188.34, 204.98, 221.62, 238.26, 254.9, 271.54, 288.18, 304.82, 321.46, 334.32, 347.18, 360]
# ang_120 = [0, 13.35, 26.7, 40.05, 53.4, 70.28, 87.16, 104.04, 120.92, 137.8, 154.68, 171.56, 188.44, 205.32, 222.2, 239.08, 255.96, 272.84, 289.72, 306.6, 319.95, 333.3, 346.65, 360]
# ang_150 = [0, 13.64, 27.28, 40.92, 54.56, 68.2, 85.4, 102.6, 119.8, 137, 154.2, 171.4, 188.6, 205.8, 223, 240.2, 257.4, 274.6, 291.8, 305.44, 319.08, 332.72, 346.36, 360]
# ang_180 = [0, 13.665, 27.33, 40.995, 54.66, 68.325, 81.99, 99.81, 117.63, 135.45, 153.27, 171.09, 188.91, 206.73, 224.55, 242.37, 260.19, 278.01, 291.675, 305.34, 319.005, 332.67, 346.335, 360]
