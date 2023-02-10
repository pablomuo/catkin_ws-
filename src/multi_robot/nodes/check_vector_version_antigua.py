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
        self.cont_ant        = 0
        self.dist_rec        = 0
        self.dist_lineal     = 0
        self.ang2            = 0


    def get_point_position(self, lado1, lado2, ang, ang2):
        #formula para calcular el lado de un triangulo no rectangulo, sabiendo dos lados y un angulo
        point_position = np.sqrt(math.pow(lado1,2)+math.pow(lado2,2)-(2*lado1*lado2*math.cos(math.radians(((ang)*15)+7.5)+ang2)))
        return point_position

    def reset_cont(self, action_done, action_done_past, cont,  num_rep):
        if action_done_past == 2 or abs(action_done - action_done_past) <= 1:   # action_done - action_done_past) <= 1 contempla cuando (action_done == action_done_past) y hace acciones hacia el mismo lado
           cont+=1
           if cont > num_rep:
            cont = num_rep
        else:
           cont = 1
        return cont


    def check_laser(self, value):
        val_a_cambiar = int(math.floor(value/2))+1          #hay que establecer una variable que indique cuando valores arroja la camara, si 12 o menos
        return val_a_cambiar


    def check_dist_angle(self, diff_time, vel_lineal, vel_ang, cont):
        self.dist_rec = diff_time*abs(vel_lineal)           #distancia que recorrge en recto tiempo x velocidad lineal
        if vel_ang != 0:
            radio = abs(vel_lineal/vel_ang)
            dist_ang = abs(vel_ang*diff_time)                    # desplazamiento angular en radianes
            ang1 = (math.radians(180)-dist_ang)/2               #el triangulo isoceles
            self.ang2 = (math.radians(90)-ang1)                 # buscamos el angulo complementario del ang1
            self.dist_lineal = self.get_point_position(radio, radio, dist_ang, 0)
        else:
            self.ang2 = 0
            pass

        self.cont_ant = cont



    def act_diff_eq_2(self, scan_data, scan_data_past, val_a_cambiar):          #action_done == action_done_past == 2
        scan_data[val_a_cambiar:(val_a_cambiar+self.cont_ant)] = scan_data_past[val_a_cambiar:(val_a_cambiar+self.cont_ant)]
        scan_data[23-val_a_cambiar:(23-val_a_cambiar-self.cont_ant)] = scan_data_past[23-val_a_cambiar:(23-val_a_cambiar-self.cont_ant)]
        scan_data[10:14] = scan_data_past[10:14]

        return scan_data



    def act_diff_inf_2(self, scan_data, scan_data_past, action_done, action_done_past, cont, val_a_cambiar):
        if abs(action_done - action_done_past) == 1:    #cuando esta en 0 y viene de 1, y viceversa
            scan_data[23-val_a_cambiar:(23-val_a_cambiar-cont)] = scan_data_past[23-val_a_cambiar:(23-val_a_cambiar-cont)]

        elif abs(action_done - action_done_past) > 1:    #cuando viene de la accion 3, 4
            # self.reset_cont(14-val_a_cambiar)
            if self.cont_ant > 14-val_a_cambiar:
                range_max = 14-val_a_cambiar
                scan_data[14:val_a_cambiar+self.cont_ant] = scan_data_past[14:val_a_cambiar+self.cont_ant]
            else:
                range_max = cont

            for i in range(cont):
                scan_data[val_a_cambiar+i] = scan_data_past[val_a_cambiar+i-1]

        elif action_done == 0 and action_done_past == action_done:
            for i in range(cont):
                scan_data[23-val_a_cambiar-i] = scan_data_past[23-val_a_cambiar-i+1]

        elif action_done == 1 and action_done_past == action_done:
            for i in range(cont):
                scan_data[23-val_a_cambiar-i] = self.get_point_position(scan_data_past[23-val_a_cambiar-i+1], self.dist_lineal, (val_a_cambiar+i-1), self.ang2)

        return scan_data



    def act_eq_2(self, scan_data, scan_data_past, action_done, action_done_past, cont, val_a_cambiar):
        if action_done_past == action_done:
            for i in range(cont):
                scan_data[val_a_cambiar+i]  = self.get_point_position(scan_data_past[val_a_cambiar+i-1], self.dist_rec, (val_a_cambiar+i-1), self.ang2)
                scan_data[23-val_a_cambiar-i] = self.get_point_position(scan_data_past[23-val_a_cambiar-i+1], self.dist_rec, (val_a_cambiar+i-1), self.ang2)

            for j in range(4):
                if j == 0 or j == 3:
                    scan_data[10+j] = scan_data_past[10+j] + (self.dist_rec/math.cos(math.radians(7.5+15)))
                else:
                    scan_data[10+j] = scan_data_past[10+j] + (self.dist_rec/math.cos(math.radians(7.5)))

        elif action_done_past < 2:  #cuando viene de la accion 0, 1
            scan_data[23-val_a_cambiar:23-val_a_cambiar-self.cont_ant] = scan_data_past[23-val_a_cambiar:23-val_a_cambiar-self.cont_ant]

        elif action_done_past > 2:  #cuando viene de la accion 3,4
            scan_data[val_a_cambiar:val_a_cambiar+self.cont_ant] = scan_data_past[val_a_cambiar:val_a_cambiar+self.cont_ant]

        return scan_data


    def act_5(self, scan_data, scan_data_past):
            for j in range(4):
                if j == 0 or j == 3:
                    scan_data[10+j] = scan_data_past[10+j] - (self.dist_rec/math.cos(math.radians(7.5+15)))
                else:
                    scan_data[10+j] = scan_data_past[10+j] - (self.dist_rec/math.cos(math.radians(7.5)))
            return scan_data



    def act_diff_sup_2(self, scan_data, scan_data_past, action_done, action_done_past, cont, val_a_cambiar):
        if abs(action_done - action_done_past) == 1:    #cuando esta en 3 y viene de 4 y viceversa
            scan_data[val_a_cambiar:(val_a_cambiar+cont)] = scan_data_past[val_a_cambiar:(val_a_cambiar+cont)]

        elif abs(action_done - action_done_past) > 1:    #cuando viene de la accion 0, 1
            # self.reset_cont(14-val_a_cambiar)
            if self.cont_ant > 14-val_a_cambiar:
                range_max = 14-val_a_cambiar
                scan_data[10-(self.cont_ant-(14-val_a_cambiar)):10] = scan_data_past[10-(self.cont_ant-(14-val_a_cambiar)):10]
            else:
                range_max = cont
            for i in range(range_max):
                scan_data[23-val_a_cambiar-i] = scan_data_past[23-val_a_cambiar-i+1]


        elif action_done == 3 and action_done_past == action_done:
            for i in range(cont):
                scan_data[val_a_cambiar+i]  = self.get_point_position(scan_data_past[val_a_cambiar+i-1], self.dist_lineal, (val_a_cambiar+i-1), self.ang2)

        elif action_done == 4 and action_done_past == action_done:
            for i in range(cont):
                scan_data[val_a_cambiar+i]  = scan_data_past[val_a_cambiar+i-1]

        return scan_data
