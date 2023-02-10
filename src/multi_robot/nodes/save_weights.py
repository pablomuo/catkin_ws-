import rospy
import time
import numpy as np
import math
import random
import json
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String, Float32MultiArray, Float64MultiArray
from math import pi
import numpy
import os
import sys

class Weights_to_save(object):
    ""
    ""
    def __init__(self):
        self.dirPath_2                     = "/home/mcg/catkin_ws/src/multi_robot/save_model/save"
        self.start_time                   = time.time()


    def saving_weights(self,name,step,action,weights,state):
        # if not os.path.exists(self.dirPath_2 +'_saved.txt'):
        #     with open(self.dirPath_2 +'_saved.txt', 'a') as outfile:
        #         outfile.write("network".rjust(8," ")+ "   "+"weights".rjust(8," ")+"   "+"state".rjust(200," ")+"\n")

        m, s = divmod(int(time.time() - self.start_time), 60)
        h, m = divmod(m, 60)
        with open(self.dirPath_2 +'_state.txt', 'a') as outfile:
            outfile.write(str(name)+"   "+str(step)+"   "+str(action)+"   "+str(state)+"\n")
        #
        # with open(self.dirPath_2 +'_weights.txt', 'a') as outfile:
        #     outfile.write(str(name)+"   "+str(step)+"   "+str(weights)+"\n")
