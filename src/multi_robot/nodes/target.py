#!/usr/bin/env python
#################################################################################
#Copyright 2022 Elizabeth
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#distributed under the License is distributed on an "AS IS" BASIS,
#See the License for the specific language governing permissions and
#limitations under the License.
#################################################################################


import rospy
import numpy as np
import time
import os
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class Target(object):
    def __init__(self,agent_name,ID):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('multi_robot/nodes',
                                                'multi_robot/worlds/goal_box_'+str(agent_name)+'/model_'+str(agent_name)+'.sdf')
        # self.f=str(agent_name)
        self.agent_name= open(self.modelPath, 'r')
        self.model = self.agent_name.read()
        self.target_position = Pose()
        # FOR BOX
        self.ID=ID
        self.ini=random.choice(range(4))
        # self.init_goal_x = [4,-2.5,-2, 4, 0,-3,0][self.ini]
        # self.init_goal_y = [4, 0.5,-3,-1,-1, 0,0][self.ini]
        if self.ID==0:
            # 2 rooms small
            self.init_goal_x =0
            self.init_goal_y=3

            # 4 rooms original
            # self.init_goal_x =-0.5
            # self.init_goal_y=0.5

        else:
            # 2 rooms small
            self.init_goal_x =1.5
            self.init_goal_y=-2.6

            # 4 rooms original
            # self.init_goal_x =0.5
            # self.init_goal_y=-6


        # self.init_goal_x =[1.3,-1,-1,   1.5][self.ini]
        # self.init_goal_y=[ 0,  -1, 1.5, 0.4][self.ini]
        self.target_position.position.x = self.init_goal_x
        self.target_position.position.y = self.init_goal_y
        print("test agentname")
        self.modelName = 'goal_'+str(agent_name)
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.check_model = False
        self.index = 0


    def respawnModel(self):
        while True:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.target_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f ", self.target_position.position.x,
                              self.target_position.position.y )
                break
    def deleteModel(self):
        while True:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break

    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()
        while position_check:
            if self.ID==0:
                # 2 rooms small
                goal_x_list = [2.2, 2, 0.9, 0, -2.2, -1.4, -0.6, 2.1, -1.2, 0]
                goal_y_list = [4.5, 0.6, 0.9, 4.7, 0.7, 5, 1.5, 2.2, 3.2, 3]

                # 4 rooms original
                # goal_x_list = [-1, 1.8, 0.6, 1.9,  0.7, 0.2, -1.3, -1, -1.9,  0.5, 0.5, 0, -0.1, -2,  -0.5]
                # goal_y_list = [-1.7,-1.8, 0,  -0.5, -1.9, 1.5, -0.9,  1,  1.1, -1.5, 1.8, -1, 1.6, -0.8, 0.5]

            if self.ID==1:
                # 2 rooms small
                goal_x_list = [1.85, 2.4, 2, 0.41, -2.2, 1.5, -2, -1.9, 0.3, -0.9, 0.1]
                goal_y_list = [-1.3, -2.8, -4.7, -4.9, -1.1, -2.6, -4.5, -2.6, -1, -1.8, -3.7]

                # 4 rooms original
                # goal_x_list =[0.5, 1, 1.8, -0.5, -2,-2 ,0,2, -1.5,1.9]
                # goal_y_list =[-6,-3, -3,-4.1,-4.1, -3,-6,-5,-6.5 ,-0.5]

            # if self.ID==2:
            #     goal_x_list =[1,4,4,3]
            #     goal_y_list =[-3,-4,-6,-3.5]
            #
            # if self.ID==3:
            #     goal_x_list =[-1,4,5,5]
            #     goal_y_list =[1,0,1.5,-1.5]

            # goal for corridor
            # goal_x_list = [0.5,1,    2  ,3 , 4  ,7 , 9  ,  12,   4.5, 7.5,-1,-4, -1,-5,  -7.5,-9,-10,-11,9.5]
            # goal_y_list = [0.5,1,   0.5, 1,-0.8, 0.8,  -1.5,0,  -1.5,-1.5,-1,-0.5,1,-1.2,1,   -1, 1,  0, 1.5]

            self.index = np.random.randint(0, len(goal_y_list), 1)[0]


            # self.index = random.randrange(0, len(goal_y_list))
            if self.last_index == self.index:
                position_check = True

            else:
                self.last_index = self.index
                position_check = False

            self.target_position.position.x = goal_x_list[self.index]
            self.target_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.target_position.position.x
        self.last_goal_y = self.target_position.position.y

        return self.target_position.position.x, self.target_position.position.y

    @property
    def position(self):
        return (self.target_position.position.x ,self.target_position.position.y)
