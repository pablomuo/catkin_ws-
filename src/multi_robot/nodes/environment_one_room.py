#!/usr/bin/env python
#################################################################################

#                                            codigo original

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

import rospy
import math
import time
import numpy as np
from std_srvs.srv import Empty
from target import Target
import random
from math import pi
import os
import sys
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
class Behaviour(object):
    def __init__(self,agent_name,id,rank):
        # self.reset_proxy         = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # self.reset_proxy         = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy       = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy         = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.ID=id
        self.target_position     = Target(agent_name,self.ID)
        self.agent_name          = agent_name
        self.number_agents      =rank
        self.goal_x              = 0
        self.goal_y             = 0
        self.goal_distance         = 0
        self.current_distance_1    = 0
        self.last_heading        = 0
        self._maximo_reward      = 300
        self._maximo_reward_angle      =15
        self._distancegoal       = 0.3
        self.initial_steps       = 0
        self.best_time           = 0
        self.cont_step           = 0
        self.initial_steps       = 0
        self.get_goalbox         = False
        self.initGoal            = True
        self.reward_bt           = 0
        self._goal_distance_initial = 0
        self.initial_time        = time.time()
        self.score               = 0
        self.state_msg = ModelState()
    # @property
    # def change_leader(self):
    #     reward=np.array([robot.reward for robot in self.agents])
    #     position=np.argmax(reward)
    #     for robot in self.agents:
    #         robot.status = "follower"
    #     self.agents[position].status="lead"
    #
    #     return position

    def fix_angle(self,angle):
        """
          fix an angle to be between 180 and -180 degree
        """
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi
        return angle

    def get_Distance_goal(self,x,y):
        '''
        Calculate the initial distance to the goal
        '''
        self.goal_distance= (math.hypot(self.goal_x - x, self.goal_y- y))
        self._goal_distance_initial=self.goal_distance
        return self.goal_distance


    def get_current_Distance(self,x,y):
        '''
        Calculate the actual distance to the goal
        '''
        # print("CD: ",math.hypot(self.goal_x - x, self.goal_y- y) )
        return math.hypot(self.goal_x - x, self.goal_y- y)

    def set_reward(self, state, done, action, step):
        '''
        Calculate reward(distance-angle-wall-time)
        scan_data + [dis_near_agent,head_near_agent,heading, current_distance,wall_dist,goal_heading_initial]
        '''
        # print("CD reward: ", state[-3])
        # print("CD reward normalize: ", state[-3]*self._goal_distance_initial)

        # dis_near_agent      = state[-6]
        # head_near_agent     = np.degrees(state[-5])
        heading             = state[-4]
        current_distance    = state[-3]*self._goal_distance_initial
        wall_dist           = state[-2]*3.5
        goal_heading_initial= np.degrees(state[-1])
        # Save the last steps in an array
        # self.turn[0:-1]     = self.turn[1:]
        # self.turn[-1]       = action
        # print("Current distance, ", current_distance)
        last_distance = self.goal_distance

        # if dis_near_agent < 0.50:
        #     reward_a2a = -5
        # else:
        #     reward_a2a = 0



        if wall_dist<0.25:
            wall_reward = -5
        else:
            wall_reward = 0

        if step >1:
            if action ==5 :
                self.reward_current_angle = 0.0
            else:
                    self.reward_current_angle = ((np.exp(-abs(np.degrees(self.last_heading))) - np.exp(-abs(np.degrees(heading))))/(np.exp(-abs(goal_heading_initial-6))-1))*self._maximo_reward_angle
            # print("last hading ", np.degrees(heading),np.degrees(self.last_heading),goal_heading_initial)
            self.last_heading = heading
        else:
            self.reward_current_angle = 0.


        #Reward goal and best time
        if (0<current_distance < self._distancegoal) and (-pi/2< heading <pi/2):
            # Calculate the goal_time
            self.initial_steps = (self._goal_distance_initial+0.7)/0.15
            #Calculate time used
            t_steps = time.time() - self.initial_time
            self.initial_time = time.time()
            #Calculate best_time
            self.best_time = self.initial_steps/t_steps
            if self.best_time > 1:
                self.best_time1 = 1
            else:
                self.best_time1 = -(1-self.best_time)
                #Reward best_time
            self.reward_bt = 100*self.best_time1
                #Reward goal
            # print("win",current_distance)
            self.get_goalbox = True
            self._cont_step = self.cont_step
            self.cont_step = 0

        if action ==5 :
            distance_rate = 0
        else:
            if abs(current_distance-self.goal_distance)>0.8*0.15: # avoid get negative reward for reset
                distance_rate =0
                self.reward_current_angle =0
            elif current_distance <self._distancegoal:
                distance_rate =0
            else:
                distance_rate = ((np.exp(-last_distance) - np.exp(-current_distance))/(np.exp(-(self._goal_distance_initial -self._distancegoal))-1))*self._maximo_reward
        self.goal_distance = current_distance

        reward = distance_rate  + self.reward_current_angle +wall_reward
        # reward = distance_rate  + self.reward_current_angle +wall_reward+reward_a2a
        #Reward collision
        if done:
            reward = -1000
            # self.pub_cmd_vel.publish(Twist())


        if self.get_goalbox:
            reward = 1000 +self.reward_bt
            # self.pub_cmd_vel.publish(Twist())

        # print("action: ", action,"reward: ", reward, "distance_rate ",distance_rate,"self.reward_current_angle ",self.reward_current_angle,"wall_reward: ",wall_reward )
        return reward


    def reset_gazebo(self):
        # idx=random.choice(range(4))
        # idx=random.choice(range(2))
        if self.number_agents ==4:
            idx=0
        elif self.number_agents==5:
            idx=1
        else:
            raise Exception("check number of robot")

        y_orientation=- 3.1416
        if self.ID==0:
            # x=[-1,   1.5]
            # y=[1.5,0.5]
            x=[-0.8, -0.8]
            y=[-0.8, 0]
            # x=[1.3,-1,-1,   1.5]
            # y=[0,-1,1.5,0.5]
            # x=[0,1,0.7, 2 , -4, 0,  -3,-4.5,3]
            # y=[0,2,  3,-1 ,3.5,-3,-3.5,   4,4]
        elif self.ID==1:
            x=[-1,1]
            y=[ -6,-6]
        elif self.ID==2:
            x=[-1,1]
            y=[ -6,-6]
        elif self.ID==3:
            x=[-0.8,-0.8]
            y=[-0.8,0]
        else:
            raise Exception("There is not room with this ID "+str(self.ID))

        # rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.state_msg.model_name=self.agent_name
            self.state_msg.reference_frame = 'world'  # ''ground_plane'
            self.state_msg.pose.position.x = x[idx]
            self.state_msg.pose.position.y = y[idx]
            # self.state_msg.pose.orientation.x = y_orientation

            self.state_msg.pose.orientation.z = pi/4.0
            # self.state_msg.pose.orientation.w = pi*self.number_agents
            resp = set_state(self.state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def reset(self,x,y):
        if self.initGoal:
            self.goal_x, self.goal_y = self.target_position.getPosition()
            self.initGoal = False
        self.goal_distance = self.get_Distance_goal(x,y)

        self.done =False

        # print("Get initial position of the agent")
