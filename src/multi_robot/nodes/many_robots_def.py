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

import numpy as np
import math
from math import pi
import os
import sys
import random

"""
Creates a file with the specified number of robots and targets
"""
import matplotlib
color=[]
for cname, hex in matplotlib.colors.cnames.items():
    color.append(cname)


"""
Creates a file with the specified number of robots
"""
arguments = sys.argv
# agents = int(arguments[1][1])      #numero de robot
agents =int(arguments[1])            #numero de robots
print("number of robots: ", agents)

robot1 = int(arguments[3])
robot2 = int(arguments[4])
room_chosen = int(arguments[5])

# robot_list = [robot1, robot2]
# se establece si hay un robot, en que room esta
if agents == 1:
    robot_list = [robot1]
    if room_chosen == 0:
        rax = [0]
        ray = [1]
        # 4 rooms original
        # rax = [-0.8]
        # ray = [-0.8]

    else:
        rax = [1.4]
        ray = [-1]
elif agents == 2:
    robot_list = [robot1, robot2]
    rax = [0,1.4]
    ray = [1,-1]
else:
    raise Exception("You need more cores than rooms!")

# 0 for 1 camera 90
# 1 for 1 camera 120
# 2 for 1 camera 150
# 3 for 1 camera 180
# 4 for 2 cameras
# 5 for laser
for i in range(len(robot_list)):
    if robot_list[i] == 0:
        robot_list[i] = '_1cam90'
    elif robot_list[i] == 1:
        robot_list[i] = '_1cam120'
    elif robot_list[i] == 2:
        robot_list[i] = '_1cam150'
    elif robot_list[i] == 3:
        robot_list[i] = '_1cam180'
    elif robot_list[i] == 4:
        robot_list[i] = '_2cam'
    else:
        robot_list[i] = ''
    print("type of robot of agent"+str(i+1)+":", robot_list[i])


path= "/home/mcg/catkin_ws/src/multi_robot/launch/multi_agent.launch"
path1= "/home/mcg/catkin_ws/src/multi_robot/worlds/goal_box/model"

out_str= "<launch>\n"+\
       '<param name="robot_description"\n'+\
       'command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_burger_2cam.urdf.xacro" />"\n'

x_initial=int(arguments[2])
xp_initial= 0.5
xy_initial= -1.3
Y_initial= 3.1416
# rax = [1,1.4]
# ray = [1,-1]

for i in range(1,agents+1,1):
    out_str += '<group ns="'+"agent"+str(i)+'">\n'+\
    '<param name="tf_prefix" value="'+"agent"+str(i)+'_tf" />\n'+\
    '<include file="$(find multi_robot)/launch/one_agent'+robot_list[i-1]+'.launch" >\n' + \
    '<arg name="init_pose" value="-x '+str(rax[i-1])+' -y '+str(ray[i-1])+' -z 0 -Y '+str(Y_initial+i*0.5)+'" />\n'+\
    '<arg name="agent_name"  value="'+"agent"+str(i)+'" />\n'+\
    '</include>\n'+\
    '</group>\n'

out_str+='</launch>'
with open(path,"w") as out :
  out.write(out_str)


"""
Creates a file with the specified number of targets, each of them with a different color
"""

for i in range(1,agents+1,1):  #si hay dos robots; robot1 y robot2  (siempre el primero es robot1)intrai
    print("hola pablo", i)
    # color=random.choice(["Black","Red","Green","Yellow","Purple","Turquoise","Blue"])
    color=["Black","Yellow","Purple","Green","Turquoise","Blue","Red"]
    if i >= len(color):
        col = random.choice(["Black","Red","Green","Yellow","Purple","Turquoise","Blue"])
    else:
        col = color [i]
    #col=random.choice(["FlatBlack","Black","Red","Green","Yellow","Purple","Turquoise","RedEmissive","GreenEmissive","PurpleEmissive","BlueLaser","BlueEmissive","JointAnchor","Blue","Skull","ExclamationPoint","QuestionMark","SmileyHappy","SmileySad","SmileyDead","SmileyPlain","WoodFloor","CeilingTiled","PaintedWall","PioneerBody","Pioneer2Body","CloudySky","RustySteel","Chrome","BumpyMetal","GrayGrid","Rocky","GrassFloor","Rockwall","RustyBarrel","WoodPallet","Fish","LightWood","WoodTile","Brick","DepthMap","PCBGreen","Turret","EpuckBody","EpuckRing","EpuckPlate","EpuckLogo","EpuckMagenta","EpuckGold"])
    os.system("mkdir /home/mcg/catkin_ws/src/multi_robot/worlds/goal_box_"+"agent"+str(i))
    out_str= "<?xml version='1.0'?>\n"+\
           "<sdf version='1.6'>\n"+\
           "  <model name='goal_box_"+"agent"+str(i)+"'>\n"+\
           "    <pose frame=''>0 0 0 0 -0 -1.5708</pose>\n"+\
           "    <link name='goal_box_"+"agent"+str(i)+"'>\n"+\
           "      <visual name='goal_box_"+"agent"+str(i)+"'>\n"+\
           "        <pose frame=''>0 0 0.0005 0 -0 0</pose>\n"+\
           "        <geometry>\n"+\
           "          <box>\n"+\
           "            <size>0.5 0.5 0.001</size>\n"+\
           "          </box>\n"+\
           "        </geometry>\n"+\
           "        <material>\n"+\
           "          <script>\n"+\
           "            <uri>file://media/materials/scripts/gazebo.material</uri>\n"+\
           "            <name>Gazebo/"+col+"</name>\n"+\
           "          </script>\n"+\
           "          <ambient>1 0 0 1</ambient>\n"+\
           "        </material>\n"+\
           "      </visual>\n"+\
           "      <pose frame=''>0 0 0 0 -0 0</pose>\n"+\
           "    </link>\n"+\
           "    <static>1</static>\n"+\
           "  </model>\n"
    out_str+='</sdf>'
    with open("/home/mcg/catkin_ws/src/multi_robot/worlds/goal_box_"+str("agent"+str(i))+"/model_"+str("agent"+str(i))+".sdf","w") as out :
      out.write(out_str)

    out_str_1= "<?xml version='1.0'?>\n"+\
           "<model>\n"+\
           "  <name>goal_box_"+"agent"+str(i)+"</name>\n"+\
           "  <version>1.0</version>\n"+\
           "  <sdf version='1.6'>model_"+"agent"+str(i)+".sdf</sdf>\n"+\
           "  <author>\n"+\
           "      <name></name>\n"+\
           "      <email></email>\n"+\
           "  <author>\n"+\
           "  <description></description>\n"
    out_str_1+='</model>'
    with open("/home/mcg/catkin_ws/src/multi_robot/worlds/goal_box_"+str("agent"+str(i))+"/model_"+str("agent"+str(i))+".config","w") as out_1 :
      out_1.write(out_str_1)
