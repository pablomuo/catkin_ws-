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

from mpi4py import MPI
import numpy as np
import rospy
import random
from std_msgs.msg import Float32MultiArray
import tensorflow as tf
from agents import Agent
from environment import Behaviour
from collaboration import Collaboration
from reinforcenment import ReinforcementNetwork
from pathfinding import pathfinding
from log_class import Logfile
import os
import sys
import time
import os
import keras
import tensorflow as tf
gpu_options = tf.GPUOptions(allow_growth=True)
sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))
keras.backend.tensorflow_backend.set_session(sess)
from numba import cuda
tf.logging.set_verbosity(tf.logging.ERROR)

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '0'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '1'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
comm = MPI.COMM_WORLD
size = comm.Get_size()
rank = comm.Get_rank()
name = MPI.Get_processor_name()
state_size         = 28
# state_size         = 30
action_size        = 6
episodes           = 19000
episode_step       = 6000
e                  = 0
global_step        = 0
number_episode     = 0
number_rooms       = 1
number_robot       = int(size)-(number_rooms*2) #Number of robots

if size<=number_rooms:
    raise Exception("You need more cores than rooms!")

if rank==0:

    #Delete old folders of goal_box
    os.system('rm -r /home/mcg/catkin_ws/src/multi_robot/worlds/goal_box_'+"*")
    os.system('rm -r /home/mcg/catkin_ws/src/multi_robot/save_model/en'+"*")
    #Creates a file with the specified number of robots and targets
    os.system('python many_robots.py'+" "+str([number_robot,number_rooms]))
    os.system("roslaunch multi_robot turtlebot3_multi.launch &")

comm.Barrier()
log= Logfile(rank)
# Process to be followed for each agent
if rank>(number_rooms*2)-1:
    robot_ID = rank-1
    cloud_ID = 0
    node = 'network'+str(rank)
    rospy.init_node(node)
    # Instances
    agents  = Agent("agent"+str(robot_ID),action_size,state_size,number_episode,robot_ID,number_rooms,number_robot,log,cloud_ID)
    agents.call_sub("agent"+str(robot_ID),action_size,state_size,number_episode,robot_ID)

    # YA NO HAY QUE ENVIAR LOS PESOS DESDE EL ROBOT A LA NUBE Y EN LA NUBE 1 NO HAY QUE REGISTRARSE
    # The agent sends its ID to the cloud in order to register.
    # log.message("Register new user : "+str(robot_ID))
    # comm.send(rank, dest=agents.ID, tag=0*(size+1)+agents.ID*10000)
    # weight_q_to=agents.learning.q_model.get_weights()
    # comm.send(weight_q_to, dest=agents.ID, tag=4*(size+1)+agents.ID*10000+rank+100)
    # weight_target_to=agents.learning.target_model.get_weights()
    # comm.send(weight_target_to, dest=agents.ID, tag=5*(size+1)+agents.ID*10000+rank+100)
    # log.message("Finish to send networks from the new user to the cloud : "+str(rank) +" "+str(agents.ID))

    # The agent sends its ID to the cloud in order to register.
    log.message("Register new user in the CLOUD 2: "+str(rank))
    comm.send(rank, dest=1, tag=51)


    #Initialization of variables to check if any agent is sending memory data
    init=True
    init_G=True

    Room_member_ID=range(number_rooms,number_robot+number_rooms,1)
    # Resetting the position of each agent within the environment
    # agents.environment.reset_gazebo()

    while not rospy.is_shutdown():
        """ Your main process which runs in thread for each chunk"""
        e=agents.learning.load_epidose +1
        while (e <= episodes):
            log.message("New episode, robot: " +str(robot_ID))
            e+=1
            # Reseting of variables
            agents.reset()
            agents.environment.reset_gazebo()
            time.sleep(0.5)
            # Resetting the position of the target and obtaining the distance to it
            agents.environment.reset(agents.robot_position_x, agents.robot_position_y)
            # time.sleep(0.5)
            log.message("1. Reset position robots : " +str(rank))
            # sys.stdout.flush()
            # Gets the initial state  (scan_data + [heading, current_distance,
            # wall_dist,goal_heading_initial])
            # agents.get_initial_status
            agents.last_state= agents.state
            # print("DONE initial ", agents.done)
            log.message("heading initial before for robots: " +str(rank))

            agents.goal_heading_initial = agents.heading

            for step in range(episode_step):
                agents.dist_a2a                     = []
                agents.hea_a2a                      = []
                agents.step = step

###################################
                #  HABRA QUE VER ESTO QUE ES CUANDO HAYA MAS DE UN ROBOT!!!!!!!!! SALE ERROR DE QUE "2 NO ESTA EN ELA LISTA"!!!!!!!!!!!!!!!!!
                #Sending to robots
                # log.message("send to"+" "+str(Room_member_ID[Room_member_ID.index(rank)+1:]))
                # for p in Room_member_ID[:]:
                #     if p==rank:
                #         continue
                #     dist_yaw_a2a = [agents.robot_position_x,agents.robot_position_y,rank]
                #     log.message("before to send position a2a "+str(p)+" "+str(dist_yaw_a2a))
                #     comm.issend(dist_yaw_a2a, dest=p, tag=999999)
                #     log.message("after to send position a2a to: "+str(p) +" from: "+"rank")
                # log.message("ROM members : "+str(Room_member_ID))
                #
                # while comm.Iprobe(source=MPI.ANY_SOURCE, tag=999999): #ask if recive from the all sources
                #     log.message("before to receive data a2a from  "+ str(rank))
                #     dist_yaw_a2a=comm.recv(source=MPI.ANY_SOURCE,tag=999999)
                #     log.message("after to receive data a2a from  "+ str(rank))
                #     visiting_robot_x,visiting_robot_y,rank1= dist_yaw_a2a
                #     agents.get_heading_a2a(visiting_robot_x,visiting_robot_y,rank1)
                    # log.message("near 2a2: "+str(agents.dist_a2a)+str(agents.hea_a2a))

                # If the agent has a Pa (level of knowledge) > normal proces
                # (ability to execute backward action). Each collision ends
                # the episode and the agent is restarted in a random position
                # within the room where it crashedevolve

                # agents.get_action_value()                     lo hace cloud 2 ahora
                state_send, _ = agents.last_state
                comm.send(state_send, dest = 1, tag = 125)
                log.message("Agent send state to cloud 2")
                if agents.learning.Pa >agents.learning.normal_process:
                    if agents.process =="collision":
                        break
                    else:
                        pass
###################################
                # get accion of neuronal network or call evolve rule (path planning Algorithm)
                log.message("Before receiving action from CLOUD 2")
                action_evolve = comm.recv(source=1, tag=100)
                agents.action = action_evolve[0]
                agents.evolve_rule = action_evolve[1]

                log.message(" 2. Get from CLOUD 2 action of evolve rule with tag 100, action: "+str(agents.action)+" "+str(agents.evolve_rule)+" PA: "+str(agents.learning.Pa)+" "+str(rank))
                # According to the type of knowledge or processes. The agent will execute:
                if agents.evolve_rule or agents.process =="collision":
                    agents.perform_action(agents.evolve()) # Get the action and Execute the action
                    agents.next_values() # Return next state, reward, done
                else:
                    agents.perform_action(agents.action)# Execute the action
                    agents.next_values() # Return next state, reward, done
                log.message(" 3. Execute action "+str(agents.action)+" "+str(rank))
                # sys.stdout.flush()

                # Save all data for plot
                agents.save_data(robot_ID,step,e)
                if agents.done==True or agents.environment.get_goalbox == True:
                    agents.save_data_win(robot_ID,step,e)
                    log.message(" 4. save data"+" "+str(rank))
                # sys.stdout.flush()

                # Agent append data into memory D and Eps
                # data = state_initial,  action, reward, next_state, done
                agents.append_memory()
                log.message(" 5. Append memory"+" "+str(rank))


                # The agent starts to send its memory to the corresponding cloud(ID)
                # to the room it is browsing
                # At the begining all robots have ID=0, which mean they are inside room 0

                if len(agents.learning.memory_D)>=20:
                    log.message(" 6.Start to send memory D of lenght "+str(len(agents.learning.memory_D))+" "+str(rank))
                    if init:
                        init=False
                        data=np.array(agents.learning.memory_D)
                        req=comm.issend(data, dest=agents.ID, tag=11)
                        agents.learning.memory_D.clear()
                        log.message(" 6.1 after init Sending memory D from: "+" "+str(rank)+" to:  "+str(agents.ID))
                    else:
                        # if MPI.Request.Test(req):
                        data=np.array(agents.learning.memory_D)
                        MPI.Request.Wait(req)
                        req=comm.issend(data, dest=agents.ID, tag=11)
                        agents.learning.memory_D.clear()
                        log.message(" 6.1 after Sending memory D from: "+" "+str(rank)+" to:  "+str(agents.ID))


                if len(agents.learning.memory_GT)>=2:
                    log.message("8. Before Sending memory GT of lenght "+str(len(agents.learning.memory_GT))+" "+str(rank))
                    if init_G:
                        init_G=False
                        data_G=np.array(agents.learning.memory_GT)
                        req_g=comm.issend(data_G, dest=agents.ID, tag=21)
                        agents.learning.memory_GT.clear()
                        log.message("8. after init Sending memory GT from: "+" "+str(rank)+" to:  "+str(agents.ID))

                    else:
                        # if MPI.Request.Test(req_g):
                        data_G=np.array(agents.learning.memory_GT)
                        MPI.Request.Wait(req_g)
                        req_g=comm.issend(data_G, dest=agents.ID, tag=21)
                        agents.learning.memory_GT.clear()
                        log.message("8. after Sending memory GT from: "+" "+str(rank)+" to:  "+str(agents.ID))



                # if len(agents.learning.memory_D)>=20:
                #     log.message(" 6.Start to send memory D of lenght "+str(len(agents.learning.memory_D))+" "+str(rank))
                #     if init:
                #         init=False
                #         data=np.array(agents.learning.memory_D)
                #         comm.send(data, dest=agents.ID, tag=11)
                #         agents.learning.memory_D.clear()
                #         log.message(" 6.1 after init Sending memory D from: "+" "+str(rank)+" to:  "+str(agents.ID))
                #     else:
                #         data=np.array(agents.learning.memory_D)
                #         comm.send(data, dest=agents.ID, tag=11)
                #         agents.learning.memory_D.clear()
                #         log.message(" 6.1 after Sending memory D from: "+" "+str(rank)+" to:  "+str(agents.ID))
                #
                #
                # if len(agents.learning.memory_GT)>=2:
                #     log.message("8. Before Sending memory GT of lenght "+str(len(agents.learning.memory_GT))+" "+str(rank))
                #     if init_G:
                #         init_G=False
                #         data_G=np.array(agents.learning.memory_GT)
                #         comm.send(data_G, dest=agents.ID, tag=21)
                #         agents.learning.memory_GT.clear()
                #         log.message("8. after init Sending memory GT from: "+" "+str(rank)+" to:  "+str(agents.ID))
                #
                #     else:
                #         data_G=np.array(agents.learning.memory_GT)
                #         comm.send(data_G, dest=agents.ID, tag=21)
                #         agents.learning.memory_GT.clear()
                #         log.message("8. after Sending memory GT from: "+" "+str(rank)+" to:  "+str(agents.ID))


                # ESTO AQUI NO, EN CLOUD 2
                # if the agent has data to receive
                # if comm.Iprobe(source=agents.ID,tag=1*(size+1)+agents.ID*10000+rank+100):
                #     log.message("12.0 before receive network tag:"+str(1*(size+1)+agents.ID*10000+rank+100))
                #     weight_q=comm.recv(source=agents.ID,tag=1*(size+1)+agents.ID*10000+rank+100)
                #     agents.learning.q_model.set_weights(weight_q)
                #     weight_target=comm.recv(source=agents.ID,tag=2*(size+1)+agents.ID*10000+rank+100)
                #     agents.learning.target_model.set_weights(weight_target)
                #     log.message("12.0 after receive q network from "+str(rank)+" to:  "+str(agents.ID)+"TAG: "+str(2*(size+1)+agents.ID*10000+rank+100))
                #     Room_member_ID=comm.recv(source=agents.ID,tag=3*(size+1)+agents.ID*10000+rank+100)
                #     log.message("ROM members after unsubscribe : "+str(Room_member_ID)+"TAG: "+str(3*(size+1)+agents.ID*10000+rank+100))
                #
                #     log.message("12.0 before receive target network from "+str(rank)+" to:  "+str(agents.ID))
                #     agents.Pa=comm.recv(source=agents.ID,tag=6*(size+1)+agents.ID*10000+rank+100)
                #     log.message("12.0 before receive Pa from "+str(rank)+" to:  "+str(agents.ID)+"TAG: "+str(6*(size+1)+agents.ID*10000+rank+100))

                # Check if the agent changed rooms
                agents.check_room()
                # log.message("ROM members : "+str(Room_member_ID))
                if agents.old_ID != agents.ID:
                    # if comm.Iprobe(source=agents.ID,tag=1*(size+1)+agents.ID*10000+rank+100):
                    #     log.message("12.0 before receive network because it is new in the room ")
                    #     weight_q=comm.recv(source=agents.ID,tag=1*(size+1)+agents.ID*10000+rank+100)
                    #     log.message("12.0 before receive q network from "+str(rank)+" to:  "+str(agents.ID)+"tag: "+str(1*(size+1)+agents.ID*10000+rank+100))
                    #     weight_target=comm.recv(source=agents.ID,tag=2*(size+1)+agents.ID*10000+rank+100)
                    #     log.message("12.0 before receive target network from "+str(rank)+" to:  "+str(agents.ID)+"tag: "+str(2*(size+1)+agents.ID*10000+rank+100))
                    #     Room_member_ID=comm.recv(source=agents.ID,tag=3*(size+1)+agents.ID*10000+rank+100)
                    #     log.message("12.0 before receive room member from "+str(rank)+" to:  "+str(agents.ID)+"tag: "+str(3*(size+1)+agents.ID*10000+rank+100))
                    #     agents.Pa=comm.recv(source=agents.ID,tag=6*(size+1)+agents.ID*10000+rank+100)
                    #     log.message("12.0 before receive Pa from "+str(rank)+" to:  "+str(agents.ID)+"tag: "+str(6*(size+1)+agents.ID*10000+rank+100))
                    # log.message("ROM members before unsubscribe: "+str(Room_member_ID))

                    # Unsubscribe from old room
                    # log.message("13.4 after Unsubscribe "+" old ID "+ str(agents.old_ID) + " new ID "+str(agents.ID))
                    comm.send(rank, dest=agents.old_ID, tag=91)
                    log.message("tag unsubscribe : "+str(91))

                    # Register in a new room
                    log.message("13.0 before subscription network and the network from the old room ")
                    comm.send(rank, dest=agents.ID, tag=51)
                    # log.message("12.0 before send q new member from "+str(rank)+" to:  "+str(agents.ID))
                    # weight_q_to=agents.learning.q_model.get_weights()
                    # comm.send(weight_q_to, dest=agents.ID, tag=4*(size+1)+agents.ID*10000+rank+100)
                    # log.message("12.0 after send q new member from "+str(rank)+" to:  "+str(agents.ID))
                    # weight_target_to=agents.learning.target_model.get_weights()
                    # comm.send(weight_target_to, dest=agents.ID, tag=5*(size+1)+agents.ID*10000+rank+100)
                    # log.message("12.0 after send target new member from "+str(rank)+" to:  "+str(agents.ID))

                # When the agent achieves the target, it calculates its total
                #reward and sends it to the different memories
                agents.work_out_best_state()
                log.message("15 after work out state " +str(rank))

                # SE HACE EN LA NUBE 1
                # agents.learning.save_model(rank,e)
                # log.message("16 after learning save model" +str(rank))

                    # break
                # print("DONE BEFORE DONE ", agents.done)
                if agents.time_out(step):
                    log.message("Time OUT!! and Agent sends to CLOUD 1 the order to update Pa with tag 321")
                    d = True    #para enviar algo
                    comm.send(d, dest=0, tag=321)
                    log.message("Time out!!")
                    break
                if agents.done:
                    #envia a CLOUD 2 AVISO PARA ACTUALIZAR PA
                    #check if has to go at the begining of done
                    if not comm.Iprobe(source=0,tag=321):
                        Done = [True,rank]
                        log.message("16 Agent sends to CLOUD 1 the order to update Pa with tag 321")
                        # log.message("before Done from " +str(agents.ID)+" to "+str(rank))
                        comm.send(Done, dest=0, tag=321)
                        # log.message("after Done from " +str(agents.ID)+" to "+str(rank))

                        # sys.stdout.flush()
                    agents.keep_or_reset()
                    log.message("after keep or reset " +" to "+str(rank))
                    # agents.done =False
                    if agents.finish:
                        agents.finish = False
                        log.message("break becouse finish" +" to "+str(rank))
                        break

                    if agents.evolve_rule:
                        agents.process="collision"
                        e +=1
                        agents.score = 0
                        agents.done  = False

                        agents.cont+=1
                        agents.learning.increase_fact()
                        # agents.last_heading=list()
                        agents.environment.reset(agents.robot_position_x, agents.robot_position_y)
                        log.message("before heading evolve rule" +str(rank))
                        agents.goal_heading_initial = agents.heading
                        log.message("after goal initial " +" to "+str(rank))

                        if agents.cont > 20:
                            agents.cont=0
                            log.message("break because cont " +" to "+str(rank))
                            break
                    else:
                        break
                    log.message("17 after done "+str(rank))
                agents.last_state=agents.state
                log.message("after last state "+str(rank))

                    # sys.stdout.flush()
            #Increase Pa
            agents.learning.increase_fact()
            log.message("18 after increase factor")
            # sys.stdout.flush()

#################################################################################
# Process to be followed for each cloud
#################################################################################
# One cloud is one room
if rank<number_rooms:
    with tf.device('/cpu:0'):
        cluster=ReinforcementNetwork(state_size,action_size,number_episode,load=False)
        step=0
        # Room_member_ID=[]
        while not rospy.is_shutdown():
            # Registering to the room
            step+=1
            log.message("step "+str(step))
            # Cluster starts training the network q
            train= cluster.start_training()
            log.message("10 Training")
            # sys.stdout.flush()
            # dones=False
            # while comm.Iprobe(source=MPI.ANY_SOURCE,tag=4*(size+1)+rank*10000):
            #     log.message("receive done")
            #     Dones=comm.recv(source=MPI.ANY_SOURCE, tag=4*(size+1)+rank*10000)
            #     dones=Dones[0]
            #     ag=Dones[1]
            #     print("DDD",dones,ag)
            #     cluster.increase_fact()
            #     cluster.update_target_cloud()
            #     log.message( "19.1 UPDATE TARGET NETWORK CLOUD for done")
            #     if not comm.Iprobe(source=ag,tag=10*(size+1)+rank*10000+g+100):
            #         weight_q      = cluster.q_model.get_weights()
            #         weight_target = cluster.target_model.get_weights()
            #         log.message("11.0 waiting for send network done ",ag)
            #         #Sending q network to each agent
            #         comm.send(weight_q,dest=ag,tag=10*(size+1)+rank*10000+g+100)
            #         #Sending target network to each agent
            #         comm.send(weight_target,dest=ag,tag=20*(size+1)+rank*10000+g+100)
            #         dones=False

            if (step%2000==0) and (train==True) :
                cluster.update_target_cloud()
                log.message( "19.1 UPDATE TARGET NETWORK CLOUD "+str(rank))
                # sys.stdout.flush()




            if step%1020==0 and (step>0):
                cluster.save_model(rank,step)
                log.message("20 Save model")

                # sys.stdout.flush()

            if not train:
                time.sleep(2)
                log.message("21 Sleeping")
                # sys.stdout.flush()

            # # NO HACE FALTA TENER REGISTRO DE QUIEN HAY EN EL ROOM
            # # Comunication section
            # # n*(size+1)+rank*10000
            # # Agent sended to the cloud its ID in order to register in the cloud
            # # also q, target networks are send to the cloud
            # while comm.Iprobe(source=MPI.ANY_SOURCE,tag=0*(size+1)+rank*10000):
            #     member_ID=comm.recv(source=MPI.ANY_SOURCE, tag=0*(size+1)+rank*10000)
            #     Room_member_ID.append(member_ID)
            #     log.message("14.0 Agent recive a new member merge ANOTHER AREA"+": "+str( member_ID)+"tag: "+str(0*(size+1)+rank*10000))
            #
            #     weight_q_to=comm.recv(source=MPI.ANY_SOURCE, tag=4*(size+1)+rank*10000+member_ID+100)
            #     log.message("14.3  recive q MERGE ANOTHER AREA from" +" to "+str(rank)+"tag: "+str(4*(size+1)+rank*10000+member_ID+100))
            #     weight_target_to=comm.recv(source=MPI.ANY_SOURCE,tag=5*(size+1)+rank*10000+member_ID+100)
            #     log.message("14.3  recive target MERGE ANOTHER AREA from" +" to "+str(rank)+"tag: "+str(5*(size+1)+rank*10000+member_ID+100))
            #     cluster.merge_target_cloud(weight_q_to,weight_target_to)
            #     log.message("14.3  cloud MERGE ANOTHER AREA from" +" to "+str(rank))

            dones=False
            if comm.Iprobe(source=MPI.ANY_SOURCE,tag=321):
                log.message("before update PA" +str(rank))
                Dones=comm.recv(source=MPI.ANY_SOURCE, tag=321)
                # log.message(" receive done" +str(rank)+"tag: "+str(321))
                # dones=Dones[0]
                # ag=Dones[1]
                # print("DDD",dones,ag)
                # sys.stdout.flush()
                cluster.increase_fact()
                cluster.update_target_cloud()
                log.message( "19.1 UPDATE TARGET s CLOUD for done, tag " + str(321)+ "New PA CLOUD 1: "+str(cluster.Pa))
                comm.send(dones,dest=1,tag=333)
                log.message("29 CLOUD 1 sends alert to CLOUD 2 to update Pa with tag 333. "+" "+str(rank))


                # dones=False
            # log.message("ROM members before unsubscribe cloud : "+str(Room_member_ID))

            # NO HACE FALTA TENER REGISTRO DE QUIEN HAY EN EL ROOM
            # Unsubscribe from room
            # while comm.Iprobe(source=MPI.ANY_SOURCE,tag=1*(size+1)+rank*10000):
            #     log.message("Getting unsubscribe message " + str(rank))
            #     member_ID=comm.recv(source=MPI.ANY_SOURCE, tag=1*(size+1)+rank*10000)
            #     log.message("tag member to unsubscribe "+str(1*(size+1)+rank*10000))
            #     try :
            #         Room_member_ID.remove(member_ID)
            #         log.message("14.4 list of members Unsubscribe: "+str(member_ID))
            #     except:
            #         log.message("14.5 Error in list of members Unsubscribe")
            #     log.message("ROM members after unsubscribe cloud : "+str(Room_member_ID))

            if comm.Iprobe(source=MPI.ANY_SOURCE,tag=11):
                log.message("7.0 waiting append data memory D " + str(rank) +" tag: "+str(11))
                data = comm.recv(source=MPI.ANY_SOURCE, tag=11)
                log.message("7.1  data memory D received")
                for i in range(len(data)):
                    cluster.append_D(data[i][0], data[i][1],data[i][2], data[i][3], data[i][4])

            # Ask if any agent has sent data from memory_G
            # log.message("7.0 waiting append data memory D " + str(rank) +"tag: "+str(3*(size+1)+rank*10000))
            if comm.Iprobe(source=MPI.ANY_SOURCE,tag=21):
                log.message("9.0 waiting second append data " + str(rank)+" tag: "+str(21))
                data_G = comm.recv(source=MPI.ANY_SOURCE, tag=21)
                log.message("9.1  data memory GT received")
                for p in range(len(data_G)):
                    cluster.append_GT(data_G[p][0], data_G[p][1],data_G[p][2], data_G[p][3], data_G[p][4])


            # Updating the networks of the registered agents in the cloud
            # with the current cloud network
            if ((step%20==0) and (step>0)) and (train==True):
                # for g in Room_member_ID:
                if not comm.Iprobe(source=MPI.ANY_SOURCE,tag=14):
                    weight_q      = cluster.q_model.get_weights()
                    weight_target = cluster.target_model.get_weights()
                    log.message("11.0 waiting for sending q network from " + str(rank)+ " to CLOUD 2")
                    #Sending q network to each agent
                    comm.send(weight_q,dest=1,tag=14)
                    log.message("11.0 send q network " + str(rank) + " to CLOUD 2")
                    # #Sending target network to each agent
                    # comm.send(weight_target,dest=1,tag=24)
                    # log.message("11.0  send  target network " + str(rank)+ " to CLOUD 2")
                    #Sending members of the same room to each agent
                    # comm.send(Room_member_ID,dest=1,tag=3*(size+1)+rank*10000+100)
                    # log.message("11.0  for send id network " + str(rank)+ " to CLOUD 2")
                    # #Sending Pa members of the same room to each agent
                    # comm.send(cluster.Pa,dest=1,tag=6*(size+1)+rank*10000+100)
                    # log.message("11.0  for send pa network " + str(rank)+ " to CLOUD 2")


if rank>number_rooms-1 and rank<number_rooms*2:

    cluster2=ReinforcementNetwork(state_size,action_size,number_episode,load=False)
    Room_member_ID = []
    while not rospy.is_shutdown():

        while comm.Iprobe(source=MPI.ANY_SOURCE,tag=51):
            member_ID=comm.recv(source=MPI.ANY_SOURCE, tag=51)
            Room_member_ID.append(member_ID)
            log.message("24.0 (CLOUD 2) Agent recive a new member merge ANOTHER AREA"+": "+str( member_ID)+"tag: "+str(51))

            # weight_q_to=comm.recv(source=MPI.ANY_SOURCE, tag=4*(size+1)+rank*10000+member_ID+100)
            # log.message("24.3 (CLOUD 2) recive q MERGE ANOTHER AREA from" +" to "+str(rank)+"tag: "+str(4*(size+1)+rank*10000+member_ID+100))
            # weight_target_to=comm.recv(source=MPI.ANY_SOURCE,tag=5*(size+1)+rank*10000+member_ID+100)
            # log.message("24.3 (CLOUD 2) recive target MERGE ANOTHER AREA from" +" to "+str(rank)+"tag: "+str(5*(size+1)+rank*10000+member_ID+100))
            # cluster.merge_target_cloud(weight_q_to,weight_target_to)
            # log.message("24.3 (CLOUD 2) cloud MERGE ANOTHER AREA from" +" to "+str(rank))


        while comm.Iprobe(source=MPI.ANY_SOURCE,tag=91):
                log.message("(CLOUD 2) Getting unsubscribe message " + str(rank))
                member_ID=comm.recv(source=MPI.ANY_SOURCE, tag=91)
                log.message("CLOUD 2 tag member to unsubscribe "+str(91))
                try :
                    Room_member_ID.remove(member_ID)
                    log.message("24.4 CLOUD 2 list of members Unsubscribe: "+str(member_ID))
                except:
                    log.message("24.5 CLOUD 2 Error in list of members Unsubscribe")
                log.message("CLOUD 2 ROM members after unsubscribe cloud : "+str(Room_member_ID))


        # for n_robot in Room_member_ID_cloud2:        PARA MAS DE UN ROBOT EN LA MISMA ROOM
        #
        #     if comm.Iprobe(source=n_robot, tag=125):
        #         state = comm.recv(source=2, tag = 125)
        #         log.message("25 CLOUD 2 receives status from agent to calculate state with tag 125. ")
        #         act, evolve_r = cluster2.get_action(np.array(state))
        #         act_evol = [act, evolve_r]
        #         comm.send(act_evol,dest=n_robot,tag=100)
        #
        #         # print("nube 2 envia act y evolve", n_robot)
        #         # sys.stdout.flush()
        #         log.message("26 CLOUD 2 sends action, evolve_rule and q_value to agent with tag 100")


        if comm.Iprobe(source=2, tag=125):            # PARA UN UNICO ROBOT EN LA HABITACION
            state = comm.recv(source=2, tag = 125)
            log.message("25 CLOUD 2 receives status from agent to calculate state with tag 125. ")
            act, evolve_r = cluster2.get_action(np.array(state))
            # print(state)
            act_evol = [act, evolve_r]
            comm.send(act_evol,dest=2,tag=100)

            # print("nube 2 envia act y evolve", n_robot)
            # sys.stdout.flush()
            log.message("26 CLOUD 2 sends action, evolve_rule and q_value to agent with tag 100")


        # if the agent has data to receive  recibe los pesos de la nube 1
        if comm.Iprobe(source=0,tag=14):
            log.message("12.0 before receive network tag:"+str(14))
            weight_q=comm.recv(source=0,tag=14)
            print("pesos recibidos. PA: ", cluster2.Pa)
            print(weight_q)
            cluster2.q_model.set_weights(weight_q)
            # weight_target=comm.recv(source=0,tag=24)
            # cluster2.target_model.set_weights(weight_target)
            log.message("12.0 after receive q network from "+ str(0) +" TAG: "+ str(14) +" and "+ str(24))
            # Room_member_ID=comm.recv(source=0,tag=3*(size+1)+agents.ID*10000+rank+100)
            # log.message("ROM members after unsubscribe : "+str(Room_member_ID)+"TAG: "+str(3*(size+1)+agents.ID*10000+rank+100))
            #
            # log.message("12.0 before receive target network from "+str(rank)+" to:  "+str(agents.ID))
            # agents.Pa=comm.recv(source=0,tag=6*(size+1)+agents.ID*10000+rank+100)
            # log.message("12.0 before receive Pa from "+str(rank)+" to:  "+str(agents.ID)+"TAG: "+str(6*(size+1)+agents.ID*10000+rank+100))


        if comm.Iprobe(source=0,tag=333):                #CAMBIO
            alert=comm.recv(source=0,tag=333)
            cluster2.increase_fact()
            log.message("28 CLOUD 2 receives alert from CLOUD 1 to update Pa with tag 333. "+" "+str(rank)+ "New PA CLOUD 1: "+str(cluster2.Pa))
