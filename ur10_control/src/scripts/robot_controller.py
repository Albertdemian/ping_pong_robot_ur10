#!/usr/bin/env python3

import urx
import time
import rospy
from rospy_tutorials.msg import Floats
from dynamic_reconfigure.msg import Config
from time import sleep
from numpy import deg2rad, zeros
from sorting_variables import *

'''
This node is used to control robot using interface 
It is responsible for receiving target position in either joint or task spaces
and drive the robot to desired position using point to point position control

Note...: the interface is showing angles values in degrees for making it more user friendly
        but the real convention that is fed to robot is in randian scale
'''

def wait():
    if do_wait:
        print("Click enter to continue")
        input()

do_wait = True





#callback function for robot pos
joints_position = Config()      #variable for retrieving joints position
activate_joints = Config()      #a boolean variable to activate joint space control
execute_trajectory = Config()   #another boolean variable to excute desired trajectory
def pos_callback(robot_joints_pos):
    global joints_position, activate_joints, execute_trajectory
    joints_position = robot_joints_pos.doubles
    activate_joints = robot_joints_pos.bools[1].value
    execute_trajectory = robot_joints_pos.bools[0].value
    '''
    print("pose : \n",joints_position)
    print("Condition", activate_joints)
    print("Execute", execute_trajectory)'''


#callback function for robot pos in cartesian space
cartesian_pos = Config()        #variable to retrieve cartesian position
activate_cartesian = Config()   #a boolean variable to activate task space control
execute_path = Config()         #another boolean variable to excute desired path
def cartes_callback(robot_cartesian_pos):
    global cartesian_pos , activate_cartesian, execute_path
    cartesian_pos = robot_cartesian_pos.doubles
    activate_cartesian  =robot_cartesian_pos.bools[1].value
    execute_path = robot_cartesian_pos.bools[0].value
    '''
    print("cartesian : \n", cartesian_pos)
    print("Condition", activate_cartesian)
    print("Execute", execute_path)'''


a = 0.2  #robot acceleration
v = 0.2  #robot velocity

#connect to robot
rob = urx.Robot("172.31.1.3")    #UR10 TCP/IP adress  can be found or changed in robot settings
rob.set_tcp((0, 0, v, 0, 0, 0))  #to set transformation from end effector to tool 
rob.set_payload(0, (0, 0, 0.0))     #to set payload if there is any
sleep(1)  #leave some time to robot to process the setup commands

rospy.init_node("ur10_commander")
r = rospy.Rate(60) #60Hz

cartesian_sub = rospy.Subscriber("/cartesian_controller/parameter_updates", Config, cartes_callback )
joints_sub  = rospy.Subscriber("/Joint_controller/parameter_updates", Config, pos_callback)

cur_time = time.time()
while not rospy.is_shutdown():

    try:
        #check first if the desired position vector is not empty
        if len(joints_position)>0:
            #update command variable and sort variables in the right order
            goal_joints_pos = arrange_joints(joints_position)
            
        #Same procedure but for task space control
        if len(cartesian_pos)>0:
            goal_cartesian_pos = arrange_cartesian(cartesian_pos)

        #check if it should move in task space or not
        if activate_cartesian and execute_path: 
            print("cartesian")
            # move to target
            rob.movel(goal_cartesian_pos, a,v, wait=False)
            
            #check error to target and keep moving till distance to target is within tolerance
            while rob._get_dist(goal_cartesian_pos, False) > 0.001:
                print("stuck", rob._get_dist(goal_joints_pos, False))
                
                #if distance still bigger than tolerance wait for 100ms 
                time.sleep(0.1)

        
        #same for joint space
        if activate_joints and execute_trajectory:
            
            
            rob.movej(goal_joints_pos, a,v, wait=False, threshold=0.001)
            
            while rob._get_dist(goal_joints_pos, True) > 0.001:
                time.sleep(0.1)
    
    except TypeError:
        pass
 
