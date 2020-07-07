#!/usr/bin/env python3

import urx
import time
import rospy
from rospy_tutorials.msg import Floats
from dynamic_reconfigure.msg import Config
from time import sleep
from numpy import deg2rad, zeros


def wait():
    if do_wait:
        print("Click enter to continue")
        input()

do_wait = True


def arrange_cartesian(cartesian): 
    var = zeros((6),dtype=float)

    for element in cartesian: 
        if element.name == "x": 
            var[0] = element.value
        elif element.name == "y":
            var[1] = element.value
        elif element.name == "z":
            var[2] = element.value
        elif element.name == "phi":
            var[3] = deg2rad(element.value) 
        elif element.name == "theta":
            var[4] = deg2rad(element.value)
        elif element.name == "epsi":
            var[5] = deg2rad(element.value)

    return (var[0],var[1], var[2],var[3],var[4],var[5])

def arrange_joints(joints): 
    var = zeros((6),dtype=float)

    for i  in range(6):
        element = joints[i] 
        #print(element,"____________", element.name)
        if element.name == "Joint_1": 
            var[0] = deg2rad(element.value)
        elif element.name == "Joint_2":
            var[1] = deg2rad(element.value)
        elif element.name == "Joint_3":
            var[2] = deg2rad(element.value)
        elif element.name == "Joint_4":
            var[3] = deg2rad(element.value) 
        elif element.name == "Joint_5":
            var[4] = deg2rad(element.value)
        elif element.name == "Joint_6":
            var[5] = deg2rad(element.value)

    return (var[0],var[1], var[2],var[3],var[4],var[5])


#callback function for robot pos
joints_position = Config()
activate_joints = Config()
execute_trajectory = Config()
def pos_callback(robot_joints_pos):
    global joints_position, activate_joints, execute_trajectory
    joints_position = robot_joints_pos.doubles
    activate_joints = robot_joints_pos.bools[1].value
    execute_trajectory = robot_joints_pos.bools[0].value
    '''
    print("pose : \n",joints_position)
    print("Condition", activate_joints)
    print("Execute", execute_trajectory)'''

cartesian_pos = Config()
activate_cartesian = Config()
execute_path = Config()
def cartes_callback(robot_cartesian_pos):
    global cartesian_pos , activate_cartesian, execute_path
    cartesian_pos = robot_cartesian_pos.doubles
    activate_cartesian  =robot_cartesian_pos.bools[1].value
    execute_path = robot_cartesian_pos.bools[0].value
    '''
    print("cartesian : \n", cartesian_pos)
    print("Condition", activate_cartesian)
    print("Execute", execute_path)'''




 



a = 0.2
v = 0.2

rob = urx.Robot("172.31.1.3")
rob.set_tcp((0, 0, 0, 0, 0, 0))
rob.set_payload(0.5, (0, 0, 0.0))
sleep(1)  #leave some time to robot to process the setup commands

rospy.init_node("ur10_commander")
r = rospy.Rate(60)

cartesian_sub = rospy.Subscriber("/cartesian_controller/parameter_updates", Config, cartes_callback )
joints_sub  = rospy.Subscriber("/Joint_controller/parameter_updates", Config, pos_callback)

cur_time = time.time()
while not rospy.is_shutdown():

    #joints_goal = arrange_joints(joints_position)
    #print(type(joints_position))
    try:
        if len(joints_position)>0:
            goal_joints_pos = arrange_joints(joints_position)
            

        if len(cartesian_pos)>0:
            goal_cartesian_pos = arrange_cartesian(cartesian_pos)

        if activate_cartesian and execute_path: 
            print("cartesian")
            rob.movel(goal_cartesian_pos, a,v, wait=False)
            old_err = 0
            while rob._get_dist(goal_cartesian_pos, False) > 0.001:
                print("stuck", rob._get_dist(goal_joints_pos, False))
                #new_err = rob._get_dist()
                time.sleep(0.1)

        

        if activate_joints and execute_trajectory:
            
            
            rob.movej(goal_joints_pos, a,v, wait=False, threshold=0.001)
            
            while rob._get_dist(goal_joints_pos, True) > 0.001:
                time.sleep(0.1)
    
    except TypeError:
        pass
 
