#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
from rospy_tutorials.msg import Floats
from dynamic_reconfigure.msg import Config
from numpy import rad2deg

def callback(config):
    rospy.loginfo("""Reconfigure Request: {Joint_1}, {Joint_2},\ 
          {Joint_3}, {Joint_4}, {Joint_5},{Joint_6},{Activate}, {Execute}""".format(**config))

def callback2(config):
    rospy.loginfo(("""Reconfigure Request: {x}, {y},\ 
          {z}, {phi}, {theta},{epsi},{Activate},{Execute}""".format(**config)))


#callback function for robot pos
joints_position = Floats()
def pos_callback(robot_joints_pos):
    global joints_position
    joints_position = robot_joints_pos.data
    #print("pose : ",joints_position[0])

cartesian_pos = Floats()
def cartes_callback(robot_cartesian_pos):
    global cartesian_pos 
    cartesian_pos = robot_cartesian_pos.data 
    print("cartesian :", robot_cartesian_pos)





rospy.init_node("dynamic_sync_joints")
r = rospy.Rate(0.5)
joints_sub = rospy.Subscriber("/joints_state", Floats, pos_callback)
cartesian_sub  = rospy.Subscriber("/end_effector_position", Floats, cartes_callback)

joints_client = dynamic_reconfigure.client.Client("Joint_controller", timeout=30, config_callback=callback)
cartesian_client = dynamic_reconfigure.client.Client("cartesian_controller", timeout= 30, config_callback=callback2)

i = 0
while not rospy.is_shutdown() and i <6:
    print("Position :", joints_position)

    try: 
        j1 = rad2deg(joints_position[0])
        j2 = rad2deg(joints_position[1])
        j3 = rad2deg(joints_position[2])
        j4 = rad2deg(joints_position[3])
        j5 = rad2deg(joints_position[4])
        j6 = rad2deg(joints_position[5])
        
        joints_client.update_configuration({"Joint_1":j1, "Joint_2": j2, "Joint_3": j3, \
            "Joint_4": j4, "Joint_5": j5, "Joint_6": j6})

        x = cartesian_pos[0]
        y = cartesian_pos[1]
        z = cartesian_pos[2]
        phi = rad2deg(cartesian_pos[3])
        theta = rad2deg(cartesian_pos[4])
        epsi = rad2deg(cartesian_pos[5])

        cartesian_client.update_configuration({"x":x, "y": y, "z": z, "phi": phi,\
            "theta": theta, "epsi": epsi})

    except TypeError:
        pass 
    print(i)
    i +=1 
    r.sleep()