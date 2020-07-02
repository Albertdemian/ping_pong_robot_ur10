#!/usr/bin/env python

import urx
import time
import rospy
from rospy_tutorials.msg import Floats
from dynamic_reconfigure.msg import Config
from time import sleep
from numpy import deg2rad, zeros, rad2deg
import helper_fns


rob = urx.Robot("172.31.1.3")
rob.set_tcp((0, 0, 0, 0, 0, 0))
sleep(1)  #leave some time to robot to process the setup commands

cartesian_pos = Floats()
def cartes_callback(robot_cartesian_pos):
    global cartesian_pos 
    cartesian_pos = robot_cartesian_pos.data 
    print("cartesian :", robot_cartesian_pos)



rospy.init_node("catch_ball")
r = rospy.Rate(60)

cartesian_sub  = rospy.Subscriber("/end_effector_position", Floats, cartes_callback)

# ball_poses = []

while not rospy.is_shutdown():  # and len(ball_poses)<2:

    x = cartesian_pos[0]
    y = cartesian_pos[1]
    z = cartesian_pos[2]
    phi = cartesian_pos[3]
    theta = cartesian_pos[4]
    epsi = cartesian_pos[5]

    ball_x = ball_pose[0]
    ball_y = ball_pose[1]
    ball_z = ball_pose[2]

    rob.speedl([ball_x-x,ball_y - y, ball_z -z,0,0,0], 1,0.15)




    #ball = helper_fns.ball_projectile(poses=ball_poses, time_step=1/fps)



