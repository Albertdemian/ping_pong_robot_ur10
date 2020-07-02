#!/usr/bin/env python3

import urx
import time
import rospy
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Point
from dynamic_reconfigure.msg import Config
from time import sleep
from numpy import deg2rad, zeros, rad2deg
import helper_fns


rob = urx.Robot("172.31.1.3")
rob.set_tcp((0, 0, 0.15, 0, 0, 0))
sleep(1)  #leave some time to robot to process the setup commands

cartesian_pos = Floats()
def cartes_callback(robot_cartesian_pos):
    global cartesian_pos 
    cartesian_pos = robot_cartesian_pos.data 
    # print("cartesian :", robot_cartesian_pos)

ball_pose = Point()
def ball_callback(position_ball):
    global ball_pose
    ball_pose = position_ball
    # print("BALL:", ball_pose)




rospy.init_node("catch_ball")
r = rospy.Rate(60)

cartesian_sub  = rospy.Subscriber("/end_effector_position", Floats, cartes_callback)
ball_pose_sub = rospy.Subscriber("/ball_position_pub", Point,ball_callback )

# ball_poses = []

while not rospy.is_shutdown():  # and len(ball_poses)<2:

    try:
        if len(cartesian_pos)>0:
                
            x = cartesian_pos[0]
            y = cartesian_pos[1]
            z = cartesian_pos[2]
            phi = cartesian_pos[3]
            theta = cartesian_pos[4]
            epsi = cartesian_pos[5]

            ball_x = ball_pose.x
            ball_y = ball_pose.y
            ball_z = ball_pose.z

            
            
            if rob._get_dist((ball_x, ball_y, ball_z,0,0,0), False)> 0.005:
                rob.speedl([ball_x-x-0.05,ball_y - y, ball_z -z,0,0,0], 0.5,0.2)

            elif rob._get_dist((ball_x, ball_y, ball_z,0,0,0), False) < 0.005: 
                rob.stopl(5)

    except TypeError:
        pass
 




    #ball = helper_fns.ball_projectile(poses=ball_poses, time_step=1/fps)



