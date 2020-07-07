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



def check_bounds(ball_pose, xlim=[0.7, 1.4], ylim=[-0.6, 0.6], zlim=[-0.15, 1]):
    x = ball_pose.x
    y = ball_pose.y
    z = ball_pose.z 

    if x > xlim[0] and x< xlim[1]: 
        check_x = True
    else:
        check_x = False
    
    if y > ylim[0] and y< ylim[1]:
        check_y = True
    else:
        check_y = False
    
    if z > zlim[0] and z< zlim[1]:
        check_z = True 
    else:
        check_z = False

    if check_x and check_y and check_z:
        within_bounds = True
    else: 
        within_bounds = False

    return within_bounds

def interpret_flag(flag): 
    if flag == 0 :
        in_scene = False
    elif flag == 1: 
        in_scene = True

    return in_scene


rob = urx.Robot("172.31.1.3")
rob.set_tcp((0, 0, 0.185, 0, 0, 0))
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
    print("BALL:", ball_pose)

ball_in_scene_flag = Floats()
def flag_callback(ball_flag):
    global ball_in_scene_flag
    ball_in_scene_flag = ball_flag





K = 2

rospy.init_node("catch_ball")
r = rospy.Rate(60)

cartesian_sub  = rospy.Subscriber("/end_effector_position", Floats, cartes_callback)
ball_pose_sub = rospy.Subscriber("/ball_position_pub", Point,ball_callback )
ball_flag_sub = rospy.Subscriber("/ball_in_a_scene_flag", Floats, flag_callback)
ball_poses = []

start_time = time.time()
cur_time = 0
trajectory = helper_fns.ball_projectile()
while not rospy.is_shutdown(): 
    start_time = time.time()
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

        cur_time = time.time() - start_time

        ball_in_scene = interpret_flag(ball_in_scene_flag)

        if ball_in_scene: 
            try:
                trajectory.step([ball_x,ball_y, ball_z], cur_time)
            except ZeroDivisionError:
                pass

            # print("class_____: ",trajectory.ball_position)
            # print("Node _____: ",ball_pose, "\n")
            # print(len(trajectory.xs), "\n")

            if len(trajectory.xs) >2:
                
                try: 
                    trajectory.get_velocity([ball_x,ball_y, ball_z])
                    y_inter, z_inter, time_to_plane = trajectory.get_trajectory_intercept()
                    check = trajectory.check_intercept_bounds(y_inter,z_inter)
                except ZeroDivisionError:
                    pass

                pose = trajectory.control(kick=False)
                # print("flag", pose)
                # print(check)


                if check:
                    print("moving")
                    
                    rob.speedl([K*(pose[0]-x), K*(pose[1] - y), K*(pose[2] -z),0,0,0], 5,0.3)

                else: 
                    rob.stopl(5)

    except (TypeError, ZeroDivisionError)  :
        pass
 




    #ball = helper_fns.ball_projectile(poses=ball_poses, time_step=1/fps)



