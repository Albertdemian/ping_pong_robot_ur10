#!/usr/bin/env python3

import urx
import time
import rospy
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from dynamic_reconfigure.msg import Config
from time import sleep
from numpy import deg2rad, zeros, rad2deg
import helper_fns



def check_bounds(ball_pose, xlim=[0.7, 1.4], ylim=[-0.5, 0.5], zlim=[-0.2, 0.8]):
    x = ball_pose.x
    y = ball_pose.y
    z = ball_pose.z 

    if x > xlim[0] and x< xlim[1]: 
        check_x = True
    else:
        check_x = True
    
    if y > ylim[0] and y< ylim[1]:
        check_y = True
    else:
        check_y = False
    
    if z > zlim[0] and z< zlim[1]:
        check_z = True 
    else:
        check_z = False



    return check_x, check_y, check_z

def ball_is_close(ball_pose, x_plane = 0.9, limit = 0.15):
    x = ball_pose.x
    if x > x_plane-limit and x< x_plane+limit:
        follow = True
    else: 
        follow = False
    return follow
    

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
    # print("BALL:", ball_pose)

ball_in_scene_flag = Float64()
def flag_callback(ball_flag):
    global ball_in_scene_flag
    ball_in_scene_flag = ball_flag.data

    # print(ball_in_scene_flag)





K = 2

rospy.init_node("catch_ball")
r = rospy.Rate(60)

cartesian_sub  = rospy.Subscriber("/end_effector_position", Floats, cartes_callback)
ball_pose_sub = rospy.Subscriber("/ball_position_pub", Point,ball_callback )
ball_flag_sub = rospy.Subscriber("/ball_in_a_scene_flag", Float64, flag_callback)
ball_poses = []

start_time = time.time()
cur_time = 0
trajectory = helper_fns.ball_projectile()
check = False
x_plane_shift = 1
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

        

        

        ball_in_scene = interpret_flag(ball_in_scene_flag)
        # print(ball_in_scene)
        if ball_in_scene: 
            ball_x = ball_pose.x
            ball_y = ball_pose.y
            ball_z = ball_pose.z



        check_x, check_y, check_z =  check_bounds(ball_pose)
        # print("moving")

        vx = K*(x_plane_shift-x)
        vy = K*(ball_y-y)
        vz = K*(ball_z-z)
        
        if not check_x: 
            vx = 0
        
        if not check_y: 
            vy = 0
            
        if not check_z: 
            vz = 0
        # rob.speedl([K*(pose[0]-x), K*(pose[1] - y), K*(pose[2] -z),0,0,0], 5,0.3)
        

        if ball_is_close(ball_pose, x_plane=x_plane_shift, limit= 0.15): 
            vx = K* (ball_x - x)
        
        rob.speedl([vx,vy ,vz ,0,0,0],8, 0.2)

        # else: 
        #     rob.stopl(10)

    except (TypeError, ZeroDivisionError, NameError)  :
        pass
 




    #ball = helper_fns.ball_projectile(poses=ball_poses, time_step=1/fps)



