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


'''
This node is responsible for subscribing to ball position and robot positions
and follow the ball in space
The node receives ball position relative to robot base
******
[Note]: time calculation logic has some issues and if used this way to calculate velocity
        It will not give proper results
'''


#Function checks if the robot within some bounds 
#this is to decide whether the robot should move towards the ball or it's out of reach
def check_bounds(ball_pose, xlim=[0.7, 1.4], ylim=[-0.5, 0.5], zlim=[-0.2, 0.8]):
    '''
    The function is designed this way to enable to robot to track the ball in y-z plane
    even if the ball is out of reach on x-axis, the robot will still move
    '''
    x = ball_pose.x
    y = ball_pose.y
    z = ball_pose.z 
    
    
    check_x= True
    
    #to check if ball is within bouns in y-axis
    if y > ylim[0] and y< ylim[1]:
        check_y = True
    else:
        check_y = False
    
    #to check if ball is within reach in z-axis
    if z > zlim[0] and z< zlim[1]:
        check_z = True 
    else:
        check_z = False

    return check_x, check_y, check_z

def ball_is_close(ball_pose, x_plane = 0.9, limit = 0.15):
    '''
    This function detects if the ball is close to robot within some range on x-axis
    '''
    x = ball_pose.x
    if x > x_plane-limit and x< x_plane+limit:
        follow = True
    else: 
        follow = False
    return follow
    

def interpret_flag(flag): 
    '''
    Camera node returns 0 if ball is not in scene
    and returns 1 if ball is in scene
    this function is to convert numerical value to boolean
    '''

    if flag == 0 :
        in_scene = False
    elif flag == 1: 
        in_scene = True

    return in_scene


#connecting to robot
rob = urx.Robot("172.31.1.3")
rob.set_tcp((0, 0, 0.185, 0, 0, 0))
sleep(1)  #leave some time to robot to process the setup commands

#callback function subscribes to robot pose
cartesian_pos = Floats()
def cartes_callback(robot_cartesian_pos):
    global cartesian_pos 
    cartesian_pos = robot_cartesian_pos.data 
    # print("cartesian :", robot_cartesian_pos)

#callback function subscribing to ball pose
ball_pose = Point()
def ball_callback(position_ball):
    global ball_pose
    ball_pose = position_ball
    # print("BALL:", ball_pose)

#callback function subscribing to flag if ball is in scene or not
ball_in_scene_flag = Float64()
def flag_callback(ball_flag):
    global ball_in_scene_flag
    ball_in_scene_flag = ball_flag.data

    # print(ball_in_scene_flag)





K = 2   #proportional gain for control velocity

rospy.init_node("catch_ball")
r = rospy.Rate(60)

#subscribers
cartesian_sub  = rospy.Subscriber("/end_effector_position", Floats, cartes_callback)
ball_pose_sub = rospy.Subscriber("/ball_position_pub", Point,ball_callback )
ball_flag_sub = rospy.Subscriber("/ball_in_a_scene_flag", Float64, flag_callback)


start_time = time.time()
cur_time = 0


check = False
x_plane_shift = 1 #distance where robot should operate away from its base on x-axis
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

        

        
        #check if ball in scene or not
        ball_in_scene = interpret_flag(ball_in_scene_flag)
        # print(ball_in_scene)

        #if ball is in scene, update desired position
        if ball_in_scene: 
            ball_x = ball_pose.x
            ball_y = ball_pose.y
            ball_z = ball_pose.z


        #check if ball is within bounds of reach
        check_x, check_y, check_z =  check_bounds(ball_pose)
        # print("moving")
        
        #calculate desired velocities
        vx = K*(x_plane_shift-x)
        vy = K*(ball_y-y)
        vz = K*(ball_z-z)
        
        #if ball is out of one axis reach, velocity along this axis is set to 0
        if not check_x: 
            vx = 0
        
        if not check_y: 
            vy = 0
            
        if not check_z: 
            vz = 0
        
        
        #if ball is within x-axis tolerance determined by ball_is_close() function 
        #robot will start chasing in x_axis also
        if ball_is_close(ball_pose, x_plane=x_plane_shift, limit= 0.15): 
            vx = K* (ball_x - x)
        
        #transfer command to robot
        rob.speedl([vx,vy ,vz ,0,0,0],8, 0.2)

        # else: 
        #     rob.stopl(10)

    except (TypeError, ZeroDivisionError, NameError)  :
        pass
 




