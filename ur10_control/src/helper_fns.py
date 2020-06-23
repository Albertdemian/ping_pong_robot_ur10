#!/usr/bin/env python3
import numpy as np





class ball_projectile():

    def __init__(self, poses = poses, time_step = time_step, skipped_frames = 1):
        self.ax = 0
        self.ay = 0
        self.az = -9.81

        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.pose_1 = poses[0]
        self.pose_2 = poses[1]

        self.ball_position = self.pose_2

        self.x_intercept = 0.5

        self.time_step = time_step
        self.skipped_frames = skipped_frames
        self.time_between_frames = skipped_frames* time_step
        self.current_time = 0

        self.diameter= ball_diameter
        self.radius = self.diameter/2


    def get_vel_and_dir(self): 
        #to calculate velocity and direction from frames
        '''
        argumends: 
        poses: list of two poses [x,y,z] of the ball
        time: time between these two poses

        Returns: 
        magnitude of ball velocity (V) and directional vector [vx, vy, vz]/V 
        '''
        x1,y2,z1 = self.pose_1
        x2,y2,z2 = self.pose_2

        self.vx = (x2-x1)/self.time_between_frames
        self.vy = (y2-y1)/self.time_between_frames
        self.vz = (z2-z1)/self.time_between_frames

        magnitude = np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

        return magnitude, [vx/magnitude,vy/magnitude,vz/magnitude]

    def get_trajectory_intercept(self):


    def check_intercept_bounds(self):

    
    def append_data(self):

        #append velocity and pose
        # 
    
    def step(self, pose):
        #update velocity and position of ball 

    def control(self): 
        #control velocty 




