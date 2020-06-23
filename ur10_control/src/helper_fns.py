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

        self.window_limit_y = [-1,1]
        self.window_limit_z = [-1,1]
        

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

        
=======

        time_to_plane = self.x_intercept/self.vx
        y_intercept = self.vx* time_to_plane
        z_intercept  = self.vz - (0.5* self.az * time_to_plane**2)

        return y_intercept, z_intercept, time_to_plane




    def check_intercept_bounds(self, y_intercept, z_intercept):
        
        if y_intercept > self.window_limit_y[0]  and y_intercept < self.window_limit_y[1]:
            within_y = True

        if z_intercept > self.window_limit_z[1] and z_intercept < self.window_limit_z[1]:
            within_z = True


        if within_y and within_z:
            within_range = True

        else: 
            within_range =  False

        return within_range

    
    def append_data(self):

        #append velocity and pose

        
    
=======
        # 


    def step(self, pose):
        #update velocity and position of ball 

    def control(self): 
        #control velocty
         trjectory_intercept = get_trajectory_intercept(self)
         





