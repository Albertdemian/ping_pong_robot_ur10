#!/usr/bin/env python3
import numpy as np





class ball_projectile():

    def __init__(self, poses = None, time_step = 0.01, skipped_frames = 1):
        
        # acceleration components
        self.ax = 0
        self.ay = 0
        self.az = -9.81

        #ball velocity components
        self.vx = 0
        self.vy = 0
        self.vz = 0

        #initialization poses
        self.pose_1 = poses[0]
        self.pose_2 = poses[1]

        #ball position
        self.ball_position = self.pose_2

        # x-axis distance where robot will always recieve the ball
        self.x_intercept = 0.5
        # y and z limits for the robot to recieve the ball
        self.window_limit_y = [-1,1]
        self.window_limit_z = [-1,1]

        #time information
        self.time_step = time_step
        self.skipped_frames = skipped_frames
        self.time_between_frames = skipped_frames* time_step
        self.current_time = time_step


        self.diameter= 0.04
        self.radius = self.diameter/2
        
        #arrays to save data 
        self.vxs = []
        self.vys  =[]
        self.vzs  =[]
        self.xs = []
        self.ys = []
        self.zs = []


    def get_vel_and_dir(self): 
        #to calculate velocity and direction from frames

        x1,y2,z1 = self.pose_1
        x2,y2,z2 = self.pose_2

        self.vx = (x2-x1)/self.time_between_frames
        self.vy = (y2-y1)/self.time_between_frames
        self.vz = (z2-z1)/self.time_between_frames

        self.append_data()


    def get_velocity(self, pose): 

        x1,y1,z1 = self.ball_position
        x2,y2,z2 = pose

        vx = (x2-x1)/self.time_between_frames
        vy = (y2-y1)/self.time_between_frames
        vz = (z2-z1)/self.time_between_frames

        return vx, vy , vz 




    def get_trajectory_intercept(self):

        time_to_plane = (self.x_intercept-self.ball_position[0])/self.vx
        y_intercept = self.ball_position[1] + self.vy* time_to_plane
        z_intercept  = self.ball_position[2] + (self.vz - (0.5* self.az * time_to_plane**2))

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
        
        self.vxs.append(self.vx)
        self.vys.append(self.vy)
        self.vzs.append(self.vz)
        self.xs.append(self.ball_position[0])
        self.ys.append(self.ball_position[1])
        self.zs.append(self.ball_position[2])


    def step(self, pose):
        #update velocity and position of ball 
        
        self.ball_position = pose
        self.vx, self.vy, self.vz = self.get_velocity(pose)
        self.append_data()
        self.current_time += self.time_step

    
    def ball_closing(self):
        if self.vx < 0: 
            coming = True

        else: 
            coming = False

        return coming


    def control(self, kick = False): 
        #control velocty
        y_intercept, z_intercept, time_to_plane = self.get_trajectory_intercept()
        vx= self.vx 
        vy= self.vy
        vz= z_intercept+ 0.5 * (self.az) * time_to_plane**2
        magnitude= np.sqrt(vx**2 + vy**2 + vz**2)
        phi= np.arccos(vx/magnitude)
        theta= np.arccos(vy/magnitude)
        psi= np.arccos(vz/magnitude)

        intercept_position = (self.x_intercept, y_intercept, z_intercept, -phi, -theta, -psi)

        if kick: 
            pass
        
        return intercept_position


    def kick(self, a_robot):
        #calculating the distance from the plane inorder to kick the ball at the right time
        y_intercept, z_intercept, time_to_plane = get_trajectory_intercept(self)
        d_x= -0.5 * a_robot * time_to_plane**2 + 0.5
        d_y= -0.5 * a_robot * time_to_plane**2 + y_intercept
        d_z= -0.5 * a_robot * time_to_plane**2 + z_intercept
        d_vx= np.sqrt(2 * a_robot * d_x)
        d_vy= np.sqrt(2 * a_robot * d_y)
        d_vz= np.sqrt(2 * a_robot * d_z)
        d_magnitude= np.sqrt(d_x**2 + d_y**2 + d_z**2)
        



    def reset(self): 
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.current_time = 0

        self.vxs = []
        self.vys  =[]
        self.vzs  =[]
        self.xs = []
        self.ys = []
        self.zs = []
