#!/usr/bin/env python3
import numpy as np

'''
The purpose of this class is two main things:
- predict ball motion according to projectile motion 
- calculate desired velocity to meet the ball for shooting depending of on concept of plactic collision
*****
[Note]: class is not complete and missing many modules
*****
The main idea is to predict ball position according to projectile motion 
and kick the ball with angle and speed to serve it to desired position

The total image is: 
    - calculate current position of ball 
    - calculate velocity 
    - predict ball projectile
    - calculate proper angle and pose where robot should meet the ball 
    - if "kick" is True:
    - calculate step back 
    - take desired position (where the ball should go when the robot hit it)
    - based on desired position and plastic collision factor:
            calculate desired speed that robot should meet the ball with
'''

class ball_projectile():

    def __init__(self, poses = None,  time_step = 0.01, skipped_frames = 1):
        
        # acceleration components
        self.ax = 0
        self.ay = 0
        self.az = -9.81

        #ball velocity components
        self.vx = 0
        self.vy = 0
        self.vz = 0


        #ball position
        self.ball_position = None
        self.count = 0

        # x-axis distance where robot will always recieve the ball
        self.x_intercept = 0.9

        # y and z limits for the robot to recieve the ball
        self.window_limit_y = [-0.6,0.6]
        self.window_limit_z = [-0.15,1]

        #time information
        self.time_step = time_step
        self.skipped_frames = skipped_frames
        self.time_between_frames = 0
        self.current_time = 0

        #ball information   [Note: not used but might be needed later]
        self.diameter= 0.04
        self.radius = self.diameter/2
        
        #arrays to save data [prefered not to depend on them a lot to keep memory in good shape]
        self.vxs = []
        self.vys  =[]
        self.vzs  =[]
        self.xs = []
        self.ys = []
        self.zs = []



    def get_velocity(self, pose): 
        '''
        function to calculate ball velocity and update position 
        input: new ball position
        output: valocity components on 3 axes
        '''
        x1,y1,z1 = [self.xs[-2], self.ys[-2], self.zs[-2]]
        x2,y2,z2 = pose

        self.vx = (x2-x1)/self.time_between_frames
        self.vy = (y2-y1)/self.time_between_frames
        self.vz = (z2-z1)/self.time_between_frames

        return self.vx, self.vy , self.vz 




    def get_trajectory_intercept(self):
        '''
        This method is to predict ball motion based on projectile model 
        output: - intercept point on y-z plane at x_intercept distance that is predetermined in class above
                - time_left to reach this point
        '''
        time_to_plane = (self.x_intercept-self.ball_position[0])/self.vx
        y_intercept = self.ball_position[1] + self.vy* time_to_plane
        z_intercept  = self.ball_position[2] + (self.vz - (0.5* self.az * time_to_plane**2))

        return y_intercept, z_intercept, time_to_plane




    def check_intercept_bounds(self, y_intercept, z_intercept):
        '''
        Checks if predictied intercept point is within physical bounds
        returns: one boolean variable with True or False
        '''
        
        if y_intercept > self.window_limit_y[0]  and y_intercept < self.window_limit_y[1]:
            within_y = True
        else: 
            within_y = False

        if z_intercept > self.window_limit_z[1] and z_intercept < self.window_limit_z[1]:
            within_z = True
        else:
            within_z = False


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


    def step(self, pose, time):
        '''
        Takes new ball pose and time since previous pose and update data in class
        ********
        [Note]: structure of this function is not the best and can be modified 
        '''
        #update velocity and position of ball 
        self.time_between_frames = time
        self.ball_position = pose

        if len(self.xs) >=2: 
            self.vx, self.vy, self.vz = self.get_velocity(pose)

        self.append_data()
        self.current_time += time

    
    def ball_closing(self):
        '''
        checks if the ball is going towards the robot on x-axis or in the other direction
        if velocity component on x is negative, the ball is approaching the robot
        '''
        if self.vx < 0: 
            coming = True

        else: 
            coming = False

        return coming


    def control(self, kick = False): 

        '''
        The purpose of this function is to calculate the pose that the robot should move to
        to meet the ball in propoer way 

        **** 
        [Note]: function is not complete
        ****

        - The primitive idea is to move to a pose orthogonal to ball direction. 
        - The "Kick" argument is supposed later to calculate a step back to prepare for kicking the ball
        '''
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
