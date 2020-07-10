import urx
import time
import rospy
from math import pi
from time import sleep
from numpy import deg2rad

'''
This is a dummy script used to test quickly some commands related to UR10 control
'''

a = 0.2
v = 0.2

def wait():
    if do_wait:
        print("Click enter to continue")
        increment = input()
        return increment

do_wait = True
move = False
move2 = True

home_position = [0, -pi/2 , 0, 0, 0,0 ]
print(home_position)

gain = 5

rob = urx.Robot("172.31.1.3")
rob.set_tcp((0, 0, 0, 0, 0, 0))
rob.set_payload(0, (0, 0, 0.05))
sleep(0.5)  #leave some time to robot to process the setup commands

try: 
    current_pos = rob.getj()
    print('pose: ', current_pos)
    current_pos[3]+= deg2rad(60)
    current_pos[2]+= deg2rad(30)
    print(current_pos)
    
    if move:
        rob.movej(home_position, a, v, wait= False)
        time.sleep(5)
        increment  = wait()
        print(increment, deg2rad(float(increment)))
        j1 = current_pos[0] + deg2rad(float(increment))
        j2 = current_pos[1] + deg2rad(float(increment))
        j3 = current_pos[2] + deg2rad(float(increment))
        j4 = current_pos[3] + deg2rad(float(increment))
        j5 = current_pos[4] + deg2rad(float(increment))
        j6 = current_pos[5] + deg2rad(float(increment))
        des_pos = [j1,j2,j3,j4,j5,j6]
        print(des_pos)
        rob.movej(des_pos,a, v, wait=False)
        time.sleep(5)
        print(rob.getj())
    
    
    if move2: 
        _ = wait()
        #rob.speedl([0.5,0.1,0,1.57,0,0], 0.5 , 0.1)
        i=0

        robot_pos = rob.getl(wait=True)
        goal = (0.9, 0.1, 0.5,0,0,0)
        # goal = (0.59, -0.63, 0.66)

        #while goal[0]-robot_pos[0] >0.001 or goal[1]-robot_pos[1]> 0.001 or goal[2]-robot_pos[2]>0.001 :
        while i <=10:

            # rob.speedl([gain*(goal[0]-robot_pos[0]),gain*(goal[1]-robot_pos[1]),\
                # gain*(goal[2]-robot_pos[2]),0,0,0], 5,0.20)
            rob.speedl([-20,-20,-20,0,0,0], 0.1, 0.1)
            # robot_pos = rob.getl(wait=True)
            i+=1
        rob.stopl(acc = -0.5)
        #time.sleep(2)

finally: 
    rob.close()
    
'''rob.movel((x, y, z, rx, ry, rz), a, v)
print "Current tool pose is: ",  rob.getl()
rob.movel((0.1, 0, 0, 0, 0, 0), a, v, relative=true)  # move relative to current pose
rob.translate((0.1, 0, 0), a, v)  #move tool and keep orientation
rob.stopj(a)

robot.movel(x, y, z, rx, ry, rz), wait=False)
while True :
    sleep(0.1)  #sleep first since the robot may not have processed the command yet
    if robot.is_program_running():
        break

robot.movel(x, y, z, rx, ry, rz), wait=False)
while.robot.getForce() < 50:
    sleep(0.01)
    if not robot.is_program_running():
        break
robot.stopl()

try:
    robot.movel((0,0,0.1,0,0,0), relative=True)
except RobotError, ex:
    print("Robot could not execute move (emergency stop for example), do something", ex)'''