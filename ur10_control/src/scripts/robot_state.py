#!/usr/bin/env python3

import urx
import time
import rospy
from rospy_tutorials.msg import Floats
from time import sleep
from numpy import deg2rad

'''
This node is used to connect to robot and monitor position 
in both task and joint spaces and publish them 
'''

def wait():
    if do_wait:
        print("Click enter to continue")
        input()

do_wait = True


# to connect to robot via network and Initialize the robot class
rob = urx.Robot("172.31.1.3")   #UR10 TCP/IP adress  can be found or changed in robot settings
rob.set_tcp((0, 0, 0.185, 0, 0, 0))     #to set transformation from end effector to tool 
sleep(1)  #leave some time to robot to process the setup commands



def talker():
    joint_states_pub = rospy.Publisher('joints_state',Floats,queue_size=10)
    robot_state_pub  = rospy.Publisher('end_effector_position', Floats, queue_size=10)

    rospy.init_node('UR10', anonymous=True)
    r = rospy.Rate(60) # 60hz

    while not rospy.is_shutdown():
        joints_pos = rob.getj()
        print('Joints pose : ', joints_pos)
        robot_pos = rob.getl(wait=True)
        print('robot position : ', robot_pos)
        joint_states_pub.publish(joints_pos)
        robot_state_pub.publish(robot_pos)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Node was manually terminated using keyboard interrup')