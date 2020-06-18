#!/usr/bin/env python3

import urx
import time
import rospy
from rospy_tutorials.msg import Floats
from time import sleep
from numpy import deg2rad


def wait():
    if do_wait:
        print("Click enter to continue")
        input()

do_wait = True



rob = urx.Robot("172.31.1.3")
rob.set_tcp((0, 0, 0, 0, 0, 0))
rob.set_payload(0.5, (0, 0, 0.05))
sleep(0.5)  #leave some time to robot to process the setup commands



def talker():
    joint_states_pub = rospy.Publisher('joints_state',Floats,queue_size=10)
    robot_state_pub  = rospy.Publisher('end_effector_position', Floats, queue_size=10)

    rospy.init_node('UR10', anonymous=True)
    r = rospy.Rate(10) # 10hz

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