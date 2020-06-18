#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from interface.cfg import joint_controllerConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Joint_1}, {Joint_2},\ 
          {Joint_3}, {Joint_4}, {Joint_5},{Joint_6}, {Activate}, {Execute}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("Joint_controller", anonymous = False)

    srv = Server(joint_controllerConfig, callback)
    rospy.spin()