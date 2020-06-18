#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from interface.cfg import cartesian_controllerConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {x}, {y},\ 
          {z}, {phi}, {theta},{epsi},{Activate},{Execute}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("cartesian_controller", anonymous = False)

    srv = Server(cartesian_controllerConfig, callback)
    rospy.spin()