#!/usr/bin/env python3

import rospy
from rospy_tutorials.msg import Floats
from dynamic_reconfigure.msg import Config
from numpy import deg2rad, zeros

'''
Sometimes variables come from dynamic reconfigurator not in the same order
these functions are used to make sure that everything in desired order
'''
def arrange_cartesian(cartesian): 
    var = zeros((6),dtype=float)

    for element in cartesian: 
        if element.name == "x": 
            var[0] = element.value
        elif element.name == "y":
            var[1] = element.value
        elif element.name == "z":
            var[2] = element.value
        elif element.name == "phi":
            var[3] = deg2rad(element.value) 
        elif element.name == "theta":
            var[4] = deg2rad(element.value)
        elif element.name == "epsi":
            var[5] = deg2rad(element.value)

    return (var[0],var[1], var[2],var[3],var[4],var[5])

def arrange_joints(joints): 
    var = zeros((6),dtype=float)

    for i  in range(6):
        element = joints[i] 
        #print(element,"____________", element.name)
        if element.name == "Joint_1": 
            var[0] = deg2rad(element.value)
        elif element.name == "Joint_2":
            var[1] = deg2rad(element.value)
        elif element.name == "Joint_3":
            var[2] = deg2rad(element.value)
        elif element.name == "Joint_4":
            var[3] = deg2rad(element.value) 
        elif element.name == "Joint_5":
            var[4] = deg2rad(element.value)
        elif element.name == "Joint_6":
            var[5] = deg2rad(element.value)

    return (var[0],var[1], var[2],var[3],var[4],var[5])