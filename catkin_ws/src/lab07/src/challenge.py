#!/usr/bin/env python

''' IMPORTS '''

# Packages
import rospy
import tf

import numpy as np
import math

import os

# Message Types
from geometry_msgs.msg import Twist
from ar_pose.msg import ARMarker, ARMarkers

''' CALLBACK FUNCTIONS '''
def ar_callback(data):
    global xyz, rpy, first_ar, ar_bias, ar_received, marker_in_view

    if len(data.markers) > 0:
        position = data.markers[0].pose.pose.position
        xyz = [position.z, position.y, -position.x]

        orientation = data.markers[0].pose.pose.orientation
        orientation = [orientation.z, orientation.y, -orientation.x, orientation.w]
        rpy = list(tf.transformations.euler_from_quaternion(orientation))

        marker_in_view = True
    else:
        marker_in_view = False
        return

    if first_ar == True:
        ar_bias = rpy[2]
        first_ar = False

    rpy[2] -= ar_bias
    ar_received = True


''' HELPER FUNCTIONS '''




''' MAIN METHOD '''
if __name__=='__main__':
    rospy.init_node("challenge")
    rate = rospy.Rate(10)

    vel_pub = rospy.Publisher('/challenge/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/ar_pose_marker', ARMarkers, ar_callback)