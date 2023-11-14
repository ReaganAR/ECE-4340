#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

import numpy
import tf
import math

import os

rTc = numpy.identity(4)
rTc[0,3] = 0.067357083
rTc[1,3] = -0.009439617
rTc[2,3] = 0.211620667

quat2 = tf.transformations.quaternion_from_euler(-1.67493, 0.01726275, -1.575038333, axes='sxyz')
euler2 = tf.transformations.euler_from_quaternion(quat2)
rotation_matrix2 = numpy.array([[math.cos(euler2[2]), -math.sin(euler2[2]), 0],
                        [math.sin(euler2[2]), math.cos(euler2[2]), 0],
                        [0, 0, 1]])
rTc[:3,:3] = rotation_matrix2



"""
Main function
"""
if __name__=='__main__':

    # Initialize node
    rospy.init_node('camnav')
    rate = rospy.Rate(10)

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    listener = tf.TransformListener()

    vel_msg = Twist()

    while not rospy.is_shutdown():
        cTb = numpy.identity(4)

        try:
            (trans, quat) = listener.lookupTransform("camera", "4x4_2", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('cant find')
            rate.sleep()
            continue
        
        euler = tf.transformations.euler_from_quaternion(quat)
        rotation_matrix = numpy.array([[math.cos(euler[2]), -math.sin(euler[2]), 0],
                                [math.sin(euler[2]), math.cos(euler[2]), 0],
                                [0, 0, 1]])



        cTb[:3,3] = numpy.array(trans)
        cTb[:3,:3] = rotation_matrix
        
        os.system('clear')
        newmat = numpy.linalg.inv(numpy.dot(cTb, rTc))

        print('x:', -newmat[2,3])
        print('y:', newmat[1,3])
        print('z:', -newmat[0,3])
        print(newmat)

        rate.sleep()
