#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist

import numpy
import tf
import math

import os

polarCoords = [0,0,0]
mHr = numpy.identity(4)

rHc = numpy.identity(4)
rHc[0,3] = 0.067357083
rHc[1,3] = -0.009439617
rHc[2,3] = 0.211620667

# Gains
GAIN_RHO = 0.3
GAIN_ALPHA = 0.8
GAIN_BETA = -0.15

# quat2 = tf.transformations.quaternion_from_euler(-1.67493, 0.01726275, -1.575038333, axes='sxyz')
# euler2 = tf.transformations.euler_from_quaternion(quat2)
# rotation_matrix2 = numpy.array([[math.cos(euler2[2]), -math.sin(euler2[2]), 0],
#                         [math.sin(euler2[2]), math.cos(euler2[2]), 0],
#                         [0, 0, 1]])
# rHc[:3,:3] = rotation_matrix2

'''
HELPER FUNCTIONS
'''
# Makes sure an angle is between -pi and +pi
def normalizeAngle(angle):
    angle = angle % (2 * math.pi)
    if (angle > math.pi):
        angle -= 2 * math.pi
    return angle

# Calculates current rho alpha and beta and stores in global 'polarCoords'
def cart2pol():
    global polarCoords, mHr

    x = mHr[0,3]
    y = mHr[1,3]

    rotation_matrix = numpy.identity(3)
    rotation_matrix[:3,:3] = mHr[:3,:3]

    euler = tf.transformations.euler_from_matrix(rotation_matrix)

    rho = numpy.sqrt(x ** 2 + y ** 2)
    alpha = normalizeAngle(math.atan2(-y, -x) - euler[2])
    
    if (alpha > math.pi/2):
        alpha -= math.pi
    elif (alpha <= -math.pi):
        alpha += math.pi  
    
    beta = -alpha - euler[2]

    polarCoords[:3] = rho, alpha, beta



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
        mHc = numpy.identity(4)

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



        mHc[:3,3] = numpy.array(trans)
        mHc[:3,:3] = rotation_matrix
        
        os.system('clear')
        mHr = numpy.dot(mHc, numpy.linalg.inv(rHc)) # Robot w.r.t the marker (or marker to robot)


        print('x:', mHr[0,3])
        print('y:', mHr[1,3])
        print('z:', mHr[2,3])
        # print('x:', -mHr[2,3])
        # print('y:', mHr[1,3])
        # print('z:', -mHr[0,3])


        print(mHr)
        rospy.logerr("STOP HERE")
        

        mHr[0,3] = math.abs(mHr[0,3]) - 0.15 # Adjust stop position to not collide with marker

        cart2pol()

        vel_msg.linear.x = GAIN_RHO * polarCoords[0]
        vel_msg.angular.z = (GAIN_ALPHA * polarCoords[1]) + (GAIN_BETA * polarCoords[2])
        vel_pub.publish(vel_msg)


                # Stop when close enough to threshold
        pos_thresh = 0.05
        angle_thresh = 0.15

        if abs(mHr[0,3]) <= pos_thresh and abs(mHr[1,3]) <= pos_thresh:
            break
            
        rate.sleep()
