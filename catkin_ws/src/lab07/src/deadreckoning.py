#!/usr/bin/env python

import rospy
import numpy
import math
import tf

import sys
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import os

"""
Global Variables
"""
# Gains
GAIN_RHO = 0.3
GAIN_ALPHA = 0.8
GAIN_BETA = -0.15

# Movement data
angular_w = None
linear_v = None
polarCoords = [0.0, 0.0, 0.0] # Rho, Alpha, Beta
rpy = [0,0,0]

# Position matrixes
init_H_curr = numpy.identity(4)
goal_H_curr = numpy.identity(4)
init_H_goal = numpy.identity(4)

# Thresholds
thresh_pos = 0.005
thresh_orient = 0.15

"""
Utility Functions
"""

# Processes Odometry data and puts info into globals
def odom_callback(odom_data):
    global init_H_curr, rpy
    
    rospy.logdebug("Recieving info")

    init_H_curr[0,3] = odom_data.pose.pose.position.x
    init_H_curr[1,3] = odom_data.pose.pose.position.y
    init_H_curr[2,3] = odom_data.pose.pose.position.z

    quat = (
        odom_data.pose.pose.orientation.x,
        odom_data.pose.pose.orientation.y,
        odom_data.pose.pose.orientation.z,
        odom_data.pose.pose.orientation.w
    )
    
    init_H_curr[:3,:3] = tf.transformations.quaternion_matrix(quat)[:3,:3]

# Makes sure an angle is between -pi and +pi
def normalizeAngle(angle):
    angle = angle % (2 * math.pi)
    if (angle > math.pi):
        angle -= 2 * math.pi
    return angle

# Calculates current rho alpha and beta and stores in global 'polarCoords'
def cart2pol():
    global polarCoords, goal_H_curr

    x = goal_H_curr[0,3]
    y = goal_H_curr[1,3]

    rotation_matrix = numpy.identity(3)
    rotation_matrix[:3,:3] = goal_H_curr[:3,:3]

    euler = tf.transformations.euler_from_matrix(rotation_matrix)

    rho = numpy.sqrt(x ** 2 + y ** 2)
    alpha = normalizeAngle(math.atan2(-y, -x) - euler[2])
    
    if (alpha > math.pi/2):
        alpha -= math.pi
    elif (alpha <= -math.pi):
        alpha += math.pi  
    
    beta = -alpha - euler[2]

    polarCoords[:3] = rho, alpha, beta

# Initializes init_H_goal
def initGoal(goalcoords):
    rospy.logdebug("Initializing goal frame")
    global init_H_goal

    init_H_goal[0,3] = goalcoords[0]
    init_H_goal[1,3] = goalcoords[1]
    init_H_goal[2,3] = 0

    init_H_goal[0,0] = math.cos(goalcoords[2])
    init_H_goal[1,0] = math.sin(goalcoords[2])
    init_H_goal[0,1] = -math.sin(goalcoords[2])
    init_H_goal[1,1] = math.cos(goalcoords[2])


"""
Main function
"""
if __name__=='__main__':

    rospy.init_node('dead_reckoning')
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)
    vel_pub = rospy.Publisher('/deadreckoning/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)

    vel_msg = Twist()

    goalCoords = [0,0,0]
    goalCoords[0]=input("x: ")
    goalCoords[1]=input("y: ")
    goalCoords[2]=input("theta: ") 

    initGoal(goalCoords)

    while not rospy.is_shutdown():
        goal_H_curr = numpy.dot(numpy.linalg.inv(init_H_goal), init_H_curr)

        cart2pol()

        linvel = GAIN_RHO * polarCoords[0]
        if(linvel > 0.2):
            linvel = 0.2

        vel_msg.linear.x = linvel
        vel_msg.angular.z = (GAIN_ALPHA * polarCoords[1]) + (GAIN_BETA * polarCoords[2])
        vel_pub.publish(vel_msg)

        # Stop when close enough to threshold
        pos_thresh = 0.05
        angle_thresh = 0.15

        if abs(goalCoords[0] - init_H_curr[0][3]) <= pos_thresh and abs(goalCoords[1] - init_H_curr[1][3]) <= pos_thresh:
            break
            
        rate.sleep()
