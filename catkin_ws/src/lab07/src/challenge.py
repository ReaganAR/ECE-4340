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
from nav_msgs.msg import Odometry

# Other Labs
import deadreckoning as deadr

'''GLOBALS'''
# Communication
vel_msg = Twist()
ar_visible = False

# Gains & Movement
GAIN_RHO = 0.3
GAIN_ALPHA = 0.8
GAIN_BETA = -0.15

pos_thresh = 0.05
angle_thresh = 0.15

# Transformations
init_H_curr = np.identity(4)
init_H_goal= np.identity(4)


''' CALLBACK FUNCTIONS '''
def ar_callback(data): # Set init_H_goal FROM odometry and marker matrices
    global ar_visible

    ar_visible = True


def odom_callback(data):
    global init_H_curr

    deadr.odom_callback(data)
    init_H_curr = deadr.init_H_curr

''' HELPER FUNCTIONS '''

def moveToSpot():
    global init_H_goal
    global vel_msg

    # Run deadreckoning algorithm to determine odometry-based 
    deadr.init_H_goal = init_H_goal
    deadr.goal_H_curr = np.dot(np.linalg.inv(deadr.init_H_goal), deadr.init_H_curr)
    deadr.cart2pol()

    linvel = GAIN_RHO * deadr.polarCoords[0]
    if(linvel > 0.2):
        linvel = 0.2

    vel_msg.linear.x = linvel
    vel_msg.angular.z = (GAIN_ALPHA * deadr.polarCoords[1]) + (GAIN_BETA * deadr.polarCoords[2])

def resetVelocity():
    global vel_msg
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0

''' MAIN METHOD '''
if __name__=='__main__':
    rospy.init_node("challenge")
    rate = rospy.Rate(10)

    vel_pub = rospy.Publisher('/challenge/cmd_vel', Twist, queue_size=1)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback)

    # Rotate in place until marker is visible
    while not ar_visible:
        vel_msg.angular.z = 0.2
        vel_pub.publish(vel_msg)
        rate.sleep()

    resetVelocity()
    vel_pub.publish(vel_msg)

    # Make sure goal is MARKER 4
    '''TEMPORARY@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@'''
    goalCoords = [0,0,0]
    goalCoords[0]=input("x: ")
    goalCoords[1]=input("y: ")
    goalCoords[2]=input("theta: ") 

    deadr.initGoal(goalCoords)
    '''SETS A GOAL TEMPORARY@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@'''
    
    # Navigate towards goal
    while (abs(init_H_goal[0,3] - init_H_curr[0,3]) <= pos_thresh and abs(init_H_goal[1,3] - init_H_curr[1,3]) <= pos_thresh) and not rospy.is_shutdown:
        # Do camera-navigation stuff
        if ar_visible:
            pass

        # Do dead-reckonign stuff
        if not ar_visible:
            moveToSpot()
        
        vel_pub.publish(vel_msg)
        rate.sleep()
