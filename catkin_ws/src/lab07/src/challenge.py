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
from ar_pose.msg import ARMarkers

# Other Labs
import deadreckoning as deadr

'''GLOBALS'''
# Communication
vel_msg = Twist()
ar_visible = False

# Gains & Movement
GAIN_RHO = 0.3
GAIN_ALPHA = 1.4
GAIN_BETA = -0.2

pos_thresh = 0.05
angle_thresh = 0.15

# Transformations
init_H_curr = np.identity(4)
init_H_goal= np.identity(4)

curr_H_goal = np.identity(4)

xyz = [0,0,0]
rpy = [0,0,0]


''' CALLBACK FUNCTIONS '''
# Callback function for AR data
def ar_callback(data):
    global ar_visible, rpy, xyz
    global curr_H_goal

    if len(data.markers) > 0: # If marker exists, use the first to calculate goal position
        ar_visible = True

        position = data.markers[0].pose.pose.position
        xyz = [position.z, position.y, -position.x]

        orientation = data.markers[0].pose.pose.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        rpy = list(tf.transformations.euler_from_quaternion(orientation))

        curr_H_goal[0,3] = xyz[0]
        curr_H_goal[1,3] = xyz[1]
        curr_H_goal[2,3] = xyz[2]
        # curr_H_goal[:3,:3] = tf.transformations.quaternion_matrix(orientation)[:3,:3]

        curr_H_goal[0,0] = math.cos(rpy[1])
        curr_H_goal[0,1] = math.sin(rpy[1])
        curr_H_goal[1,1] = math.cos(rpy[1])
        curr_H_goal[1,0] = -math.sin(rpy[1])

    else:
        ar_visible = False

# Callback function for odometry data
def odom_callback(data):
    global init_H_curr

    deadr.odom_callback(data)
    init_H_curr = deadr.init_H_curr

''' HELPER FUNCTIONS '''
# Runs feedback algorithm to calculate polar coordinates, 
# Requires 'deadr.goal_H_curr' set before function call
def moveToSpot():
    global init_H_goal
    global vel_msg

    deadr.cart2pol()

    # Calculate velocities
    linvel = GAIN_RHO * deadr.polarCoords[0]
    angvel = (GAIN_ALPHA * deadr.polarCoords[1]) + (GAIN_BETA * deadr.polarCoords[2])
    if(linvel > 0.2):
        linvel = 0.2
    if(angvel > 0.8):
        angvel = 0.8

    vel_msg.linear.x = linvel
    vel_msg.angular.z = angvel

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
    ar_sub = rospy.Subscriber('ar_pose_marker', ARMarkers, ar_callback)

        
    '''Navigate towards FIRST marker'''

    # Rotate in place until marker is visible
    while not ar_visible:
        vel_msg.angular.z = 0.4
        vel_pub.publish(vel_msg)
        rate.sleep()

    init_H_goal = np.dot(init_H_curr, curr_H_goal)
    resetVelocity()
    vel_pub.publish(vel_msg)

    while not rospy.is_shutdown():
        if abs(init_H_goal[0,3] - init_H_curr[0,3]) <= (pos_thresh + 0.8) and abs(init_H_goal[1,3] - init_H_curr[1,3]) <= pos_thresh:
            print("At Goal")
            break

        # Do camera-navigation stuff
        if ar_visible:
            rospy.logwarn("Using ARMarkers")
            deadr.goal_H_curr = np.linalg.inv(curr_H_goal)
            init_H_goal = np.dot(init_H_curr, curr_H_goal)
            moveToSpot()

        # Do dead-reckoning stuff
        if not ar_visible:
            rospy.logwarn("Using odometry")
            deadr.init_H_goal = init_H_goal
            deadr.goal_H_curr = np.dot(np.linalg.inv(deadr.init_H_goal), deadr.init_H_curr)
            moveToSpot()

        # rospy.logwarn(np.linalg.inv(deadr.goal_H_curr))
        # print(rpy)
        
        vel_pub.publish(vel_msg)
        rate.sleep()

    # Rotate 90 degrees left TODO check odom info to measure angle Z directly
    angle = tf.transformations.euler_from_matrix(list(init_H_curr[:3,:3]))[2] + (math.pi / 2)
    goalcoords = [init_H_curr[0,3], init_H_curr[1,3], angle]
    deadr.initGoal(goalcoords)
    while abs(deadr.init_H_goal[0,0] - init_H_curr[0,0]) > angle_thresh:
        deadr.goal_H_curr = np.dot(np.linalg.inv(deadr.init_H_goal), deadr.init_H_curr)
        moveToSpot()
        vel_pub.publish(vel_msg)
        rate.sleep()

    # Move x meters forward
    tempmatrix = np.identity(4)
    tempmatrix[0,3] = 0.3
    deadr.init_H_goal = np.dot(init_H_curr, tempmatrix)
    
    while abs(init_H_goal[0,3] - init_H_curr[0,3]) > (pos_thresh + 0.8) or abs(init_H_goal[1,3] - init_H_curr[1,3]) > pos_thresh:
        deadr.goal_H_curr = np.dot(np.linalg.inv(deadr.init_H_goal), deadr.init_H_curr)
        moveToSpot()
        vel_pub.publish(vel_msg)
        rate.sleep()

    ''' DISPENSE CYLINDER'''
    os.system("matlab -nodisplay -nosplash -nodesktop -r \"run('~/Robotics/Labs/lab05/grabbit.m');exit;\"")

    
    '''Navigate towards SECOND marker'''
