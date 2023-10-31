#!/usr/bin/env python
import rospy
import actionlib
import numpy as np
import math
import random as rnd
import sensor_msgs.point_cloud2 as pc2
import tf
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist, Pose2D, Transform, Pose, Quaternion
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from research.msg import Arbitraror
from nav_msgs.msg import Odometry
from std_msgs.msg import String

"""
Global variables
"""

"""
TODO: Set up the feedback gains by replacing None.
"""
GAIN_RHO = None 
GAIN_ALPHA = None  
GAIN_BETA = None


angular_w = None 
linear_v = None
rho = 0.0 
alpha = 0.0 
beta = 0.0
theta = 0.0 
odom_not_received = True
goal_to_world = np.identity(4)
robot_to_world = np.identity(4)
robot_to_goal = np.identity(4)
odom_pose = Pose2D()


pos_thresh = 0.005  
orient_thresh = 0.15 


"""
Utility functions
"""
# FIRST FUNCTION:
# This function is the callback for the subscriber.
# It returns the odometry information as the robot moves along its path toward the desired_goal pose
def odom_callback(odom_msg):
    global odom_pose, odom_not_received, robot_to_world
    pose = Pose()
    pose.position.x = odom_msg.pose.pose.position.x
    pose.position.y = odom_msg.pose.pose.position.y
    pose.position.z = 0.0

    #print("x: {} - y: {} - z: {}".format(pose.position.x, pose.position.y, pose.position.z))
    
    quat = (
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quat)

    #print("r: {} - p: {} - y: {}".format(euler[0], euler[1], euler[2]))
    
    pose.orientation.x = odom_msg.pose.pose.orientation.x
    pose.orientation.y = odom_msg.pose.pose.orientation.y
    pose.orientation.z = odom_msg.pose.pose.orientation.z
    pose.orientation.w = odom_msg.pose.pose.orientation.w

    rotation_matrix = np.array([[math.cos(euler[2]), -math.sin(euler[2]), 0],
                                [math.sin(euler[2]), math.cos(euler[2]), 0],
                                [0, 0, 1]])

    robot_to_world[:3,:3] = rotation_matrix
    robot_to_world[:3,3] = np.array([pose.position.x,
                                    pose.position.y,
                                    pose.position.z])                                    

    #print("robot_to_world:\n{}".format(robot_to_world))


    
    odom_pose.x = pose.position.x
    odom_pose.y = pose.position.y
    odom_pose.theta = euler[2]
    #print("From odometry:\nx: {} - y: {} - yaw: {}".format(odom_pose.x, odom_pose.y, euler[2]))

    # check that the odometry was received
    if odom_not_received: odom_not_received = False


# this function normalizes the angles between -pi and +pi
def normalize_angles(angle):
    angle = angle % (2*math.pi)
    if (angle > math.pi):
        angle -= 2*math.pi
    return angle


# SECOND FUNCTION:
# This function calculatesthe 3 parameters of the controller
# These parameters are used to perform the feed-back control loop
def calculate_rho_alpha_beta():
    global rho, theta, alpha, beta
    
    position = Point(); orientation = Quaternion()

    rotation_matrix = np.identity(4)
    rotation_matrix[:3,:3] = robot_to_goal[:3,:3]

    euler = tf.transformations.euler_from_matrix(rotation_matrix)

    position.x = robot_to_goal[0,3]
    position.y = robot_to_goal[1,3]
    position.z = robot_to_goal[2,3]

    theta = euler[2]
    rho = math.sqrt(math.pow(position.x, 2) + (math.pow(position.y, 2)))
    alpha = math.atan2(-position.y, -position.x) - theta
    alpha = normalize_angles(alpha)

   
    if (alpha > math.pi/2):
        alpha -= math.pi
    elif (alpha <= -math.pi):
        alpha += math.pi  
    

    beta = -alpha - theta


    #print("theta: {}, rho: {}, alpha: {}, beta: {}".format(theta, rho, alpha, beta))
    
# THIRD FUNCTION:
# This function finds the transformation between the robot and the robot
# It mainly builds the transformation to be tracked as the robot moves  
def initialize_goal_to_world(x, y, theta):

    if(odom_not_received):

	    position_vector = Point()
	    position_vector.x = x
	    position_vector.y = y
	    position_vector.z = 0.0   

	    rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0],
	                                [math.sin(theta), math.cos(theta), 0],
	                                [0, 0, 1]])

	    goal_to_world[:3,:3] = rotation_matrix
	    goal_to_world[:3,3] = np.array([position_vector.x,
	                                    position_vector.y,
	                                    position_vector.z])

	    print("goal_to_world:\n{}".format(goal_to_world))
    

# FOURTH FUNCTION:
# This function finds the linear and angular velocities based on the polar coordinates
def calculate_velocities():
    global rho, theta, alpha, beta
    global GAIN_RHO, GAIN_ALPHA, GAIN_BETA, linear_v, angular_w

    linear_v = (GAIN_RHO*rho)
    angular_w = (GAIN_ALPHA*alpha) + (GAIN_BETA*beta) 

    #print("linear_v: {} / angular_w: {}".format(linear_v, angular_w))

# clean chutdown of the node
def clean_shutdown():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)

"""
Main function
"""
if __name__=='__main__':

	# declare us to be a node to be shut down safely
    rospy.on_shutdown(clean_shutdown)
    rospy.init_node('controller')

    
    """
    TODO: Set up your publisher and subscriber.by replacing None
    """
    odom_sub = None 
    cmd_vel_pub = None


    # Take input from the user
    args = rospy.myargv(argv=sys.argv)
        
    if len(args) != 4:
        print("Usage: node.py arg1 arg2 arg3")
        sys.exit(1)

    # Take the input goal from the user
    desired_goal = Pose2D()
    desired_goal.x = float(args[1]); 
    desired_goal.y = float(args[2]); 
    desired_goal.theta = math.radians(float(args[3]))


    """
    TODO: Think about how you are going to save the paths to reproduce the figures
    """


    # set the rate 
    rate = rospy.Rate(10)

    
    # initialize the goal to world transformation
    initialize_goal_to_world(desired_goal.x, desired_goal.y, desired_goal.theta)
    
    while not rospy.is_shutdown():

        if odom_not_received == False:
	        print("")
	        rospy.loginfo("Feedback control - go to goal behavior")

	        # update the transformation from robot to goal as the robot is moving
	        robot_to_goal = np.dot(np.linalg.inv(goal_to_world), robot_to_world)
	        
	        # calculate the polar coordinates
	        calculate_rho_alpha_beta()

	        # calculate the linear and angular velocities
	        calculate_velocities()

	        print("linear_v: {} / angular_w: {}".format(linear_v, angular_w))

	        print("\nDesired Robot Pose: ")
	        print("desired_x: {}".format(desired_goal.x))
	        print("desired_y: {}".format(desired_goal.y))
	        print("desired_theta: {}".format(desired_goal.theta))

	        print("\nCurrent Robot Pose: ")
	        print("current_x: {}".format(odom_pose.x))
	        print("current_y: {}".format(odom_pose.y))
	        print("current_theta: {}".format(odom_pose.theta))

	        # set the velocities in the twist message and publish
	        twist = Twist()
	        twist.linear.x = linear_v
	        twist.angular.z = angular_w
	        cmd_vel_pub.publish(twist)

	        # break out of the loop when the threshold is met
	        pos_thresh = 0.005  
	        orient_thresh = 0.15 
	        
            # orientation angle comparison is a little more involved 
            # this is because angle can fall in any quadrand
	        compared_angle = desired_goal.theta - odom_pose.theta
	        compared_angle = (compared_angle+np.pi)%(2*np.pi) - np.pi
	        print("error in yaw: {}".format(compared_angle))

	        if abs(desired_goal.x - odom_pose.x) <= pos_thresh and abs(desired_goal.y - odom_pose.y) <= pos_thresh and abs(compared_angle) <= orient_thresh:
	            break

	        # sleep a little
	        rate.sleep()

