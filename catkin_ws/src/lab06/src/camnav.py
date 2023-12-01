#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from ar_pose.msg import ARMarker, ARMarkers

import numpy
import tf
import math

import os

polarCoords = [0,0,0]
mHr = numpy.identity(4)

rHc = numpy.identity(4)
rHc[0,3] = 0.015
rHc[1,3] = -0.009439617
# rHc[1,3] = -0.023
rHc[2,3] = 0.235 - 0.05

# Gains
GAIN_RHO = 0.2
GAIN_ALPHA = 0.35
GAIN_BETA = -0.15

# GAIN_ALPHA = 0.4
# GAIN_BETA = -0.05
xyz = [0,0,0]
rpy = [0,0,0]
first_ar = True
ar_bias = 0
ar_received = False

marker_in_view = False

quat2 = tf.transformations.quaternion_from_euler(-1.67493, 0.01726275, -1.575038333, axes='sxyz')

rHc[:3,:3] = tf.transformations.quaternion_matrix(quat2)[:3,:3]

'''
HELPER FUNCTIONS
'''
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

# Makes sure an angle is between -pi and +pi
def normalizeAngle(angle):
    angle = angle % (2 * math.pi)
    if (angle > math.pi):
        angle -= 2 * math.pi
    return angle

# Calculates current rho alpha and beta and stores in global 'polarCoords'
def cart2pol():
    global polarCoords, mHr
    rHm = numpy.linalg.inv(mHr)

    x = rHm[0,3]
    y = rHm[1,3]

    euler = tf.transformations.euler_from_matrix(rHm[:3,:3], 'rxyz')

    rho = numpy.sqrt(x ** 2 + y ** 2)
    alpha = math.atan2(-y, -x) - euler[2]
    
    alpha = alpha % (2 * math.pi)

    if (alpha > math.pi):
        alpha -= 2*math.pi

    beta = -alpha - euler[2]

    polarCoords[:3] = rho, alpha, beta




"""
Main function
"""
if __name__=='__main__':

    # Initialize node
    rospy.init_node('camnav')
    rate = rospy.Rate(1)

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/ar_pose_marker', ARMarkers, ar_callback)
    listener = tf.TransformListener()

    vel_msg = Twist()

    llTrans = 0
    llQuat = 0

    while not rospy.is_shutdown():
        mHc = numpy.identity(4)

        try:
            listener.waitForTransform("camera", "4x4_2", rospy.Time(0), rospy.Duration(4))
            # now = rospy.Time.now()
            # listener.waitForTransform("camera", "4x4_2", now, rospy.Duration(4))
            (trans, quat) = listener.lookupTransform("camera", "4x4_2", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('cant find')
            rate.sleep()
            continue
        except Exception as e:
            rospy.logerr(e)
            rate.sleep()
            continue


        # except Exception as error

        # if llTrans == trans and llQuat == quat:
        #     rospy.logerr("Lost Tracking")
        #     break

        llTrans = trans
        llQuat = quat


        euler = tf.transformations.euler_from_quaternion(quat)

        mHc[:3,3] = numpy.array(trans)
        mHc[:3,:3] = tf.transformations.euler_matrix(euler[0],euler[1],euler[2])[:3,:3]
        
        os.system('clear')
        mHr = numpy.dot(mHc, numpy.linalg.inv(rHc)) # Robot w.r.t the marker (or marker to robot)


        # x = -numpy.abs(mHr[2,3])
        # y = mHr[1,3]

        # print(mHr)
        # print(numpy.linalg.inv(mHr))

        # print('x:', x)
        # print('y:', y)
        # print('z:', mHr[0,3])
        # print(numpy.linalg.inv(mHr))

        cart2pol()

        # print(polarCoords[0])
        # print(math.degrees(polarCoords[1]))
        # print(math.degrees(polarCoords[2]))

        vel_msg.linear.x = GAIN_RHO * polarCoords[0]
        vel_msg.angular.z = (GAIN_ALPHA * polarCoords[1]) + (GAIN_BETA * polarCoords[2])
        vel_pub.publish(vel_msg)

        # print('Linear Velocity:',vel_msg.linear.x)
        # print('Angular Velocity:',vel_msg.angular.z)

        print(xyz)
        print(rpy)
        print(first_ar)
        print(ar_received)




                # Stop when close enough to threshold
        pos_thresh = 0.05
        angle_thresh = 0.15

        # if abs(x) <= pos_thresh and abs(y) <= pos_thresh:
        #     rospy.logwarn("At Goal")
        #     break
            
        rate.sleep()
