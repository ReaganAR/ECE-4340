#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from ca_msgs.msg import Bumper

bumper = [None, None]

def bumper_callback(data):
    global bumper

    bumper[0] = data.is_left_pressed
    bumper[1] = data.is_right_pressed


"""
Main function
"""
if __name__=='__main__':

    # Initialize node
    rospy.init_node('obstacle_avoidance')
    rate = rospy.Rate(10)

    # Set up velocity messages
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    bump_sub = rospy.Subscriber('/bumper', Bumper, bumper_callback)

    vel_msg = Twist()

    while not rospy.is_shutdown():
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        if bumper[0] is True:
            vel_msg.angular.z = -0.4
        if bumper[1] is True:
            vel_msg.angular.z = 0.4
        if bumper[0] is True and bumper[1] is True:
            vel_msg.angular.z = 0
            vel_msg.linear.x = -0.3
        # if bumper[0] is True or bumper[1] is True:
        #     t0 = rospy.Time.now().to_sec()
        #     currentDistance = 0

        #     while(currentDistance < 0.15):
        #         # Publish velocity
        #         vel_msg.linear.x = -0.3
        #         vel_pub.publish(vel_msg)

        #         t1=rospy.Time.now().to_sec()
        #         currentDistance = -0.3 * (t1 - t0)
        #         rate.sleep()



        vel_pub.publish(vel_msg)
        rate.sleep()