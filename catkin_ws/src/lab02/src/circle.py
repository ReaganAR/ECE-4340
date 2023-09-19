#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def circleNode():
    vel_msg = Twist()

    # Input velocity
    vel_msg.linear.x = input("Input speed: ")
    vel_msg.angular.z = input("Input Z rotation: ")


    # Set up node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('circleNode')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        circleNode()
    except rospy.ROSInterruptException:
        pass