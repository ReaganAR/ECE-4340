#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def squareNode():
    vel_msg = Twist()

    # Input information
    distance = input("Insert length of sides: ")
    speed = 0.2
    angularspeed = 0.1
    angle = 90.0 * 3.14159265359 / 180.0

    # Set up node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('squareNode')
    rate = rospy.Rate(10)

    # Allow node to fully initialize before starting
    rospy.sleep(1)
    # Running loop    
    while not rospy.is_shutdown():
        # Set current time
        currentDistance = 0
        t0 = rospy.Time.now().to_sec()

        # Move turtle specified distance
        while(currentDistance < distance):
            # Publish velocity
            vel_msg.linear.x = speed
            pub.publish(vel_msg)

            t1=rospy.Time.now().to_sec()
            currentDistance = speed * (t1 - t0)
        
        vel_msg.linear.x = 0
        pub.publish(vel_msg)

        t0 = rospy.Time.now().to_sec()
        currentAngle = 0

        # Rotate turtle specified angle
        while(currentAngle < angle):
            # Publish velocity
            vel_msg.angular.z = angularspeed
            pub.publish(vel_msg)

            t1=rospy.Time.now().to_sec()
            currentAngle = angularspeed * (t1 - t0)

        vel_msg.angular.z = 0
        pub.publish(vel_msg)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        squareNode()
    except rospy.ROSInterruptException:
        pass
