#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

isMoving = False

def twistinfo(data):
    rospy.logdebug("Recieving info.")

    global isMoving

    isMoving = True
    if (data.twist.twist.linear.x == 0) and (data.twist.twist.angular.z == 0):
        isMoving = False

def squareNode():
    # Define variables
    global isMoving
    vel_msg = Twist()

    # Input information
    distance = input("Insert length of sides: ")
    speed = 0.4 # 0.2
    angularspeed = 0.7 # 0.8
    angle = (90.0 - 6) * 3.14159265359 / 180.0

    # Set up node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('odom', Odometry, twistinfo)
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
        rospy.loginfo("Moving forward %f meters", distance)
        isMoving = True
        while(currentDistance < distance):
            # Publish velocity
            vel_msg.linear.x = speed
            pub.publish(vel_msg)

            t1=rospy.Time.now().to_sec()
            currentDistance = speed * (t1 - t0)
            rate.sleep()
        vel_msg.linear.x = 0
        pub.publish(vel_msg)

        # Wait until odometry claims robot isn't moving
        while isMoving and not rospy.is_shutdown():
            rospy.logwarn("Sleeping extra time...")
            rospy.sleep(0.5)

        # Set current time
        t0 = rospy.Time.now().to_sec()
        currentAngle = 0

        # Rotate turtle specified angle
        rospy.loginfo("Rotating %f radians", angle)
        while(currentAngle < angle):
            # Publish velocity
            vel_msg.angular.z = angularspeed
            pub.publish(vel_msg)

            t1=rospy.Time.now().to_sec()
            currentAngle = angularspeed * (t1 - t0)
            rate.sleep()
        vel_msg.angular.z = 0
        pub.publish(vel_msg)

        # Wait until odometry claims robot isn't moving
        while isMoving and not rospy.is_shutdown():
            rospy.logwarn("Sleeping extra time...")
            rospy.sleep(0.5)

        rate.sleep()
    
# Create squareNode if not already present
if __name__ == '__main__':
    try:
        squareNode()
    except rospy.ROSInterruptException:
        pass
