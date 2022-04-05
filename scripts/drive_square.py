#!/usr/bin/env python3

""" This script causes the robot to drive in a square by moving
forward for approx. five seconds, then stopping and making 90 
degree turns at the corner or the square path"""

import rospy
from geometry_msgs.msg import Twist, Vector3

class DriveInSquare(object):
    """ This node publishes TWist messages that cause the robot
    to drive in a square"""
    def __init__(self):
        rospy.init_node('drive_in_square')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    def run(self):
        # Start driving forward initially
        my_twist = Twist(
            linear = Vector3(0.5, 0, 0),  
            angular = Vector3(0, 0, 0)) 
        
        rate = rospy.Rate(0.2) # Set turn rate once every 5 seconds

        rospy.sleep(1)
        self.publisher.publish(my_twist)
        
        # Continuous loop for driving forward and turning in a square 
        while not rospy.is_shutdown():
            # After 5 seconds begin to turn ccw at rate 
            my_twist = Twist(
                linear = Vector3(0, 0, 0), 
                angular = Vector3(0, 0, 1))
            rate.sleep()
            self.publisher.publish(my_twist)

            # After turn is completed drive forward again
            my_twist = Twist(
                linear = Vector3(0.5, 0, 0),  
                angular = Vector3(0, 0, 0)) 
        
            rospy.sleep(1.67) # Turn time = pi/2 rad / (1 rad/s)
            self.publisher.publish(my_twist)

if __name__ == '__main__':
    node = DriveInSquare()
    node.run()
