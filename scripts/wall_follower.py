#!/usr/bin/env python3

""" This script causes the robot to follow a wall ccw """

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np

class WallFollower(object):
    """ This node subscribes to LiDAR scan messages and
    publishes Twist messages that cause the robot to follow a wall ccw """
    
    def __init__(self):
        rospy.init_node('wall_follower')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist(linear=Vector3(),angular=Vector3())

    def process_scan(self, data):
        """ This function processes LiDAR messages to orient and move the robot
        in relation to a wall """ 
        
        # get rid of missing and inf in ranges
        ranges = np.nan_to_num(data.ranges)

        stop_dist = 0.6 # around how far from the wall we want
        angle = 0 # minimum angle
        distance = np.inf # minimum distance

        # for all measurements
        for i in range(360):
            # if distance is not zero and less than previous min
            if ranges[i] != 0 and ranges[i] < distance:
                angle = i
                distance = ranges[i]

        # if within 3.0 m detection range of wall
        if distance <= 3.0:
            dist_e = stop_dist - distance # error is target centered around stop_dist  
            
            # normalize angular error on scale -179 to 180 centered around 90
            if angle <= 270:
                ang_e = 90 - angle
            else:
                ang_e = angle - 180
            
            self.twist.linear.x = 0.1 + 0.2 * abs(dist_e) # proportional linear velocity 
            
            # proportional control of angular velocity, combines: 
            # if not parallel to wall, turn to become parallel and
            # if not at stop_dist from wall, turn towards it
            self.twist.angular.z =  0.005 * ang_e + 2 * dist_e
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        
        self.publisher.publish(self.twist)

    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    node = WallFollower()
    node.run()
