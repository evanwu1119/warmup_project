#!/usr/bin/env python3

""" This script causes the robot to follow a person by turning
towards the closest LiDAR signal within 3m of the robot and moving 
towards it, slowing down as it approaches to stop a few inches away """

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import numpy as np

class PersonFollower(object):
    """ This node subscribes to LiDAR scan messages and
    publishes Twist messages that cause the robot to follow a moving person"""
    
    def __init__(self):
        rospy.init_node('person_follower')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist(linear=Vector3(),angular=Vector3())

    def process_scan(self, data):
        """ This function processes LiDAR messages to orient and move towards 
        a person within control range using proportional control """ 
        
        # get rid of missing and inf in ranges
        ranges = np.nan_to_num(data.ranges)

        angle = 0 # angle of closest object
        distance = np.inf # distance of closest object

        # for all measurements
        for i in range(360):
            # if distance is not zero and less than previous min
            if ranges[i] != 0 and ranges[i] < distance:
                angle = i
                distance = ranges[i]

        stop_dist = 0.3

        # if target is within range of motion (0.2 m to 3 m)
        if distance <= 2.0 and distance >= stop_dist:
            dist_e = distance - stop_dist # distance error
            
            # normalize angular error on scale -179 to 180
            if angle <= 180:
                ang_e = angle
            else:
                ang_e = angle - 360
            
            self.twist.linear.x = 0.3 * dist_e
            self.twist.angular.z = 0.01 * ang_e

        else:
             self.twist.linear.x = 0
             self.twist.angular.z = 0

        self.publisher.publish(self.twist)

    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    node = PersonFollower()
    node.run()
