#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


threshold = 3 #THRESHOLD value for laser scan.

class wall_avoid:

    def __init__(self):
        self.publisher = rospy.Publisher('/thorvald_001/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/thorvald_001/scan", LaserScan , self.LaserScanProcess)

    def LaserScanProcess(self, data):
        ranges = np.array(data.ranges)
        min_left = min(ranges[0:360]) 
        min_right = min(ranges[360:720])
        command = Twist()
        command.linear.x = 1.0
        if min_left < threshold:
            command.angular.z = 1.0
        if min_right < threshold:
            command.angular.z = -1.0
        self.publisher.publish(command)

if __name__ == '__main__':
    rospy.init_node('wall_avoidance_node')
    wall_avoid()
    rospy.spin()