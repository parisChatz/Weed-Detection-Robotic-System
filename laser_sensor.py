#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
#import sensor_msgs.msg
from sensor_msgs.msg import LaserScan
import random
import numpy as np
from itertools import *
from operator import itemgetter

threshold = 1.5 #THRESHOLD value for laser scan.
PI = 3.14
Kp = 0.05

linear_x = 2.0 #Linear velocity 2 m/s
angular_z = 0

def LaserScanProcess(data):
    range_angels = np.arange(len(data.ranges))
    ranges = np.array(data.ranges)
    range_mask = (ranges > threshold)
    ranges = list(range_angels[range_mask])
    max_gap = 40
    #print(ranges)
    gap_list = []
    for k, g in groupby(enumerate(ranges), lambda (i,x):i-x):
        gap_list.append(map(itemgetter(1), g))
    gap_list.sort(key=len)
    largest_gap = gap_list[-1]
    min_angle, max_angle = largest_gap[0]*((data.angle_increment)*180/PI), largest_gap[-1]*((data.angle_increment)*180/PI)
    average_gap = (max_angle - min_angle)/2

    turn_angle = min_angle + average_gap

    print(min_angle, max_angle)
    print(max_gap,average_gap,turn_angle)

    #global linear_x
    #global angular_z
    
    if average_gap < max_gap:
        angular_z = -0.5
    else:
        linear_x = 2
        angular_z = Kp*(-1)*(90 - turn_angle)

def main():
    rospy.init_node('laser_sensor', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("scan", LaserScan , LaserScanProcess)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
