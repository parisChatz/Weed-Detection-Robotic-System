#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sensor_msgs.msg
from geometry_msgs.msg import Twist



def main():
    rospy.init_node('listener', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("scan", sensor_msgs.msg.LaserScan , HOW AM I GOING TO GET THE LASERS AND HOW AM I GOING TO MOVE?)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        command = Twist()
        command.linear.x = 2
        command.angular.z = RANDOM???
        pub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
