#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from math import sin


class object_frame():
    def __init__(self, robotnumber):
        self.robotnumber = robotnumber
        rospy.Subscriber('/thorvald_00' + robotnumber + '/scan', LaserScan,
                         self.laser_pose)
        self.pose_pub = rospy.Publisher(
            'object_pose' + robotnumber, PoseStamped, queue_size=1)

        self.listener = tf.TransformListener()

    def laser_pose(self, data):
        # target-source
        (trans, rot) = self.listener.lookupTransform('thorvald_00' + self.robotnumber +
                                                     '/kinect2_depth_optical_frame', 'thorvald_00' + self.robotnumber + '/base_link', rospy.Time())

        # ranges = data.ranges
        # angle_increment = data.angle_increment
        # angle_min = data.angle_min
        # min_range = min(ranges)
        # i = ranges.index(min_range)
        # angle = angle_min + (i * angle_increment)

        angle = data.angle_min + \
            (data.ranges.index(min(data.ranges)) * data.angle_increment)

        p1 = PoseStamped()
        p1.header.frame_id = "thorvald_00" + self.robotnumber + "/base_link"
        p1.pose.orientation.w = 1
        p1.pose.orientation.z = sin(angle / 2)
        self.pose_pub.publish(p1)


if __name__ == '__main__':
    rospy.init_node('oblect_frame')
    object_frame('1')
    object_frame('2')
    rospy.spin()
