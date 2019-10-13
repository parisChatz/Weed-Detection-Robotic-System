#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
from math import atan2, pi, cos, sin


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    pose_pub = rospy.Publisher(
        'test_pose', geometry_msgs.msg.PoseStamped, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                'thorvald_002/base_link', 'thorvald_001/base_link', rospy.Time())

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue

        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.frame_id = "thorvald_001/base_link"
        p1.pose.orientation.w = 1  # Neutral orientation
        # half a metre away from the from frame centre
        p1.pose.orientation.z = rot[2]
        pose_pub.publish(p1)
