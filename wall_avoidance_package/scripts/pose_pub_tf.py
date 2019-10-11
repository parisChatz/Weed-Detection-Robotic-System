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
        yaw_angle = atan2(rot[2], rot[3]) * 2
        #print("the current transformation: ", trans, yaw_angle * 180 / pi)

        # here is an exmaple pose, with a given frame of reference, e.g. somethng detected in the camera
        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.frame_id = "thorvald_002/kinect2_rgb_optical_frame"
        p1.pose.position.x = trans[0]
        p1.pose.position.y = trans[1]
        p1.pose.position.z = trans[2]
        p1.pose.orientation.w = rot[3]  # Neutral orientation
        # half a metre away from the from frame centre
        p1.pose.orientation.z = rot[2]
        # we publish this so we can see it in rviz:
        pose_pub.publish(p1)
