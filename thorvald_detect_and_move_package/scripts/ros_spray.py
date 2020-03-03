#!/usr/bin/env python

# Python libs
import sys
import math

# Ros libraries
import rospy
import tf
from std_srvs.srv import Empty

# Ros Messages
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from uol_cmp9767m_base.srv import y_axes_diff, y_axes_diffRequest


class Sprayer():

    def __init__(self, robot):
        self.robot = robot
        self.sprayed = []
        self.spray_srv = rospy.ServiceProxy(
            "{}/dynamic_sprayer".format(self.robot),
            y_axes_diff)

        self.sub = rospy.Subscriber(
            "/weed/allpoints/{}".format(self.robot),
            PointCloud,
            self.spray_weed_callback)

        self.tflistener = tf.listener.TransformListener()

        self.sprayed_points_msg = PointCloud()
        self.sprayed_points_pub = rospy.Publisher(
            "/weed/sprayed_points/{}".format(self.robot),
            PointCloud,
            queue_size=5)


    def spray_weed_callback(self, data):
        time = rospy.Time(0)
        try:
            trans, rot = self.tflistener.lookupTransform(
                'map',
                '{}/sprayer'.format(self.robot),
                data.header.stamp)
            # print(trans, rot)
        except Exception:
            return

        for point in data.points:
            dx = abs(trans[0] - point.x)
            dy = trans[1] - point.y
            if dx < 0.1 and abs(dy) < 0.6:
                if point not in self.sprayed:
                    self.sprayed.append(point)
                    print('spray!!!')
                    # delay
                    req = y_axes_diffRequest()
                    req.y_diff = dy
                    self.spray_srv(req)


        self.sprayed_points_msg.points = self.sprayed
        self.sprayed_points_msg.header.frame_id = 'map'
        self.sprayed_points_msg.header.stamp = time
        self.sprayed_points_pub.publish(self.sprayed_points_msg)    


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('spray_node', anonymous=True)
    Sprayer('thorvald_001')


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
