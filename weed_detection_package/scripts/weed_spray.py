#!/usr/bin/env python

# Python libs
import sys
import math

# Ros libraries
import rospy
import tf
import rosservice


# Ros Messages
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry



class sprayerClass():

    def __init__(self, robot):
        self.robot = robot
        self.sprayedlist = []
        self.sprayer = rospy.ServiceProxy("thorvald_001/spray", Empty)

        rospy.Subscriber("/{}/complete_weed_pointcloud/".format(self.robot),PointCloud,
                         self.spray_callback)

        self.sprayed_points_pub = rospy.Publisher(
            "/weed/sprayedpoints/{}".format(self.robot),
            PointCloud,
            queue_size=5)

        self.detected_not_sprayed_pub = rospy.Publisher(
            "/{}/detected_not_sprayed_points/".format(self.robot),
            PointCloud,
            queue_size=5)

        self.points_msg = PointCloud()

        self.tflistener = tf.listener.TransformListener()

    def publish_allpoints(self):
        time = rospy.Time(0)
        self.points_msg.points = self.keeplist
        self.points_msg.header.frame_id = 'map'
        self.points_msg.header.stamp = time

        self.points_pub.publish(self.points_msg)

    def spray_callback(self, data):
        try:
            trans, rot = self.tflistener.lookupTransform(
                'map',
                '{}/sprayer'.format(self.robot),
                data.header.stamp)
        except Exception:
            return
    
        newkeep = []
        shouldSpray = False
        for point in data.points:

            dx = abs(point.x - trans[0])
            dy = abs(point.y - trans[1])
            dist = math.hypot(dx, dy)
            if dx <0.6 and dy < 0.1:
                if point not in self.sprayedlist:
                    print(point, trans)
                    print(dx, dy)
                    print('spray')
                    self.sprayedlist.append(point)
                    self.sprayer()

        time = rospy.Time(0)
        self.points_msg.points = self.sprayedlist
        self.points_msg.header.frame_id = "map"
        self.points_msg.header.stamp = time
        self.detected_not_sprayed_pub.publish(self.points_msg)



def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('weed_spray', anonymous=True)
    sprayerClass('thorvald_002')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
