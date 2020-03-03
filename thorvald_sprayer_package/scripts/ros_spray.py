#!/usr/bin/env python

import math
import rospy
import tf
# Ros Messages
from sensor_msgs.msg import PointCloud
from uol_cmp9767m_base.srv import y_axes_diff, y_axes_diffRequest
from geometry_msgs.msg import Point32, PointStamped


class Sprayer():

    def __init__(self, robot):
        self.robot = robot
        self.sprayed = []  # Keep a list of sprayed points for rviz
        self.real_sprayed = []
        self.real_sprayed2 = []

        self.last_spray_pos = 0
        self.y_previous = 0
        self.radius = 0.5  # killbox radius

        self.spray_srv = rospy.ServiceProxy(
            "{}/dynamic_sprayer".format(self.robot),
            y_axes_diff)

        self.sub = rospy.Subscriber(
            "thorvald_001/complete_weed_pointcloud",
            PointCloud,
            self.spray_weed_callback)

        self.tflistener = tf.listener.TransformListener()

        self.sprayed_points_msg = PointCloud()
        self.sprayed_points_pub = rospy.Publisher(
            "{}/weed/sprayed_points/".format(self.robot),
            PointCloud,
            queue_size=5)

        self.sprayed_points_indirect_msg = PointCloud()
        self.sprayed_points_indirect_pub = rospy.Publisher(
            "{}/weed/sprayed_indirect_points/".format(self.robot),
            PointCloud,
            queue_size=5)

    def slep(self, sprayer_distance):

        total_travel_speed = 15
        slep_time = sprayer_distance / total_travel_speed
        rospy.sleep(slep_time)

    def spray_weed_callback(self, data):
        time = rospy.Time(0)
        # Get the sprayer position in the map coordinates
        try:
            trans, rot = self.tflistener.lookupTransform(
                'map',
                '{}/sprayer'.format(self.robot),
                data.header.stamp)
        except Exception:
            return

        # First initialise current_y_sprayer = result from tf
        # it's going to be updated according to the place it travels to.

        # Iterate every detected Weed
        for point in data.points:
            # If you see any weeds in current crop line
            # (crop line has a width of 1 meter thats why <0.5)
            if abs(trans[1] - point.y) < 0.5:
                # pos of robot
                x_sprayer = trans[0]
                current_y_sprayer = trans[1] + self.y_previous

                # Difference in 'x' and 'y' frame
                dx = abs(x_sprayer - point.x)

                # this creates mirror effect
                dy = current_y_sprayer - point.y

                # When sprayer moves, sleep for travel time
                # slep(dy)

                if dx < self.radius:  # Same as killbox radius
                    if point not in self.sprayed:

                        new_point = PointStamped()
                        new_point.header.frame_id = "map"
                        new_point.header.stamp = rospy.Time()
                        new_point.point.x = point.x
                        new_point.point.y = point.y
                        p = self.tflistener.transformPoint(
                            "{}/sprayer".format(self.robot), new_point)

                        # Initialise request and assign to it the difference in y axis
                        # between the sprayer and the weed
                        req = y_axes_diffRequest()

                        # req has to be the difference from center of sprayer
                        # and weed detected IN REFERENCE TO THE SPRAYER!
                        req.y_diff = p.point.y

                        new_point2 = PointStamped()
                        new_point2.header.frame_id = "{}/sprayer".format(
                            self.robot)
                        new_point2.header.stamp = rospy.Time()
                        new_point2.point.x = p.point.x
                        new_point2.point.y = p.point.y
                        rviz_p = self.tflistener.transformPoint(
                            "map", new_point2)

                        # Call service to spray
                        self.spray_srv(req)

                        # save the position of the point (visualise in rviz)
                        real_point = Point32(
                            x_sprayer, rviz_p.point.y, point.z)

                        # Save point for visualisation
                        self.real_sprayed.append(real_point)

            self.sprayed_points_msg.points = self.real_sprayed
            self.sprayed_points_msg.header.frame_id = 'map'
            self.sprayed_points_msg.header.stamp = time
            self.sprayed_points_pub.publish(self.sprayed_points_msg)


if __name__ == '__main__':
    rospy.init_node('spray_node', anonymous=True)
    Sprayer('thorvald_002')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"