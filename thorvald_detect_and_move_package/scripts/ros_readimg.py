#!/usr/bin/env python

# Python libs
import sys
import numpy as np
import math

# OpenCV
import cv2

# Ros libraries
import rospy
import image_geometry
import tf
import tf2_ros

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import PointStamped, Point32
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

# My libs
from canopy import CanopyClass


class image_projection(CanopyClass):
    camera_model = None
    inputimage = None

    def __init__(self, robot, inputimage=None):
        self.inputimage = inputimage
        self.robot = robot
        self.image_pub = rospy.Publisher(
            "/weed/spray/{}".format(self.robot),
            Image,
            queue_size=5)

        self.contour_pub = rospy.Publisher(
            "/weed/point/{}".format(self.robot),
            PointStamped,
            queue_size=5)

        self.weedpoints_pub = rospy.Publisher(
            "/weed/points/{}".format(self.robot),
            PointCloud,
            queue_size=5)

        self.plantpoints_pub = rospy.Publisher(
            "/plant/points/{}".format(self.robot),
            PointCloud,
            queue_size=5)

        self.bridge = CvBridge()

        self.camera_info = rospy.wait_for_message(
            '/%s/kinect2_camera/hd/camera_info' % (self.robot),
            CameraInfo)
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

        rospy.Subscriber(
            "/%s/kinect2_camera/hd/image_color_rect" % (self.robot),
            Image, self.image_callback)

        rospy.Subscriber(
            "/weed/row/{}".format(self.robot),
            String,
            self.changerow_callback)

        self.point_msg = PointStamped()
        self.points_msg = PointCloud()
        self.tflistener = tf.listener.TransformListener()
        self.tf2Buffer = tf2_ros.Buffer()
        self.tflistenerNew = tf2_ros.TransformListener(self.tf2Buffer)

    def changerow_callback(self, data):
        if self.inputimage != data.data:
            print('Changed detection row to: {}'.format(data))
            self.inputimage = data.data

    def publish_points(self, contours, publish):
        ''' Publish the points to the default topic '''
        # print('Found points: {}'.format(len(contours)))
        # time = rospy.Time(0)
        self.points_msg.points = []
        self.points_msg.header.frame_id = self.camera_model.tfFrame()
        self.points_msg.header.stamp = self.last_ts
        for cnt in contours:
            rect = self.camera_model.rectifyPoint(cnt)
            x, y, z = self.camera_model.projectPixelTo3dRay(rect)
            x *= 0.493
            y *= 0.493
            z = 0.493
            if -0.01 <= x <= 0.01:
                self.points_msg.points.append(Point32(x, y, z))
        print('Found filtered points: {}'.format(len(self.points_msg.points)))

        tf_points = self.transformPosition(self.points_msg)
        # tf_points = self.transformPositionBefore(self.points_msg)
        # print(tf_points)
        publish(tf_points)

    def transformPosition(self, points_msg):
        ''' Transform using the tf library.
            The timestamp is given by the image callback.
        '''
        tf_points = self.tflistener.transformPointCloud('map', self.points_msg)
        return tf_points

    def transformPositionBefore(self, points_msg):
        ''' Transform using the tf2 library based on Will's code '''
        trans = self.tf2Buffer.lookup_transform(
            "map",
            self.camera_model.tfFrame(),
            points_msg.header.stamp
        )
        # print(points_msg)
        for point in points_msg.points:
            # A. t1 * rot_matrix:
            t1 = np.array(
                [point.x,
                point.y,
                point.z])
            theta = tf.transformations.euler_from_quaternion(
                [trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w])
            yaw = theta[2]
            rotation_matrix = np.array([
                [(math.cos(yaw)), -(math.sin(yaw)), 0],
                [(math.sin(yaw)), (-math.cos(yaw)), 0],
                [0, 0, 1] ])
            t1_A = np.matmul(t1, rotation_matrix)

            # B. t * output_of_a
            t_x = trans.transform.translation.x + t1_A[0]
            t_y = trans.transform.translation.y + t1_A[1]
            t_z = 0

            point.x = t_x
            point.y = t_y
        return points_msg


    def image_callback(self, data):
        self.last_ts = data.header.stamp
        if not self.camera_model:
            return

        if self.inputimage is None or self.inputimage == '':
            self.image_pub.publish(data)
            return 0

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        ground_inv, ground_inv_mask = self.filter_colors(cv_image, 'ground_inv')

        weed, weed_mask = self.filter_colors(ground_inv, self.inputimage)
        cv_image, contours, contours_boxes, contours_points = self.get_contours(weed, cv_image)
        self.publish_points(contours_points, self.weedpoints_pub.publish)

        plant, plant_mask = self.filter_colors(ground_inv, self.inputimage.replace('_inv', ''))
        cv_image, contours, contours_boxes, contours_points = self.get_contours(plant, cv_image)
        self.publish_points(contours_points, self.plantpoints_pub.publish)

        contours_s = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)

        # Publish new image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(contours_s, encoding="bgr8"))


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    image_projection('thorvald_001', 'simple_inv')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
