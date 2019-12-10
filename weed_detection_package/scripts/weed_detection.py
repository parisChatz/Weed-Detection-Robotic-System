#!/usr/bin/env python

# Python libs
import numpy as np
import math

# OpenCV
import cv2
from cv_bridge import CvBridge

# Ros libraries
import rospy
import image_geometry
import tf
import actionlib

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import Point32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def movebase_client(x, y, z):  # CHANGE TO TOPOLOGICAL NAVIGATION
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


class image_converter:
    camera_model = None

    def __init__(self, robot):
        self.pointcloud_list = []
        self.robot = robot
        self.camera_listener = tf.listener.TransformListener()
        self.tf_listener = tf.TransformListener()
        self.points_msg = PointCloud()
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            "/{}/kinect2_camera/hd/image_color_rect".format(self.robot),
            Image, self.detection_callback)

        self.camera_info_sub = rospy.Subscriber(
            '/{}/kinect2_camera/hd/camera_info'.format(self.robot),
            CameraInfo,
            self.camera_info_callback)

        self.greens_publisher = rospy.Publisher(
            '/camera_greens', Image, queue_size=10)

        self.complete_pointcloud_pub = rospy.Publisher(
            "/{}/weed/points/".format(self.robot),
            PointCloud,
            queue_size=10)

        self.publish_complete_pointcloud = rospy.Publisher(
            "/{}/complete_weed_pointcloud/".format(self.robot),
            PointCloud,
            queue_size=10)

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()  # Only subscribe once

    def detection_callback(self, data):
        if not self.camera_model:
            return

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv_image2 = cv_image

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv, (40, 40))
        lower_filter = np.array([30, 30, 0])
        upper_filter = np.array([100, 90, 40])

        mask = cv2.inRange(hsv, lower_filter, upper_filter)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        gray_res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        ret, edged = cv2.threshold(gray_res, 20, 255, 0)
        im2, contours, hierarchy = cv2.findContours(
            edged,
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # draw edges
        cv2.drawContours(res, contours, -1, (0, 255, 0), 1)

        # Filter small detections
        threshold_area = 400
        overlapping_rectangles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > threshold_area:
                # CHECK ALSO THE SIDES THE WEEDS ARE MOSTLY RECTANGULAR!!!!!!
                x, y, w, h = cv2.boundingRect(cnt)
                res = cv2.rectangle(
                    res, (x, y), (x + w, y + h), (0, 255, 0), 3)
                overlapping_rectangles.append([x, y, w, h])
                overlapping_rectangles.append([x, y, w, h])

        overlapping_rectangles, weights = cv2.groupRectangles(
            overlapping_rectangles, 1, 0.1)

        # Find middle points of weeds
        middle_points = []
        res2 = None
        for rectangle in overlapping_rectangles:
            x, y, w, h = rectangle
            middle = (x + w / 2, y + h / 2)
            middle_points.append(middle)
            res2 = cv2.rectangle(
                cv_image2, (x, y), (x + w, y + h), (0, 0, 255), 2)
            res2 = cv2.circle(res2, middle, 5, (255, 255, 255), -1)

        if res2 is None:
            res2 = cv_image2

        # Find middle points
        self.local_pointcloud_callback(middle_points)
        self.greens_publisher.publish(self.bridge.cv2_to_imgmsg(res2, "bgr8"))
        cv2.waitKey(1)

    def local_pointcloud_callback(self, middle_points):
        time = rospy.Time(0)
        self.points_msg.points = []
        self.points_msg.header.frame_id = self.camera_model.tfFrame()
        self.points_msg.header.stamp = time
        for point in middle_points:
            if point[0] >= self.camera_model.fullResolution()[0]/2-20 and point[0] >= self.camera_model.fullResolution()[0]/2+20:

                u = point[0]  # x pixel
                v = point[1]  # y pixel
                # project a point in camera coordinates into the pixel coordinates
                uv = self.camera_model.projectPixelTo3dRay(
                    self.camera_model.rectifyPoint((u, v)))
                self.points_msg.points.append(
                    Point32(uv[0] * 0.493, uv[1] * 0.493, 0.493))

        tf_points = self.camera_listener.transformPointCloud(
            'map', self.points_msg)
        self.complete_pointcloud_pub.publish(tf_points)
        self.global_pointcloud_callback(tf_points)

    def global_pointcloud_callback(self, pointcloud):
        for point in pointcloud.points:
            found = False
            for keep in self.pointcloud_list:
                dx = abs(point.x - keep.x)
                dy = abs(point.y - keep.y)
                dist = math.hypot(dx, dy)
                if dist < 0.07:
                    found = True

            if not found:
                self.pointcloud_list.append(point)

        self.points_msg.points = self.pointcloud_list
        self.points_msg.header.frame_id = 'map'
        self.points_msg.header.stamp = rospy.Time()
        self.publish_complete_pointcloud.publish(self.points_msg)


if __name__ == '__main__':
    rospy.init_node('image_converter')
    ic = image_converter("thorvald_001")

    movebase_client(6, -3.8, 90)
    movebase_client(-6, -3.8, 90)
    movebase_client(-6, -2.7, 0)
    movebase_client(6, -2.7, 0)
    movebase_client(6, -0.7, 90)
    movebase_client(-6, -0.7, 90)
    movebase_client(-6, 0.2, 0)
    movebase_client(6, 0.2, 0)
    movebase_client(6, 2.2, 90)
    movebase_client(-6, 2.2, 90)
    movebase_client(-6, 3.2, 0)
    movebase_client(6, 3.2, 0)

    rospy.spin()
