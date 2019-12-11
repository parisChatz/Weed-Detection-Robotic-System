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
from topological_navigation.msg import GotoNodeActionGoal
from std_msgs.msg import String, Bool


class image_converter:
    camera_model = None

    def __init__(self, robot):
        self.pointcloud_list = []
        self.robot = robot
        self.camera_listener = tf.listener.TransformListener()
        self.tf_listener = tf.TransformListener()
        self.points_msg = PointCloud()
        self.complete_bool_msg = Bool()
        self.close_waypoints = []
        self.current_filter_var = None

        self.bridge = CvBridge()
        self.current_waypoint = None

        self.waypoint_sub = rospy.Subscriber(
            "/{}/current_node".format(self.robot), String, self.waypoint_callback)

        self.image_sub = rospy.Subscriber(
            "/{}/kinect2_camera/hd/image_color_rect".format(self.robot),
            Image, self.detection_callback)

        self.camera_info_sub = rospy.Subscriber(
            '/{}/kinect2_camera/hd/camera_info'.format(self.robot),
            CameraInfo,
            self.camera_info_callback)

        self.greens_publisher = rospy.Publisher(
            '/{}/camera_greens'.format(self.robot), Image, queue_size=10)

        self.complete_pointcloud_pub = rospy.Publisher(
            "/{}/weed/points/".format(self.robot),
            PointCloud,
            queue_size=10)

        self.publish_complete_pointcloud = rospy.Publisher(
            "/{}/complete_weed_pointcloud/".format(self.robot),
            PointCloud,
            queue_size=10)

        # 3 topic bool publishers for detection completion
        self.first_row_completion_pub = rospy.Publisher(
            "Line1_complete", Bool, queue_size=10)
        self.second_row_completion_pub = rospy.Publisher(
            "Line2_complete", Bool, queue_size=10)
        self.third_row_completion_pub = rospy.Publisher(
            "Line3_complete", Bool, queue_size=10)

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()  # Only subscribe once

    def waypoint_callback(self, data):
        self.current_waypoint = data

        if self.current_waypoint.data == "WPline1_0":
            self.current_filter_var = self.current_waypoint.data

        if self.current_waypoint.data == "WPline3_0":
            self.current_filter_var = self.current_waypoint.data

        if self.current_waypoint.data == "WPline5_0":
            self.current_filter_var = self.current_waypoint.data

        if self.current_waypoint.data == "WPline2_0":
            self.complete_bool_msg.data = True
            self.first_row_completion_pub.publish(self.complete_bool_msg)

        if self.current_waypoint.data == "WPline4_0":
            self.complete_bool_msg.data = True
            self.second_row_completion_pub.publish(self.complete_bool_msg)

        if self.current_waypoint.data == "WPline6_0":
            self.complete_bool_msg.data = True
            self.third_row_completion_pub.publish(self.complete_bool_msg)

    def mask_function(self):
        # print(self.current_waypoint)
        if self.current_filter_var == "WPline1_0":

            lower_filter = np.array([30, 30, 0])
            upper_filter = np.array([100, 90, 40])
            return upper_filter, lower_filter

        elif self.current_filter_var == "WPline3_0":
            lower_filter = np.array([30, 30, 0])
            upper_filter = np.array([100, 90, 85])
            return upper_filter, lower_filter

        elif self.current_filter_var == "WPline5_0":
            lower_filter = np.array([40, 90, 0])
            upper_filter = np.array([100, 100, 200])
            return upper_filter, lower_filter
        else:
            return np.array([255, 255, 255]), np.array([255, 255, 255])

    def detection_callback(self, data):
        if not self.camera_model:
            return

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv_image2 = cv_image

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv, (40, 40))

        upper_filter, lower_filter = self.mask_function()
        # lower_filter = np.array([30, 30, 0])
        # upper_filter = np.array([100, 90, 40])

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
            # get the resolution of the camera and only add the points in the middle of the camera -+20 pixels
            if point[0] >= self.camera_model.fullResolution()[0] / 2 - 50 and point[0] <= self.camera_model.fullResolution()[0] / 2 + 50:

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
    image_converter("thorvald_002")
    image_converter("thorvald_001")
    rospy.spin()
