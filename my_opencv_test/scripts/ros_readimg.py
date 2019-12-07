#!/usr/bin/env python

# Python libs
import sys
import time

# OpenCV
import cv2
from cv2 import imshow

# Ros libraries
import roslib
import rospy
import image_geometry

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import copy

import time
import numpy as np

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(x, y, z):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
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
        print('Done moving to: {}, {}, {}'.format(x, y, z))
        # time.sleep(3)
        return client.get_result()

class image_projection:
    camera_model = None
    inputimage = None

    def __init__(self, robot, inputimage=None):
        self.inputimage = inputimage
        self.robot = robot
        self.image_pub = rospy.Publisher(
            "/opencv/image_raw/{}".format(self.robot),
            Image,
            queue_size=5)

        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber(
            '/%s/kinect2_camera/hd/camera_info' % (self.robot),
            CameraInfo,
            self.camera_info_callback)

        rospy.Subscriber(
            "/%s/kinect2_camera/hd/image_color_rect" % (self.robot),
            Image, self.image_callback)

    def filter_colors(self, cv_image, runtype):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if runtype == 'simple':
            # hsv = cv2.blur(hsv, (10, 10))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([30, 120, 0])
            upper_filter = np.array([50, 180, 200])
        if runtype == 'simple_inv':
            # hsv = cv2.blur(hsv, (10, 10))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([0, 0, 0])
            upper_filter = np.array([255, 80, 255])
        elif runtype == 'realeasy':
            hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([0, 10, 40])
            upper_filter = np.array([60, 150, 255])
        elif runtype == 'realeasy_inv':
            # hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([30, 30, 0])
            upper_filter = np.array([100, 90, 40])
        elif runtype == 'realhard':
            hsv = cv2.blur(hsv, (40, 40))
            # hsv = cv2.GaussianBlur(hsv, ksize=(17,17), sigmaX=10)
            lower_filter = np.array([40, 40, 0])
            upper_filter = np.array([100, 80, 200])
        elif runtype == 'realhard_inv':
            hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17,17), sigmaX=10)
            lower_filter = np.array([0, 90, 0])
            upper_filter = np.array([255, 100, 255])
        elif runtype == 'ground':
            # hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([0, 30, 30])
            upper_filter = np.array([20, 140, 80])
        elif runtype == 'ground_inv':
            # hsv = cv2.blur(hsv, (40, 40))
            hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
            lower_filter = np.array([30, 0, 10])
            upper_filter = np.array([90, 255, 150])

        mask = cv2.inRange(hsv, lower_filter, upper_filter)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        # cv2.imshow('hsv', hsv)
        return res, mask

    def get_contours(self, res, cv_image):
        # Grayscale
        gray_res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Canny Edges After Contouring', gray_res)

        # Find Canny edges
        # edged = cv2.Canny(gray_res, 150, 150)
        ret, edged = cv2.threshold(gray_res, 20, 255, 0)

        # Finding Contours
        im2, contours, hierarchy = cv2.findContours(
            edged,
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contours = max(contours, key=cv2.contourArea)

        threshold_area = 300
        filtered_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > threshold_area:
                filtered_contours.append(cnt)
        # cv2.imshow('Canny Edges After Contouring', edged)
        contours = copy.copy(cv_image)
        cv2.drawContours(contours, filtered_contours, -1, (0, 255, 0), 1)
        return contours, filtered_contours

    def image_callback(self, data):
        if not self.camera_model:
            return

        if self.inputimage is None:
            self.image_pub.publish(data)
            return 0

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


        # ground, ground_mask = self.filter_colors(cv_image, 'ground')
        ground_inv, ground_inv_mask = self.filter_colors(cv_image, 'ground_inv')
        plant, plant_mask = self.filter_colors(ground_inv, self.inputimage)
        contours, filtered_contours = self.get_contours(plant, cv_image)
        contours_s = cv2.resize(contours, (0, 0), fx=0.5, fy=0.5)


        # Publish new image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(contours_s, encoding="bgr8"))

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister()  # Only subscribe once


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    img_proj = image_projection('thorvald_001')
    # image_projection('thorvald_002')

    img_proj.inputimage = 'simple_inv'
    movebase_client(6, -3.8, 90)
    movebase_client(-6, -3.8, 90)
    movebase_client(-6, -2.7, 0)
    movebase_client(6, -2.7, 0)
    img_proj.inputimage = None

    img_proj.inputimage = 'realeasy_inv'
    movebase_client(6, -0.7, 90)
    movebase_client(-6, -0.7, 90)
    movebase_client(-6, 0.2, 0)
    movebase_client(6, 0.2, 0)
    img_proj.inputimage = None

    img_proj.inputimage = 'realhard_inv'
    movebase_client(6, 2.2, 90)
    movebase_client(-6, 2.2, 90)
    movebase_client(-6, 3.2, 0)
    movebase_client(6, 3.2, 0)
    img_proj.inputimage = None

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
