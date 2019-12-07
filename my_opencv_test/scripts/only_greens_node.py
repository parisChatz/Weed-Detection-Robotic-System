#!/usr/bin/env python

import rospy
import cv2
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread, bitwise_and
from cv2 import COLOR_BGR2GRAY, COLOR_BGR2HSV, waitKey
import numpy as np
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/thorvald_001/kinect2_camera/hd/image_color_rect",
            Image, self.callback)
        # self.image_sub = rospy.Subscriber(`
        #     "/camera/rgb/image_raw",
        #     Image, self.callback)
        self.publisher = rospy.Publisher(
            '/camera_greens', Image, queue_size=10)

    def callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv_image2 = cv_image

        hsv = cvtColor(cv_image, COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
        lower_green = np.array([36, 0, 10])
        upper_green = np.array([90, 255, 255])

        mask = inRange(hsv, lower_green, upper_green)
        res = bitwise_and(cv_image, cv_image, mask=mask)
        res = cv2.GaussianBlur(res, ksize=(17, 17), sigmaX=10)

        low_weed = np.array([30, 30, 0])
        higher_weed = np.array([100, 90, 40])
        mask_weed = inRange(res, low_weed, higher_weed)
        res_weed = bitwise_and(cv_image, cv_image, mask=mask_weed)

        imshow('cv_image', cv_image2)
        imshow('res', res_weed)

        self.publisher.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
        waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_converter')
    ic = image_converter()
    rospy.spin()
