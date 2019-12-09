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

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv, (40, 40))
        # hsv = cv2.GaussianBlur(hsv, ksize=(17,17), sigmaX=10)
        lower_filter = np.array([30, 30, 10])
        upper_filter = np.array([100, 90, 40])

        mask = inRange(hsv, lower_filter, upper_filter)
        res = bitwise_and(cv_image, cv_image, mask=mask)

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
        
        # cv2.imshow('Canny Edges After Contouring', edged)
        cv2.drawContours(res, contours, -1, (0, 255, 0), 1)

        threshold_area = 350
        overlapping_rectangles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > threshold_area:
                x,y,w,h = cv2.boundingRect(cnt)
                res = cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),3)
                overlapping_rectangles.append([x,y,w,h])
                overlapping_rectangles.append([x,y,w,h])



        overlapping_rectangles , weights = cv2.groupRectangles(overlapping_rectangles,1,0.3)

        for rectangle in overlapping_rectangles:
            x,y,w,h = rectangle
            res2 = cv2.rectangle(res,(x,y),(x+w,y+h),(0,0,255),2)

        # imshow('res2', res2)
        
        self.publisher.publish(self.bridge.cv2_to_imgmsg(res2, "bgr8"))
        waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_converter')
    ic = image_converter()
    rospy.spin()
