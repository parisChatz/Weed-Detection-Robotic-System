#!/usr/bin/env python

import rospy
from cv2 import namedWindow, cvtColor, imshow, inRange
from cv2 import destroyAllWindows, startWindowThread, bitwise_and
from cv2 import COLOR_BGR2GRAY,COLOR_BGR2HSV, waitKey
import numpy as np
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
                                          Image, self.callback)
        # self.image_sub = rospy.Subscriber(
        #     "/camera/rgb/image_raw",
        #     Image, self.callback)
        self.publisher = rospy.Publisher('/camera_greens', Image, queue_size=10)

    def callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv_image2 = cv_image

        hsv = cvtColor(cv_image, COLOR_BGR2HSV)

        lower_green = np.array([20,0,20])
        upper_green = np.array([100,255,100])
        
        mask = inRange(hsv, lower_green, upper_green)

        res = bitwise_and(cv_image,cv_image, mask= mask)

        imshow('cv_image',cv_image2)
        imshow('res',res)

        self.publisher.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
        waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_converter')
    ic = image_converter()
    rospy.spin()