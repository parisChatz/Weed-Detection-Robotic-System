#!/usr/bin/env python

# Python libs
import sys, time

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry,tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

class image_projection:
    camera_model = None

    def __init__(self):    

        # -5.0 -0.06

        self.bridge = CvBridge()
        self.camera_listener = tf.listener.TransformListener()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        self.point_pub = rospy.Publisher("point_topic", PoseStamped,queue_size=1)


        rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",
            Image, self.image_callback)

    def image_callback(self, data):
        if not self.camera_model:
            return

        x = 1200 #>900 camera y positive
        y = 300 #>500 camera x positive

        #project a point in camera coordinates into the pixel coordinates
        uv = self.camera_model.projectPixelTo3dRay( self.camera_model.rectifyPoint((x,y)))

        point_msg = PoseStamped()
        point_msg.header.frame_id = self.camera_model.tfFrame()
        point_msg.pose.position.x = uv[0]*0.493
        point_msg.pose.position.y = uv[1]*0.493
        point_msg.pose.position.z = 0.493
        point_msg.pose.orientation.x = 0
        point_msg.pose.orientation.y = 0
        point_msg.pose.orientation.z = 0
        point_msg.pose.orientation.w = 1

        point_msg.header.stamp = rospy.Time()

        tf_point = self.camera_listener.transformPose("thorvald_001/base_link", point_msg)
        
        tf_point_map = self.camera_listener.transformPose("map", point_msg)

        self.point_pub.publish(tf_point)

        print 'Pixel in relateion to camera: ', uv[0]*0.493,uv[1]*0.493
        print 'Pixel in relateion to base_link: ', tf_point.pose.position.x,tf_point.pose.position.y
        print 'Pixel in relateion to map: ', tf_point_map.pose.position.x,tf_point_map.pose.position.y
        print ''

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.circle(cv_image, (x,y), 10, 255, -1)

        #resize for visualisation
        cv_image_s = cv2.resize(cv_image, (0,0), fx=0.5, fy=0.5)

        cv2.imshow("Image window", cv_image_s)
        cv2.waitKey(1)

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_projection', anonymous=True)
    ic = image_projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
