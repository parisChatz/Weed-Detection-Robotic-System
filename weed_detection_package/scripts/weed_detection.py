#!/usr/bin/env python

# Python libs
import sys, time
import numpy as np

# OpenCV
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo,PointCloud
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped,PoseStamped,Point32
from cv_bridge import CvBridge, CvBridgeError

def movebase_client(x, y, z): #CHANGE TO TOPOLOGICAL NAVIGATION
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
        print('Done moving to: {}, {}, {}'.format(x, y, z))
        return client.get_result()


class image_converter:
    camera_model = None

    def __init__(self, robot):
        self.robot = robot
        self.camera_listener = tf.listener.TransformListener()
        self.tf_listener = tf.TransformListener()

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            "/{}/kinect2_camera/hd/image_color_rect".format(self.robot),
            Image, self.callback)


        self.camera_info_sub = rospy.Subscriber(
            '/{}/kinect2_camera/hd/camera_info'.format(self.robot),
            CameraInfo,
            self.camera_info_callback)

        self.greens_publisher = rospy.Publisher(
            '/camera_greens', Image, queue_size=10)

        self.contours_pub = rospy.Publisher(
            "{}/weed_poses/".format(self.robot),
            PointStamped,queue_size=10)

        self.camera_info_sub = rospy.Subscriber(
            '/{}/kinect2_camera/hd/camera_info'.format(self.robot),
            CameraInfo,self.camera_info_callback)

        self.contours_pub = rospy.Publisher(
            "/weed/points/{}".format(self.robot),
            PointCloud,
            queue_size=10)
        
        self.posepub = rospy.Publisher("weed_pose",PoseStamped,queue_size=1)
    
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def callback(self, data):
        if not self.camera_model:
            return

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv_image2 = cv_image

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv, (40, 40))
        # hsv = cv2.GaussianBlur(hsv, ksize=(17,17), sigmaX=10)
        lower_filter = np.array([30, 30, 20])
        upper_filter = np.array([100, 90, 45])

        mask = cv2.inRange(hsv, lower_filter, upper_filter)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Grayscale
        gray_res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        # Find Canny edges
        ret, edged = cv2.threshold(gray_res, 20, 255, 0)
        # Finding Contours
        im2, contours, hierarchy = cv2.findContours(
            edged,
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cv2.drawContours(res, contours, -1, (0, 255, 0), 1)

        #Filter small detections
        threshold_area = 400
        overlapping_rectangles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > threshold_area:
                #CHECK ALSO THE SIDES THE WEEDS ARE MOSTLY RECTANGULAR!!!!!!
                x,y,w,h = cv2.boundingRect(cnt)
                res = cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),3)
                overlapping_rectangles.append([x,y,w,h])
                overlapping_rectangles.append([x,y,w,h])

        overlapping_rectangles , weights = cv2.groupRectangles(overlapping_rectangles,1,0.7)

        middle_points = []
        res2 = None
        for rectangle in overlapping_rectangles:
            x,y,w,h = rectangle
            middle = (x+w/2,y+h/2)
            middle_points.append(middle)
            res2 = cv2.rectangle(cv_image2,(x,y),(x+w,y+h),(0,0,255),2)
            res2 = cv2.circle(res2,middle,5,(255,255,255),-1)
        
        if res2 is None:
            res2 = cv_image2

        self.publish_points(middle_points)
        # self.publish_contours(middle_points)

        self.greens_publisher.publish(self.bridge.cv2_to_imgmsg(res2, "bgr8"))
        cv2.waitKey(1)

    def publish_points(self, middle_points):

        time = rospy.Time(0)
        points_msg = PointCloud()
        points_msg.points = []
        points_msg.header.frame_id = self.camera_model.tfFrame()
        points_msg.header.stamp = time
        for point in middle_points:
            u = point[0]    #x pixel
            v = point[1]    #y pixel
            #project a point in camera coordinates into the pixel coordinates
            uv = self.camera_model.projectPixelTo3dRay( self.camera_model.rectifyPoint((u,v)))
            points_msg.points.append(Point32(uv[0]*0.5,uv[1]*0.5,0.5))
            


        tf_points = self.camera_listener.transformPointCloud('map', points_msg)
        self.contours_pub.publish(tf_points)

        # print(points_msg.points)
        print 'Pixel in relation to map: ', tf_points.points
        print ''

    def publish_contours(self, middle_points):
        for point in middle_points: #get weed pixels
            u = point[0]    #x pixel
            v = point[1]    #y pixel

            #project a point in camera coordinates into the pixel coordinates
            uv = self.camera_model.projectPixelTo3dRay( self.camera_model.rectifyPoint((u,v)))

            point_msg = PoseStamped()
            point_msg.header.frame_id = self.camera_model.tfFrame()
            point_msg.pose.position.x = uv[0] 
            point_msg.pose.position.y = uv[1]
            point_msg.pose.position.z = 0.5
            point_msg.pose.orientation.x = 0
            point_msg.pose.orientation.y = 0
            point_msg.pose.orientation.z = 0
            point_msg.pose.orientation.w = 1

            point_msg.header.stamp = rospy.Time()

            tf_point = self.camera_listener.transformPose("thorvald_001/base_link", point_msg)
            
            tf_point_map = self.camera_listener.transformPose("map", point_msg)

            self.posepub.publish(tf_point_map)

            # print 'Pixel in relateion to camera: ', uv[0],uv[1]
            # print 'Pixel in relateion to base_link: ', tf_point.pose.position.x,tf_point.pose.position.y
            print 'Pixel in relateion to map: ', tf_point_map.pose.position.x,tf_point_map.pose.position.y
            print ''

            





    # def publish_contours(self, middle_points):
    #     for point in middle_points:
    #         u = point[0]
    #         v = point[1]
    #         p = [u,v]

            
    #         time = rospy.Time()
            
    #         uv_rect = self.camera_model.rectifyPoint((u, v))
    #         camera_point = self.camera_model.projectPixelTo3dRay(uv_rect)
            # point_msg = geometry_msgs.msg.PoseStamped()
            # point_msg.header.frame_id = self.camera_model.tfFrame()
            # point_msg.pose.position.x = camera_point[0] 
            # point_msg.pose.position.y = camera_point[1]
            # # point_msg.pose.position.z = 1
    #         point_msg.pose.orientation.w = 1
    #         point_msg.header.stamp = time
    #         # print("point_msg: ",point_msg.pose.position.x,point_msg.pose.position.y)
            
    #         (trans, rot) = self.tf_listener.lookupTransform(  'thorvald_001/kinect2_rgb_optical_frame','map', rospy.Time())
    #         # print 'cam to base transform:', 'T ', trans, 'R ', rot

    #         k = self.camera_model.intrinsicMatrix()
    #         inv_k = np.linalg.inv(k)
    #         r = np.dot(inv_k,p)
    #         # print r,camera_point            
    #         # self.posepub.publish(point_msg)
    #         tf_point = self.camera_listener.transformPose('map', point_msg)
    #         print('position of weed pose: ',tf_point)
    #         # self.camera_listener.lookupTransform('thorvald_001/kinect2_rgb_optical_frame','/map', rospy.Time())
    #         # # self.listener.lookupTransform(self.camera_model.tfFrame(), 'map', time)
    #         # tf_point = self.camera_listener.transformPoint('/map', point_msg)
    #         # print(tf_point.point.x,tf_point.point.y)
    #         # self.contours_pub.publish(tf_point)
        
    
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
