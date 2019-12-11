#!/usr/bin/env python
# Ros libraries
import rospy
import tf

# Ros Messages
from std_srvs.srv import Empty
from std_msgs.msg import Bool, String
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import CameraInfo, PointCloud

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal


class NavigateClass():

    def __init__(self, robot):
        self.robot = robot
        self.safe_nodes = ["WPline5_0", "WPline3_0", "WPline1_0"]
        self.danger_nodes = ["WPline6_0", "WPline4_0", "WPline2_0"]
        self.other_nodes = ["WPline5_1", "WPline3_1",
                            "WPline1_1", "WPline6_1", "WPline4_1", "WPline2_1"]
        self.ocupied = {"Line1": False, "Line2": False, "Line3": False}
        self.done = {"Line1": False, "Line2": False, "Line3": False}
        self.client = actionlib.SimpleActionClient(
            '/{}/topological_navigation'.format(self.robot), GotoNodeAction)
        self.iddle_robots = {"thorvald_001": False, "thorvald_002": False}
        self.client.wait_for_server()
        self.goal = GotoNodeGoal()

        self.sprayer = rospy.ServiceProxy('thorvald_002/spray', Empty)

        self.choose_line_sub = rospy.Subscriber(
            "/{}/current_node".format(self.robot), String, self.choose_line_callback)

        self.line1sub = rospy.Subscriber(
            "/Line1_complete", Bool, self.line1callback)
        self.line2sub = rospy.Subscriber(
            "/Line2_complete", Bool, self.line2callback)
        self.line3sub = rospy.Subscriber(
            "/Line3_complete", Bool, self.line3callback)

        self.current_robot_node1 = rospy.Subscriber(
            "/thorvald_001/current_node", String, self.robot1positioncallback)
        self.current_robot_node2 = rospy.Subscriber(
            "/thorvald_002/current_node", String, self.robot2positioncallback)

        self.iddle_robot1 = rospy.Subscriber(
            "/thorvald_001/topological_navigation/status", GoalStatusArray, self.iddle1positioncallback)

        self.iddle_robot2 = rospy.Subscriber(
            "/thorvald_002/topological_navigation/status", GoalStatusArray, self.iddle2positioncallback)

    def iddle1positioncallback(self, data):
        if len(data.status_list) != 0:
            self.iddle_robots["thorvald_001"] = True
        else:
            self.iddle_robots["thorvald_001"] = False

    def iddle2positioncallback(self, data):
        if len(data.status_list) != 0:
            self.iddle_robots["thorvald_002"] = True
        else:
            self.iddle_robots["thorvald_002"] = False

    def robot1positioncallback(self, data):
        self.robot1position = data.data
        self.robot1name = "/thorvald_001/"

    def robot2positioncallback(self, data):
        self.robot2position = data.data
        self.robot2name = "/thorvald_002/"

    def line1callback(self, data):
        if data.data in self.safe_nodes:
            self.done["Line1"] = True

    def line2callback(self, data):
        if data.data in self.safe_nodes:
            self.done["Line2"] = True

    def line3callback(self, data):
        if data.data in self.safe_nodes:
            self.done["Line3"] = True

    def choose_line_callback(self, data):
        if data.data in self.safe_nodes:
            if data.data == self.safe_nodes[2]:
                self.done["Line1"] = True
            if data.data == self.safe_nodes[1]:
                self.done["Line2"] = True
            if data.data == self.safe_nodes[0]:
                self.done["Line3"] = True
            for line, status in self.done.items():
                print(line,status,self.iddle_robots)
                if status is False:
                    print(status)

if __name__ == '__main__':
    rospy.init_node('navig_and_spray', anonymous=True)
    NavigateClass("thorvald_001")
    rospy.spin()
