#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import actionlib

class move_robot():
    def __init__(self,robotname):
        self.robotname = robotname
        self.client = actionlib.SimpleActionClient('/{}/topological_navigation'.format(self.robotname), GotoNodeAction)
        self.client.wait_for_server()
        self.goal = GotoNodeGoal()

    def move_manuvers(self):

        # send first goal
        self.goal = GotoNodeGoal()
        self.goal.target = "WayPoint1"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() # wait until the action is complete
        result = self.client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)

        # send second goal
        self.goal.target = "WayPoint5"
        # Fill in the goal here
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() # wait until the action is complete
        result = self.client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)


from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    move_robot("thorvald_001").move_manuvers()
