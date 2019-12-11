#! /usr/bin/env python
import rospy
import actionlib

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal


class move_robot():
    def __init__(self):
        self.robotname = robotname
        self.client1 = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client1.wait_for_server()

        self.client2 = actionlib.SimpleActionClient('/thorvald_002/topological_navigation', GotoNodeAction)
        self.client2.wait_for_server()

        self.goal1 = GotoNodeGoal()
        self.goal2 = GotoNodeGoal()
        self.goal1.no_orientation = 1
        self.goal2.no_orientation = 1


    def move_manuvers(self):


        self.goal.target = "WPinit"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPstart"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)
        

        self.goal.target = "WPline1_0"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline1_1"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline2_1"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline2_0"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPstart"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)
        
        self.goal.target = "WPline3_0"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline3_1"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline4_1"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline4_0"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPstart"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status is %s for %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)


        self.goal.target = "WPline5_0"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline5_1"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline6_1"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPline6_0"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        self.goal.target = "WPstart"
        self.client.send_goal(self.goal)
        status = self.client.wait_for_result() 
        result = self.client.get_result()
        rospy.loginfo("status for %s is %s", status, self.goal.target)
        rospy.loginfo("result is %s", result)

        
if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    move_robot().move_manuvers()

    
