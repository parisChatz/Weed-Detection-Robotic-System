#!/usr/bin/env python

# Python libs
import sys
import threading

# Ros libraries
import rospy

# Ros Messages
from std_msgs.msg import String

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveRobot():

    def __init__(self, robot, inputimage=None):
        self.inputimage = inputimage
        self.robot = robot
        self.changerow_pub = rospy.Publisher(
            "/weed/row/{}".format(self.robot),
            String,
            queue_size=10)
        self.changerow_msg = String()

        threading.Thread(target=self.row_thread, args=()).start()

    def changerow(self, row):
        self.changerow_msg.data = row

    def row_thread(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.changerow_pub.publish(self.changerow_msg)
            rate.sleep()

    def movebase_client(self, x, y, z):
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


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('move_robot', anonymous=True)
    mv_robot = MoveRobot('thorvald_001')
    # MoveRobot('thorvald_002')

    mv_robot.changerow('simple_inv')
    mv_robot.movebase_client(-6, -3, 90)
    mv_robot.movebase_client(6, -3, 90)
    mv_robot.movebase_client(6, -2, 0)
    mv_robot.movebase_client(-6, -2, 0)
    mv_robot.changerow('')

    mv_robot.changerow('realeasy_inv')
    mv_robot.movebase_client(6, -0.7, 90)
    mv_robot.movebase_client(-6, -0.7, 90)
    mv_robot.movebase_client(-6, 0.2, 0)
    mv_robot.movebase_client(6, 0.2, 0)
    mv_robot.changerow('')

    mv_robot.changerow('realhard_inv')
    mv_robot.movebase_client(6, 2.2, 90)
    mv_robot.movebase_client(-6, 2.2, 90)
    mv_robot.movebase_client(-6, 3.2, 0)
    mv_robot.movebase_client(6, 3.2, 0)
    mv_robot.changerow('')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
