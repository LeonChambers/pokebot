#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == '__main__':
    rospy.init_node('task_planning')
    nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    nav_client.wait_for_server()

    goal = MoveBaseGoal(1, 1)
    nav_client.send_goal(goal)
    nav_client.wait_for_result(rospy.Duration.from_secs(10.0))
