#!/usr/bin/env python

import rospy
import tf2_ros as tf
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class TaskPlanner:
    def __init__(self):
        self.nav_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction
        )
        self.nav_client.wait_for_server()
        self.tb = tf.Buffer()
        tl = tf.TransformListener(self.tb)

    def process_waypoints(self, *points):
        """
        Takes in a list of names of waypoint to move through and moves the base to
        those points in order
        """
        for point in points:
            # Get the location of the point
            t = self.tb.lookup_transform(
                "map", point, rospy.Time(0), rospy.Duration(1)
            )
            self.move_to_point(t.transform.translation, t.transform.rotation)

    def move_to_point(self, pos, ori):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = pos
        goal.target_pose.pose.orientation = ori
        self.nav_client.send_goal(goal)
        while not self.nav_client.wait_for_result(rospy.Duration.from_sec(2.0)):
            print "Waiting..."

if __name__ == '__main__':
    rospy.init_node('task_planning')
    planner = TaskPlanner()
    planner.process_waypoints("waypoint_1_0", "waypoint_1_1", "waypoint_1_shelf")
