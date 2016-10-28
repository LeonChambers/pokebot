#!/usr/bin/env python

import rospy

from me212base.srv import Set2DPose

class PathPlanner:
    def __init__(self):
        self.pose = None
        self.pose_sub = rospy.Subscriber(
            "base_pose/dead_reckoning", Pose2D, self.pose_callback
        )
        self.service = rospy.Service(
            "navigate_base_to_position", Set2DPose,
            self.navigate_base_to_position
        )

    def pose_callback(self, pose):
        self.pose = pose

    def navigate_base_to_position(self, pose):
        pass

    def move_base_to_position(self, pose):


if __name__ == '__main__':
    rospy.init_node("path_planner")
    p = PathPlanner()
    rospy.spin()
