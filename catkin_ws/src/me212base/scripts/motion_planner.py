#!/usr/bin/env python

import math
import rospy

from geometry_msgs.msg import Pose2D
from me212base.msg import WheelVelCmd
from me212base.srv import Set2DPose

class MotionPlanner:
    def __init__(self):
        self.pose = None
        self.pose_sub = rospy.Subscriber(
            "base_pose/dead_reckoning", Pose2D, self.pose_callback
        )
        self.vel_pub = rospy.Publisher(
            "velocity_command", WheelVelCmd, queue_size=10
        )
        self.service = rospy.Service(
            "move_base_to_position", Set2DPose, self.move_base_to_position
        )

    def pose_callback(self, pose):
        self.pose = pose

    def move_base_to_position(self, req):
        target = req.pose
        curr_pose = self.pose
        error = self.pose_delta(curr_pose, target)
        while self.pose_distance(error) > 0.01:
            target_heading = math.atan2(
                error.y, error.x
            )
            dtheta = target_heading - curr_pose.theta
            dtheta = dtheta % (2 * math.pi)
            if dtheta > math.pi:
                dtheta -= (2 * math.pi)
            if abs(dtheta) > 0.01:
                self.turn(dtheta)
            else:
                self.forward(self.pose_distance(error))
            rospy.sleep(0.01)
            curr_pose = self.pose
            error = self.pose_delta(curr_pose, target)
        self.stop()
        return True

    def pose_delta(self, pose1, pose2):
        return Pose2D(
            pose2.x - pose1.x,
            pose2.y - pose1.y,
            pose2.theta - pose1.theta
        )

    def pose_distance(self, pose):
        return math.sqrt(pose.x**2 + pose.y**2)

    def turn(self, dtheta):
        if dtheta > 0:
            self.vel_pub.publish(WheelVelCmd(0.1, -0.1))
        else:
            self.vel_pub.publish(WheelVelCmd(-0.1, 0.1))

    def forward(self, dpose):
        self.vel_pub.publish(WheelVelCmd(0.4, 0.4))

    def stop(self):
        self.vel_pub.publish(WheelVelCmd(0, 0))

if __name__ == '__main__':
    rospy.init_node("motion_planner")
    m = MotionPlanner()
    rospy.spin()
