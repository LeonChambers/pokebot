#!/usr/bin/env python

import tf
import rospy

from geometry_msgs.msg import Pose2D

class TfBroadcaster():
    def __init__(self, src_topic, frame_id, parent_frame_id):
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.pose_sub = rospy.Subscriber(
            src_topic, Pose2D, self.pose_callback
        )
        self.tb = tf.TransformBroadcaster()

    def pose_callback(self, pose):
        self.tb.sendTransform(
            (pose.x, pose.y, 0), 
            tf.transformations.quaternion_from_euler(0, 0, pose.theta),
            rospy.Time.now(),
            self.frame_id,
            self.parent_frame_id
        )

if __name__ == '__main__':
    rospy.init_node('pose_visualizer')
    p = TfBroadcaster("/base_pose/dead_reckoning", "base", "map")
    rospy.spin()
