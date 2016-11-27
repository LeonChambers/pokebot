#!/usr/bin/env python

import tf
import rospy

from nav_msgs.msg import Odometry

class TfBroadcaster():
    def __init__(self, src_topic, frame_id, parent_frame_id):
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.odom_sub = rospy.Subscriber(
            src_topic, Odometry, self.odom_callback
        )
        self.tb = tf.TransformBroadcaster()

    def odom_callback(self, odom):
        pos = odom.pose.pose.position
        ori = odom.pose.pose.orientation
        self.tb.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            rospy.Time.now(),
            self.frame_id,
            self.parent_frame_id
        )

if __name__ == '__main__':
    rospy.init_node('pose_visualizer')
    p = TfBroadcaster("/odom", "base_link", "map")
    rospy.spin()
