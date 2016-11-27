#!/usr/bin/env python
"""
Publishes the transformation from map to apriltag_root based on apriltag
detection events
"""

import tf
import time
import rospy

from apriltags.msg import AprilTagDetections

tb = tf.TransformBroadcaster()
tl = tf.TransformListener()

pos = (0, 0, 0)
ori = (0, 0, 0, 1)

def detection_callback(msg):
    for d in msg.detections:
        # Construct the transformation from the robot to the detected tag
        t = TransformStamped()
        t.header.frame_id = "camera_rgb_optical_frame"
        t.child_frame_id = "detected_apriltag{}".format(d.id)
        t.transform.translation.x = d.pose.position.x
        t.transform.translation.y = d.pose.position.y
        t.transform.translation.z = d.pose.position.z
        t.transform.rotation = d.pose.orientation

        # Compute how much the detected pose differs from the expected pose

        # Update the expectations accordingly
    pass

if __name__ == '__main__':
    rospy.init_node('apriltag_localization')
    ad_sub = rospy.Subscriber(
        "/apriltag/detections", AprilTagDetections, detection_callback
    )
    while not rospy.is_shutdown():
        tb.sendTransform(
            pos, ori, rospy.Time.now(), "apriltag_root", "map"
        )
        time.sleep(0.1)
