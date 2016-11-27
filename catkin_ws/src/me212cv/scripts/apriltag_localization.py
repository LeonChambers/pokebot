#!/usr/bin/env python
"""
Publishes the transformation from map to apriltag_root based on apriltag
detection events
"""

import math
import numpy
import tf2_ros as tf
import time
import rospy

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from apriltags.msg import AprilTagDetections
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

alpha = 0.2
pos_x = 0
pos_y = 0
ori = 0

class DetectionHandler:
    def __init__(self, tb):
        self.tbr = tb
        self.tbu = tf.Buffer()
        self.tl = tf.TransformListener(self.tbu)

    def on_detection(self, msg):
        for d in msg.detections:
            # Construct the transformation from the robot to the detected tag
            t = TransformStamped()
            t.header.frame_id = "camera_rgb_optical_frame"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "detected_apriltag{}".format(d.id)
            t.transform.translation = d.pose.position
            t.transform.rotation = d.pose.orientation
            self.tbr.sendTransform(t)

            # Compute how much the detected pose differs from the expected pose
            t_detected = self.tbu.lookup_transform(
                "apriltag_root", "detected_apriltag{}".format(d.id),
                rospy.Time.now() - rospy.Duration(0.1)
            )
            t_expected = self.tbu.lookup_transform(
                "apriltag_root", "apriltag{}".format(d.id),
                rospy.Time.now() - rospy.Duration(0.1)
            )

            # Update the expectations accordingly
            global pos_x, pos_y
            delta_x = t_detected.transform.translation.x - t_expected.transform.translation.x
            delta_y = t_detected.transform.translation.y - t_expected.transform.translation.y
            delta_x *= alpha
            delta_y *= alpha
            pos_x += delta_x*math.cos(ori) - delta_y*math.sin(ori)
            pos_y += delta_x*math.sin(ori) + delta_y*math.cos(ori)

            global ori
            ori_detected = euler_from_quaternion(numpy.array([
                t_detected.transform.rotation.x,
                t_detected.transform.rotation.y,
                t_detected.transform.rotation.z,
                t_detected.transform.rotation.w
            ]))[2]
            ori_expected = euler_from_quaternion(numpy.array([
                t_expected.transform.rotation.x,
                t_expected.transform.rotation.y,
                t_expected.transform.rotation.z,
                t_expected.transform.rotation.w
            ]))[2]
            delta_ori = ori_detected - ori_expected
            if delta_ori > math.pi:
                delta_ori -= 2*math.pi;
            delta_ori *= alpha
            ori += delta_ori

            # Compute the change in the expected position caused by a rotation
            # of apriltag_root by delta_ori in the coordinate frame of
            # apriltag_root
            delta_x = t_expected.transform.translation.y * delta_ori
            delta_y = -t_expected.transform.translation.x * delta_ori
            # Apply this translation to apriltag_root in the map
            # coordinate_frame
            pos_x += delta_x*math.cos(ori) - delta_y*math.sin(ori)
            pos_y += delta_x*math.sin(ori) + delta_y*math.cos(ori)

if __name__ == '__main__':
    rospy.init_node('apriltag_localization')

    tb = tf.TransformBroadcaster()
    handler = DetectionHandler(tb)

    ad_sub = rospy.Subscriber(
        "/apriltags/detections", AprilTagDetections, handler.on_detection
    )
    t = TransformStamped()
    t.header.frame_id = "map"
    t.child_frame_id = "apriltag_root"
    t.transform.translation.z = 0
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, ori))
        tb.sendTransform(t)
        time.sleep(0.1)
