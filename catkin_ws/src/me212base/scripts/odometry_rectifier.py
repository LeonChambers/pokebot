#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry

class OdometryRectifier:
    def __init__(self, dr_topic, at_topic, out_topic):
        self.odom = Odometry()

        # Set up reads from the dead reckoning topic
        self.last_dr_odom = None
        self.dr_sub = rospy.Subscriber(
            dr_topic, Odometry, self.on_dr
        )

    def on_dr(self, msg):
        """
        Process an odometry message computed using dead reckoning. The
        velocity values are probably correct, but there might be some drift, so
        we will only add the change in heading since the last timestep to the
        odometry that we're sending.
        """
        if self.last_dr_odom is None:
            self.last_dr_odom = msg
            return
        self.odom.twist = msg.twist

if __name__ == '__main__':
    rospy.init_node('odometry_rectifier'
