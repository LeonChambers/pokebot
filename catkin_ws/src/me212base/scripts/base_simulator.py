#!/usr/bin/env python

import math
import time
import rospy
import threading

from me212base.msg import WheelVelCmd
from geometry_msgs.msg import Pose2D

B = 0.225

class Simulator:
    def __init__(self):
        self.pose = Pose2D()
        self.velocity_command = WheelVelCmd(0, 0)
        self.velcmd_sub = rospy.Subscriber(
            "velocity_command", WheelVelCmd, self.command_velocity
        )
        self.pose_pub = rospy.Publisher(
            "base_pose/dead_reckoning", Pose2D, queue_size=10
        )
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def command_velocity(self, msg):
        self.velocity_command = msg

    def loop(self):
        prevTime = time.time()
        while not rospy.is_shutdown():
            # Throttle updates
            rospy.sleep(0.01)

            # Check how much time has passed
            currTime = time.time()
            dt = currTime - prevTime
            
            # How far should the wheels have moved since the last update?
            dWheelL = self.velocity_command.desiredWV_L * dt
            dWheelR = self.velocity_command.desiredWV_R * dt

            # Update the pose accordingly
            x = self.pose.x
            y = self.pose.y
            theta = self.pose.theta

            dTheta = (dWheelR - dWheelL) / (2 * B)
            theta += (dTheta/2)
            dX = 0.5 * math.cos(theta) * (dWheelR + dWheelL)
            dY = 0.5 * math.sin(theta) * (dWheelR + dWheelL)
            theta += (dTheta/2)
            x += dX
            y += dY

            self.pose.x = x
            self.pose.y = y
            self.pose.theta = theta

            # Publish the pose
            self.pose_pub.publish(self.pose)

            # Update the loop state
            prevTime = currTime


if __name__ == '__main__':
    rospy.init_node("base_simulator")
    sim = Simulator()
    rospy.spin()
