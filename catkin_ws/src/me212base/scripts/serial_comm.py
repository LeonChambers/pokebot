#!/usr/bin/env python

# 2.12 Lab 2 me212bot: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import sys
import rospy
import serial
import threading
import traceback

from tf.transformations import quaternion_from_euler as e_to_quat
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Int32

b = 0.225

class DummySerial:
    def write(self, msg):
        pass

    def readline(self):
        return "0,0,0"

class Arduino():
    def __init__(self, port):
        if port is None:
            self.comm = DummySerial()
        else:
            self.comm = serial.Serial(port, 115200, timeout=5)
        self.odom = Odometry()
        self.odom.child_frame_id = 'base_link'

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel)
        self.gripper_pub = rospy.Subscriber(
            "gripper_pos", Int32, self.on_gripper_pos
        )
        self.odom_pub = rospy.Publisher(
            "/odom", Odometry, queue_size=10
        )

        self.thread = threading.Thread(target = self.loop)
        self.thread.start()

        self.vr = 0
        self.vl = 0
        self.gripper_pos = 0

    def on_cmd_vel(self, msg):
        # Compute wheel velocities from the Twist message
        vx = msg.linear.x
        vtheta = msg.angular.z
        self.vr = vx + vtheta * b
        self.vl = vx - vtheta * b

        # Send the wheel velocities
        serial_msg = "%f,%f,%d\n" % (self.vr, self.vl, self.gripper_pos)
        print serial_msg
        self.comm.write(serial_msg)

        # Update the odometry message with the new velocities
        self.odom.twist.twist = msg

    def on_gripper_pos(self, msg):
        self.gripper_pos = msg.data

        serial_msg = "%f,%f,%d\n" % (self.vr, self.vl, self.gripper_pos)
        print serial_msg
        self.comm.write(serial_msg)

    def loop(self):
        while not rospy.is_shutdown():
            # 1. get a line of string that represent current odometry from serial
            serialData = self.comm.readline().strip()

            # 2. parse the string e.g. "0.1,0.2,0.1" to doubles
            splitData = serialData.split(',');

            try:
                x     = float(splitData[0]);
                y     = float(splitData[1]);
                theta = float(splitData[2]);

                self.odom.pose.pose.position.x = x
                self.odom.pose.pose.position.y = y
                self.odom.pose.pose.orientation = Quaternion(*e_to_quat(0, 0, theta))

                self.odom_pub.publish(self.odom)

            except (ValueError, IndexError):
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData
                ex_type, ex, tb = sys.exc_info()
                traceback.print_tb(tb)


def main():
    rospy.init_node('me212bot', anonymous=True)
    arduino = Arduino('/dev/ttyACM1')
    rospy.spin()

if __name__=='__main__':
    main()

