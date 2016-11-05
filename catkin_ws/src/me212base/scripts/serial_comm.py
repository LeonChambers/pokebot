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
from geometry_msgs.msg import Twist

b = 0.225

class Arduino():
    def __init__(self, port = '/dev/ttyACM0'):
        self.comm = serial.Serial(port, 115200, timeout = 5)
        self.odom = Odometry()
        
        self.thread = threading.Thread(target = self.loop)
        self.thread.start()
        
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel)
        self.odom_pub = rospy.Publisher(
            "/odom/dead_reckoning", Odometry, queue_size=10
        )

    def cmd_vel(self, msg):  
        # Compute wheel velocities from the Twist message
        vx = msg.linear.x
        vtheta = msg.angular.z
        vr = vx + vtheta * b
        vl = vx - vtheta * b

        # Send the wheel velocities
        self.comm.write("%f,%f\n" % (vr, vl))

        # Update the odometry message with the new velocities
        self.odom.twist.twist = msg
    
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
                
                print "(x={}, y={}, theta={})".format(x, y, theta)

                self.odom.pose.position.x = x
                self.odom.pose.position.y = y
                self.odom.pose.orientation = e_to_quat(0, 0, theta)

                self.odom_pub.publish(self.odom)
                
            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData
                ex_type, ex, tb = sys.exc_info()
                traceback.print_tb(tb)


def main():
    rospy.init_node('me212bot', anonymous=True)
    arduino = Arduino('/dev/ttyACM0')
    rospy.spin()
    
if __name__=='__main__':
    main()
    
