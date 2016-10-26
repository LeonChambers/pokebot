#!/usr/bin/python

# 2.12 Lab 2 me212bot: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import sys
import rospy
import serial
import threading
import traceback

from me212base.msg import WheelVelCmd
from geometry_msgs.msg import Pose2D

port = '/dev/ttyACM0'

class Arduino():
    def __init__(self, port = '/dev/ttyACM0'):
        self.comm = serial.Serial(port, 115200, timeout = 5)
        self.sendbuff = []
        self.readbuff = ''
        
        self.thread = threading.Thread(target = self.loop)
        self.thread.start()
        
        self.prevtime = rospy.Time.now()
        
        self.velcmd_sub = rospy.Subscriber(
            "velocity_command", WheelVelCmd, self.cmdvel
        )
        self.pose_pub = rospy.Publisher(
            "base_pose/dead_reckoning", Pose2D, queue_size=10
        )

    def cmdvel(self, msg):  
        self.comm.write("%f,%f\n" % (msg.desiredWV_R, msg.desiredWV_L))
    
    # loop() is for reading odometry from Arduino and publish to rostopic.
    def loop(self):
        while not rospy.is_shutdown():
            # 1. get a line of string that represent current odometry from serial
            serialData = self.comm.readline()
            
            # 2. parse the string e.g. "0.1,0.2,0.1" to doubles
            splitData = serialData.split(',');
            
            try:
                x     = float(splitData[0]);
                y     = float(splitData[1]);
                theta = float(splitData[2]);
                
                self.pose_pub.publish(Pose(
                    x, y, theta
                ))
                
            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData
                ex_type, ex, tb = sys.exc_info()
                traceback.print_tb(tb)


def main():
    rospy.init_node('me212bot', anonymous=True)
    arduino = Arduino()
    rospy.spin()
    
if __name__=='__main__':
    main()
    
