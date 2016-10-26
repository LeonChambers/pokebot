#!/usr/bin/env python

"""
This script allows the user to drive the mobile base around using the arrow
keys. This is mostly useful for testing that the Arduino and serial
communication work correctly and for collecting some sensor data for debugging
without the navigation stack in place
"""

import rospy
import curses

from me212base.msg import WheelVelCmd

MAX_WHEEL_VELOCITY = 1

rospy.init_node("keyboard_control")
vel_pub = rospy.Publisher(
    "velocity_command", WheelVelCmd, queue_size=10
)

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(
    0, 0, "Navigate the robot base using the arrow keys. Press 'q' to exit"
)

msg = WheelVelCmd(0, 0)
while True:
    c = stdscr.getch()
    if c == curses.KEY_LEFT:
        msg.desiredWV_R = MAX_WHEEL_VELOCITY
        msg.desiredWV_L = 0
    elif c == curses.KEY_RIGHT:
        msg.desiredWV_R = 0
        msg.desiredWV_L = MAX_WHEEL_VELOCITY
    elif c == curses.KEY_UP:
        msg.desiredWV_R = MAX_WHEEL_VELOCITY
        msg.desiredWV_L = MAX_WHEEL_VELOCITY
    elif c == ord('q'):
        msg.desiredWV_R = 0
        msg.desiredWV_L = 0
        break
    else:
        msg.desiredWV_R = 0
        msg.desiredWV_L = 0
    vel_pub.publish(WheelVelCmd(msg.desiredWV_R, msg.desiredWV_L))
vel_pub.publish(WheelVelCmd(msg.desiredWV_R, msg.desiredWV_L))

stdscr.keypad(0)
curses.nocbreak()
curses.echo()
curses.endwin()
