#!/usr/bin/env python

import sys
import time
import rospy
import tf2_ros as tf
import actionlib
import std_msgs

from apriltags.msg import AprilTagDetections
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from dynamixel_controllers.joint_position_controller import JointPositionController

class TaskPlanner:
    def __init__(self):
        self.nav_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction
        )
        self.nav_client.wait_for_server()
        self.tb = tf.Buffer()
        self.yellow = [0,0]
        self.red = [0,0]
        self.blue = [0,0]
        self.green = [0,0]
        self.has_seen_apriltag = False
        self.tl = tf.TransformListener(self.tb)
        self.clear_costmaps = rospy.ServiceProxy(
            "/move_base/clear_costmaps", Empty
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.joint1_pub = rospy.Publisher("/joint1_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher("/joint2_controller/command", std_msgs.msg.Float64, queue_size=10)
        self.gripper_pub = rospy.Publisher("/gripper_pos", std_msgs.msg.Int32, queue_size=10)
        self.red_sub = rospy.Subscriber("/object_detections/red", std_msgs.msg.Float32MultiArray, self.red_callback)
        self.blue_sub = rospy.Subscriber("/object_detections/blue", std_msgs.msg.Float32MultiArray, self.blue_callback)
        self.apriltag_sub = rospy.Subscriber(
            "/apriltags/detections", AprilTagDetections, self.apriltag_callback
        )
        self.tbu = tf.Buffer()
        self.tl = tf.TransformListener(self.tbu)

    def red_callback(self, msg):
        self.red = msg.data
    def blue_callback(self, msg):
        self.blue = msg.data

    def apriltag_callback(self, msg):
        if len(msg.detections):
            self.has_seen_apriltag = True
            self.apriltag_sub.unregister()

    def process_waypoints(self, *points):
        """
        Takes in a list of names of waypoint to move through and moves the base to
        those points in order
        """
        for point in points:
            # Get the location of the point
            t = self.tb.lookup_transform(
                "map", point, rospy.Time(0), rospy.Duration(1)
            )
            print "Moving to {}".format(point)
            if not self.move_to_point(t.transform.translation, t.transform.rotation):
                print "Failed to move to {}".format(point)
                return False
        return True

    def send_command(self, vx, vtheta, duration):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(vtheta)
        self.cmd_vel_pub.publish(cmd)
        time.sleep(duration)
        self.cmd_vel_pub.publish(Twist())

    def move_to_point(self, pos, ori):
        # Clear the costmap in case there is junk in it, and then make the plan
        self.clear_costmaps()
        time.sleep(1)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = pos
        goal.target_pose.pose.orientation = ori
        self.nav_client.send_goal(goal)
        for i in range(5):
            if self.nav_client.wait_for_result(rospy.Duration.from_sec(10)):
                break
            self.nav_client.cancel_goal()
            self.clear_costmaps()
            time.sleep(1)
            self.nav_client.send_goal(goal)
        return (self.nav_client.get_state() == 3)

    def move_arm(self, x, theta):
        real_x = x*100
        real_x = min(max(real_x, -3.14159), 3.14159)
        self.joint1_pub.publish(real_x)
        self.joint2_pub.publish(theta)

    def open_gripper(self):
        self.gripper_pub.publish(1500)

    def close_gripper(self):
        self.gripper_pub.publish(800)

    def pick_up_pokemon(self, x):
        self.close_gripper()
        time.sleep(1)
        self.move_arm(x, 2.4)
        time.sleep(1)

    def drop_pokemon(self):
        self.open_gripper()
        time.sleep(1)

    def xy_error(self, waypoint):
        """
        Moves the arm base to correct for the error in moving to the waypoint
        given by `waypoint`
        """
        t_detected = self.tbu.lookup_transform(
            "base_link", waypoint, rospy.Time.now(), rospy.Duration(0.2)
        )
        return (
            t_detected.transform.translation.x,
            t_detected.transform.translation.y
        )

ARM_UP = 1.5
ARM_DOWN = 2.5

INIT = False
TASK1 = False
TASK2 = False
TASK3 = True

if __name__ == '__main__':
    rospy.init_node('task_planning')
    planner = TaskPlanner()
    time.sleep(1)

    # Lift the arm up before we start driving
    planner.move_arm(0, ARM_UP)
    planner.open_gripper()

    ##### Initialization #####
    if INIT:
        # Wait until you can see an apriltag
        #print "Waiting to see an apriltag"
        while not planner.has_seen_apriltag:
            time.sleep(0.1)

    print "Starting navigation"

    ##### Task 1 #####
    if TASK1:
        print "Starting task 1"
        # Turn left to avoid the pokemon to the right
        planner.send_command(0, 0.5, ARM_DOWN)
        if not planner.process_waypoints(
            "waypoint_1_0", "waypoint_1_1", "waypoint_1_2"
        ):
            sys.exit(1)
        # Move the arm down to grab the pidgey
        x_error, y_error = planner.xy_error("waypoint_1_2")
        planner.move_arm(y_error, ARM_DOWN)
        time.sleep(5)
        planner.send_command(0.1, 0, 1.5 + 10*x_error)
        time.sleep(1)

        # Pick up the pidgey
        planner.pick_up_pokemon(y_error)
        # Back up safely before going to the tote
        planner.send_command(-0.1, -0.5, 6)
        planner.send_command(0, -0.2, 2)
        if not planner.process_waypoints("waypoint_1_3", "waypoint_tote"):
            sys.exit(1)
        planner.drop_pokemon()

    ##### Task 2 ####
    if TASK2:
        print "Starting task 2"
        # Back up safely before going to the shelf
        planner.send_command(-0.15, 0, 2.5)
        planner.move_arm(0, ARM_UP)
        if not planner.process_waypoints("waypoint_2_0"):
            sys.exit(1)

        # Figure out which pokemon to pick up
        red_patch = False
        blue_patch = False
        for i in range(3):
            for j in range(len(planner.red)/2):
                if planner.red[2*j+1] < 0:
                    red_patch = True
                    print "Found red patch"
                    break
            for j in range(len(planner.blue)/2):
                if planner.blue[2*j+1] < 0:
                    blue_patch = True
                    print "Found blue patch"
                    break
            time.sleep(0.1)
        green_patch = (not (red_patch or blue_patch))
        if green_patch:
            print "No patch found. Assuming green"

        red_location = None
        blue_location = None
        location_names = ["center", "left", "right"]
        for i in range(3):
            for j in range(len(planner.red)/2):
                x = planner.red[2*j]
                y = planner.red[2*j+1]
                if y > 0:
                    if abs(x) < 0.1:
                        red_location = 0
                    elif x > 0:
                        red_location = 1
                    else:
                        red_location = -1
                    print "Found red object in {}".format(
                        location_names[red_location]
                    )
            for j in range(len(planner.blue)/2):
                x = planner.blue[2*j]
                y = planner.blue[2*j+1]
                if y > 0:
                    if abs(x) < 0.1:
                        blue_location = 0
                    elif x > 0:
                        blue_location = 1
                    else:
                        blue_location = -1
                    print "Found blue object in {}".format(
                        location_names[blue_location]
                    )
            if red_location is not None and blue_location is not None:
                break
        possible_locations = [-1, 0, 1]
        if red_location is not None:
            possible_locations.remove(red_location)
        if blue_location is not None:
            possible_locations.remove(blue_location)
        green_location = 0 if len(possible_locations) > 1 else possible_locations[0]
        print "Assuming green object is in {}".format(
            location_names[green_location]
        )

        location = 0
        if red_patch:
            location = red_location
        elif blue_patch:
            location = blue_location
        else:
            location = green_location
        print "Moving to {} location".format(location_names[location])

        #Assign and move to waypoint
        waypoint = ""
        if location==-1:
            waypoint = "waypoint_2_shelf1"
            #planner.send_command(0, -0.2, 4)
        elif location==0:
            waypoint = "waypoint_2_shelf2"
        else:
            waypoint = "waypoint_2_shelf3"
            #planner.send_command(0, 0.2, 4)

        planner.send_command(-0.2, 0, 2)
        if not planner.process_waypoints(waypoint):
            sys.exit(1)

        # Move the arm down to grab the pidgey
        (x_error, y_error) = planner.xy_error(waypoint)
        planner.move_arm(y_error, ARM_DOWN)
        time.sleep(5)

        planner.send_command(0.1, 0, 1.5 + 10*x_error)
        time.sleep(1)

        #pick up pokemon
        planner.pick_up_pokemon(y_error)

        # Back up safely before going to the tote
        planner.send_command(-0.25, 0, 2.5)
        planner.send_command(0, 0.3, 10)
        if not planner.process_waypoints("waypoint_tote"):
            sys.exit(1)
        time.sleep(1)

        # Drop pokemon
        planner.drop_pokemon()

    ##### Task 3 #####
    if TASK3:
        # Back up safely before going to the shelf
        planner.send_command(-0.25, 0, 2.5)
        planner.send_command(0, -0.3, 10)
        if not planner.process_waypoints("waypoint_3_shelf"):
            sys.exit(1)

        (x_error, y_error) = planner.xy_error("waypoint_3_shelf")
        planner.move_arm(y_error, 2.5)
        time.sleep(5)

        planner.send_command(0.1, 0, 1.5 + 10*x_error)
        time.sleep(1)

        # Pick up pokemon
        planner.pick_up_pokemon(y_error)

        # Back up safely before going to the tote
        planner.send_command(-0.25, 0, 2.5)
        planner.send_command(0, -0.3, 10)
        planner.move_arm(0, ARM_UP)
        if not planner.process_waypoints("waypoint_tote"):
            sys.exit(1)
        time.sleep(1)
        # Drop pokemon
        planner.drop_pokemon()
