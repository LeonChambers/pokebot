#!/usr/bin/env python

import sys
import time
import rospy
import tf2_ros as tf
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from dynamixel_controllers.joint_position_controller import JointPositionController

class TaskPlanner:
    def __init__(self):
        self.nav_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction
        )
        self.nav_client.wait_for_server()
        self.tb = tf.Buffer()
        tl = tf.TransformListener(self.tb)
        self.clear_costmaps = rospy.ServiceProxy(
            "/move_base/clear_costmaps", Empty
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

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

if __name__ == '__main__':
    rospy.init_node('task_planning')
    planner = TaskPlanner()
    time.sleep(1)

    # Task 1
    # Turn left to avoid the pokemon to the right
    planner.send_command(0, 0.5, 2.5)
    if not planner.process_waypoints(
        "waypoint_1_0", "waypoint_1_1", "waypoint_1_2", "waypoint_1_3"
    ):
        sys.exit(1)
    # TODO: Pick up pokemon
    # Back up safely before going to the tote
    planner.send_command(-0.1, -0.5, 6)
    if not planner.process_waypoints("waypoint_1_4", "waypoint_tote"):
        sys.exit(2)
    # TODO: Drop pokemon

    # Task 2
    # Back up safely before going to the shelf
    planner.send_command(-0.25, 0, 2.5)
    if not planner.process_waypoints("waypoint_2_shelf"):
        sys.exit(3)
    # TODO: Pick up pokemon
    # Back up safely before going to the tote
    planner.send_command(-0.25, 0, 2.5)
    if not planner.process_waypoints("waypoint_tote"):
        sys.exit(4)
    # TODO: Drop pokemon

    # Task 3
    # Back up safely before going to the shelf
    planner.send_command(-0.25, 0, 2.5)
    if not planner.process_waypoints("waypoint_3_shelf"):
        sys.exit(5)
    # TODO: Pick up pokemon
    # Back up safely before going to the tote
    planner.send_command(-0.25, 0, 2.5)
    if not planner.process_waypoints("waypoint_tote"):
        sys.exit(6)
    # TODO: Drop pokemon
