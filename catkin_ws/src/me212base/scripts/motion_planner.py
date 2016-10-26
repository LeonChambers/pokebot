#!/usr/bin/env python

import rospy

from me212base.srv import Set2DPose

def move_base_to_position(pose):
    raise NotImplementedError()

if __name__ == '__main__':
    rospy.init_node("motion_planner")
    s = rospy.Service(
        'move_base_to_position', Set2DPose, move_base_to_position
    )
    rospy.spin()
