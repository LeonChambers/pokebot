#!/usr/bin/env python

import rospy

from me212base.srv import Set2DPose

def navigate_base_to_position(pose):
    raise NotImplementedError()

if __name__ == '__main__':
    rospy.init_node("path_planner")
    s = rospy.Service(
        'navigate_base_to_position', Set2DPose, navigate_base_to_position
    )
    rospy.spin()
