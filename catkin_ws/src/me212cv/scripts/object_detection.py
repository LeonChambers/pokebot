#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016

import rospy
import numpy as np
import cv2  # OpenCV module
from scipy.ndimage.filters import maximum_filter
from scipy.signal import medfilt

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import (
    ColorRGBA, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
)

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('object_detection', anonymous=True)

# Publisher for publishing pyramid marker in rviz
vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()

# Get the camera calibration parameter for the rectified image
msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=None)
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]

fx = msg.P[0]
fy = msg.P[5]
cx = msg.P[2]
cy = msg.P[6]

colors = ["red", "blue"]
colors_bgr = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 100, 100)
}
colors_lower = {
    "red": np.array([0, 90, 90]),
    "green": np.array([110,20,20]),
    "blue": np.array([110,70,70]),
    "yellow": np.array([20,50,50])
}
colors_upper = {
    "red": np.array([20,255,255]),
    "green": np.array([140,255,255]),
    "blue": np.array([140,255,255]),
    "yellow": np.array([40,255,255])
}
color_publishers = {
    color: rospy.Publisher(
        "/object_detections/{}".format(color), Float32MultiArray, queue_size=10
    ) for color in colors
}

def rosRGBDCallBack(rgb_data, depth_data):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "16SC1")
    except CvBridgeError as e:
        print(e)

    # Convert the image to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    color_masks = {
        color: cv2.inRange(hsv_image, colors_lower[color], colors_upper[color])
        for color in colors
    }
    kwidth = 41
    kheight = 65
    color_scores = {
        color: cv2.blur(color_masks[color], (kwidth, kheight)) +
        np.random.randn(len(color_masks[color]), len(color_masks[color][0]))
        for color in colors
    }
    color_maxima = {
        color:
            maximum_filter(color_scores[color], kheight) == color_scores[color]
        for color in colors
    }
    threshold = 80
    color_thresholded = {
        color: color_scores[color] > threshold for color in colors
    }
    color_object_map = {
        color: np.logical_and(color_maxima[color], color_thresholded[color])
        for color in colors
    }
    color_object_locations = {
        color: np.argwhere(color_object_map[color]) for color in colors
    }

    for color in colors:
        locs = color_object_locations[color]
        values = []
        for i in range(min(len(locs), 5)):
            loc = locs[i, :]
            center_x = loc[1]
            center_y = loc[0]
            values.extend([(center_x - cx) / fx, (center_y - cy) / fy])
            pt1 = (center_x - (kwidth/2), center_y - (kheight/2))
            pt2 = (center_x + (kwidth/2), center_y + (kheight/2))
            cv2.rectangle(
                cv_image, pt1, pt2, colors_bgr[color]
            )
            depth = cv_depthimage[loc[0], loc[1]]
            cv2.putText(
                cv_image, str(depth), (loc[1], loc[0]), cv2.FONT_HERSHEY_PLAIN,
                1, (255, 255, 255)
            )
        dim = MultiArrayDimension("coordinates", len(values), 1)
        layout = MultiArrayLayout((dim,), 0)
        color_publishers[color].publish(layout, values)
    # cv2.imshow('Objects', cv_image)
    # for color in colors:
    #     cv2.imshow(color, color_masks[color])
    # cv2.waitKey(3)

def getXYZ(xp, yp, zc, fx,fy,cx,cy):
    ##
    xn = (xp - cx) / fx
    yn = (yp - cy) / fy
    xc = xn * zc
    yc = yn * zc
    return (xc,yc,zc)

if __name__=='__main__':
    image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
    ts.registerCallback(rosRGBDCallBack)

    rospy.spin()
