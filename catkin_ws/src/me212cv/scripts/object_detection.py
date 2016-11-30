#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016

import rospy
import numpy as np
import cv2  # OpenCV module
from scipy.ndimage.filters import maximum_filter

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

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

def rosRGBDCallBack(rgb_data, depth_data):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "16SC1")
        #cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
    except CvBridgeError as e:
        print(e)

    # Convert the image to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    colors = ["red", "green", "blue"]
    colors_lower = {
        "red": np.array([170, 50, 50]),
        "green": np.array([75,50,50]),
        "blue": np.array([110,50,50])
    }
    colors_upper = {
        "red": np.array([180,255,255]),
        "green": np.array([100,255,255]),
        "blue": np.array([140,255,255])
    }
    color_masks = {
        color: cv2.inRange(hsv_image, colors_lower[color], colors_upper[color])
        for color in colors
    }
    kwidth = 20
    kheight = 30
    color_scores = {
        color: cv2.blur(color_masks[color], (kwidth, kheight)) for color in
        colors
    }
    color_maxima = {
        color:
            maximum_filter(color_scores[color], kheight) == color_scores[color]
        for color in colors
    }
    color_thresholded = {
        color:
    }
    print color_maxima

    best_idx_red = np.argmax(color_scores["red"])
    best_coord_red = (best_idx_red % len(color_scores["red"][0]), best_idx_red
            / len(color_scores["red"][0]))
    cv_image_red = cv_image
    cv2.rectangle(cv_image, pt1, pt2, color)
    cv2.circle(cv_image_red, best_coord_red, ksize, (255, 255, 0))
    cv2.imshow('Red_Best', cv_image)
    cv2.waitKey(1)

    #for cnt in contours:
    #    xp,yp,w,h = cv2.boundingRect(cnt)

        # Get depth value from depth image, need to make sure the value is in the normal range 0.1-10 meter
    #    if not math.isnan(cv_depthimage2[int(yp)][int(xp)]) and cv_depthimage2[int(yp)][int(xp)] > 0.1 and cv_depthimage2[int(yp)][int(xp)] < 10.0:
    #        zc = cv_depthimage2[int(yp)][int(xp)]
            #print 'zc', zc
    #    else:
    #        continue

    #    centerx, centery = xp+w/2, yp+h/2
    #    cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255],2)

    #    showPyramid(centerx, centery, zc, w, h)

    # for i in range(len(cv_depthimage2)):
    #     for j in range(len(cv_depthimage2[0])):
    #         if cv_depthimage2[i][j] < 0.5:
    #             cv_image.itemset((i,j,2),0)
    # showImageInCVWindow(cv_image, (255-blank_mask), (255-blank_mask))

def getXYZ(xp, yp, zc, fx,fy,cx,cy):
    ##
    xn = (xp - cx) / fx
    yn = (yp - cy) / fy
    xc = xn * zc
    yc = yn * zc
    return (xc,yc,zc)

def showImageInCVWindow(cv_image, mask_erode_image, mask_image):
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, mask = mask_image)

    # Draw a cross at the center of the image
    cv2.line(cv_image, (320, 235), (320, 245), (255,0,0))
    cv2.line(cv_image, (325, 240), (315, 240), (255,0,0))

    # Show the images
    cv2.imshow('OpenCV_Original', cv_image)
    cv2.imshow('OpenCV_Mask_Erode', mask_erode_image)
    cv2.imshow('OpenCV_Mask_Dilate', mask_image)
    cv2.imshow('OpenCV_View', res)
    cv2.waitKey(3)

# Create a pyramid using 4 triangles
def showPyramid(xp, yp, zc, w, h):
    # X1-X4 are the 4 corner points of the base of the pyramid
    X1 = getXYZ(xp-w/2, yp-h/2, zc, fx, fy, cx, cy)
    X2 = getXYZ(xp-w/2, yp+h/2, zc, fx, fy, cx, cy)
    X3 = getXYZ(xp+w/2, yp+h/2, zc, fx, fy, cx, cy)
    X4 = getXYZ(xp+w/2, yp-h/2, zc, fx, fy, cx, cy)
    vis_pub.publish(createTriangleListMarker(1, [X1, X2, X3, X4], rgba = [1,0,0,1], frame_id = '/camera'))

# Create a list of Triangle markers for visualization
def createTriangleListMarker(marker_id, points, rgba, frame_id = '/camera'):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.TRIANGLE_LIST
    marker.scale = Vector3(1,1,1)
    marker.id = marker_id

    n = len(points)

    if rgba is not None:
        marker.color = ColorRGBA(*rgba)

    o = Point(0,0,0)
    for i in xrange(n):
        p = Point(*points[i])
        marker.points.append(p)
        p = Point(*points[(i+1)%4])
        marker.points.append(p)
        marker.points.append(o)

    marker.pose = poselist2pose([0,0,0,0,0,0,1])
    return marker

def poselist2pose(poselist):
    return Pose(Point(*poselist[0:3]), Quaternion(*poselist[3:7]))

if __name__=='__main__':
    image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
    ts.registerCallback(rosRGBDCallBack)

    rospy.spin()
