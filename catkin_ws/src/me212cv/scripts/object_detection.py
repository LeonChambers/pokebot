#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016

import rospy
import numpy as np
import cv2  # OpenCV module

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

# Task 2 object detection code
def HSVObjectDetection(cv_image, toPrint = True):
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])

    lower_green = np.array([75,50,50])
    upper_green = np.array([100,255,255])

    lower_blue = np.array([110,50,50])
    upper_blue = np.array([140,255,255])

    # Threshold the HSV image to get only red colors
    #maskr = cv2.inRange(hsv_image, lower_red, upper_red)   ##
    #mask_erodedr         = cv2.erode(maskr, None, iterations = 3)  ##
    #mask_eroded_dilatedr = cv2.dilate(mask_erodedr, None, iterations = 10)  ##

    #maskg = cv2.inRange(hsv_image, lower_green, upper_green)   ##
    #mask_erodedg         = cv2.erode(maskg, None, iterations = 3)  ##
    #mask_eroded_dilatedg = cv2.dilate(mask_erodedg, None, iterations = 10)  ##

    #maskb = cv2.inRange(hsv_image, lower_blue, upper_blue)   ##
    #mask_erodedb         = cv2.erode(maskb, None, iterations = 3)  ##
    #mask_eroded_dilatedb = cv2.dilate(mask_erodedb, None, iterations = 10)  ##

    #detectorIm = (255-mask_eroded_dilatedg)

    #detector = cv2.SimpleBlobDetector_create()

    blank_mask = cv2.inRange(hsv_image, np.array([10,50,50]), np.array([10,50,50]))
    mask_eroded = blank_mask
    mask_eroded_dilated=blank_mask

    #keypoints = detector.detect(detectorIm)
    #if len(keypoints)>1:
    #    mask_eroded = mask_erodedg
    #    mask_eroded_dilated = mask_eroded_dilatedg
    #else:
    #    keypoints = detector.detect((255-mask_eroded_dilatedb))
    #    if len(keypoints)>1:
    #        mask_eroded = mask_erodedb
    #        mask_eroded_dilated = mask_eroded_dilatedb

    #image size = (240,320)

    target = (0,0)

    test_point = hsv_image[400][320][0]
    if test_point <=180 and test_point >=170:
        target = (170,180)
    elif test_point <=80 and test_point>=50:
        target = (50,80)
    elif test_point <=140 and test_point>=110:
        target = (110,140)

    target = (170,180)
    res = []
    for i in range(len(hsv_image[0])):
        if hsv_image[400][i][0] >= target[0] and hsv_image[400][i][0] <= target[1]:
            res.append(i)

    point = 0
    s = np.std(res)
    a = np.average(res)
    outliers = []
    for i in range(len(res)):
        if i>=len(res):
            break
        if abs(res[i]-a)>s:
            res.pop(i)
            i-=1

    if len(res) > 30:
        point = res[(int)(len(res)/2)]

    for i in range(len(hsv_image[0])):
        hsv_image.itemset((400,i,0),255)
        hsv_image.itemset((100,i,2),0)
    for j in range(len(hsv_image)):
        hsv_image.itemset((j,point,2),0)
        hsv_image.itemset((j,400,2),0)

    #im_with_keypoints = cv2.drawKeypoints(detectorIm, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    if toPrint:
        print test_point
        #print point
        #print 'hsv', hsv_image[240][320] # the center point hsv
        #print len(keypoints)

    showImageInCVWindow(hsv_image, (255-blank_mask), (255-blank_mask))
    #showImageInCVWindow(im_with_keypoints, mask_eroded, mask_eroded_dilated)
    image,contours,hierarchy = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    return contours, mask_eroded_dilated

# Task 3 callback
def rosRGBDCallBack(rgb_data, depth_data):
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
        cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "16SC1")
        #cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
    except CvBridgeError as e:
        print(e)

    cv_depthimage = cv_depthimage + 2*np.absolute(cv_depthimage)
    cv_depthimage = np.amax(cv_depthimage) - cv_depthimage

    print np.amax(cv_depthimage)
    cv2.imshow('Depth', 255*cv_depthimage/np.amax(cv_depthimage))

    # Convert the image to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define the range of colors in HSV
    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180,255,255])
    lower_green = np.array([75,50,50])
    upper_green = np.array([100,255,255])
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([140,255,255])

    mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

    score_red = np.multiply(cv_depthimage, mask_red)
    score_green = np.multiply(cv_depthimage, mask_green)
    score_blue = np.multiply(cv_depthimage, mask_blue)

    ksize = 25
    score_red = cv2.blur(score_red, (ksize, ksize))
    best_idx_red = np.argmax(score_red)
    best_coord_red = (best_idx_red % len(score_red[0]), best_idx_red / len(score_red[0]))
    cv_image_red = cv_image
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
