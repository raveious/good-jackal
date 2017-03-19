#!/usr/bin/env python

# imports
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge
from collections import deque
import argparse

from good_jackal.msg import Tracked_Object

H_MIN = 26
H_MAX = 37
S_MIN = 115
S_MAX = 220
V_MIN = 36
V_MAX = 255

ERODE_X = 4
ERODE_Y = 4
DIALATE_X = 7
DIALATE_Y = 7

MAX_ER_DI = 20

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

windowColorRaw = "Color Raw"
windowColorHSV = "Color HSV"
windowColorThresh = "Color Threshold"
windowColorErode = "Mask Erode"
windowColorDialate = "Mask Dialate"

windowHSVTrackbars = "HSV Trackbars"
windowErodeDialateTrackbars = " Erode Dialate Trackbars"


def on_trackbar(a):
    global H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX,ERODE_X,ERODE_Y,DIALATE_X,DIALATE_Y
    
    H_MIN = cv2.getTrackbarPos("H_MIN",windowHSVTrackbars)
    H_MAX = cv2.getTrackbarPos("H_MAX",windowHSVTrackbars)
    S_MIN = cv2.getTrackbarPos("S_MIN",windowHSVTrackbars)
    S_MAX = cv2.getTrackbarPos("S_MAX",windowHSVTrackbars)
    V_MIN = cv2.getTrackbarPos("V_MIN",windowHSVTrackbars)
    V_MAX = cv2.getTrackbarPos("V_MAX",windowHSVTrackbars)

    ERODE_X = cv2.getTrackbarPos("Erode X",windowErodeDialateTrackbars)
    ERODE_Y = cv2.getTrackbarPos("Erode Y",windowErodeDialateTrackbars)
    DIALATE_X = cv2.getTrackbarPos("Dialate X",windowErodeDialateTrackbars)
    DIALATE_Y = cv2.getTrackbarPos("Dialate Y",windowErodeDialateTrackbars)    
    
    if(ERODE_X<1):
        ERODE_X=1
        cv2.setTrackbarPos("Erode X", windowErodeDialateTrackbars, 1)
    
    if(ERODE_Y<1):
        ERODE_Y=1
        cv2.setTrackbarPos("Erode Y", windowErodeDialateTrackbars, 1)
    
    if(DIALATE_X<1):
        DIALATE_X=1
        cv2.setTrackbarPos("Dialate X", windowErodeDialateTrackbars, 1)
    
    if(DIALATE_Y<1):
        DIALATE_Y=1
        cv2.setTrackbarPos("Dialate Y", windowErodeDialateTrackbars, 1)
    
    
def createHSVTrackbars():
    cv2.namedWindow(windowHSVTrackbars,0)
    cv2.createTrackbar("H_MIN", windowHSVTrackbars, H_MIN, H_MAX, on_trackbar)
    cv2.createTrackbar("H_MAX", windowHSVTrackbars, H_MAX, H_MAX, on_trackbar)
    cv2.createTrackbar("S_MIN", windowHSVTrackbars, S_MIN, S_MAX, on_trackbar)
    cv2.createTrackbar("S_MAX", windowHSVTrackbars, S_MAX, S_MAX, on_trackbar)
    cv2.createTrackbar("V_MIN", windowHSVTrackbars, V_MIN, V_MAX, on_trackbar)
    cv2.createTrackbar("V_MAX", windowHSVTrackbars, V_MAX, V_MAX, on_trackbar)

def createErodeDialateTrackbars():
    cv2.namedWindow(windowErodeDialateTrackbars,0)
    cv2.createTrackbar("Erode X", windowErodeDialateTrackbars, ERODE_X, MAX_ER_DI, on_trackbar)
    cv2.createTrackbar("Erode Y", windowErodeDialateTrackbars, ERODE_Y, MAX_ER_DI, on_trackbar)
    cv2.createTrackbar("Dialate X", windowErodeDialateTrackbars, DIALATE_X, MAX_ER_DI, on_trackbar)
    cv2.createTrackbar("Dialate Y", windowErodeDialateTrackbars, DIALATE_Y, MAX_ER_DI, on_trackbar)
    
def image_cb(msg):
    print('Image Rxd')
    try:
        srcA = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except cv_bridge.CvBridgeError as e:
        print(e)

    hsvMat = cv2.cvtColor(srcA, cv2.COLOR_BGR2HSV)
    
    lower = np.array([H_MIN,S_MIN,V_MIN])
    upper = np.array([H_MAX,S_MAX,V_MAX])
    
    threshMat = cv2.inRange(hsvMat,lower,upper)
    
    erodeElement = np.ones((ERODE_X,ERODE_Y),np.uint8)
    dilateElement = np.ones((DIALATE_X,DIALATE_Y),np.uint8)
    
    erodeMat = cv2.erode(threshMat,erodeElement,iterations = 2)
    dialateMat = cv2.dilate(erodeMat,dilateElement,iterations = 2)
    
    blurMat = cv2.blur(dialateMat,  (9, 9));        
    
    #cv2.imshow(windowColorRaw, cv2.resize(srcA, (0,0), fx=0.5, fy=0.5))
    #cv2.imshow(windowColorHSV, cv2.resize(hsvMat, (0,0), fx=0.5, fy=0.5))
    #cv2.imshow(windowColorErode, cv2.resize(erodeMat, (0,0), fx=0.5, fy=0.5))
    #cv2.imshow(windowColorDialate, cv2.resize(dialateMat, (0,0), fx=0.5, fy=0.5))
    cv2.imshow('Blur', cv2.resize(blurMat, (0,0), fx=0.5, fy=0.5))
    
    threshMat = blurMat
    
    circles = cv2.HoughCircles(threshMat, cv2.HOUGH_GRADIENT, 1, 20,
                  param1=50,
                  param2=25,
                  minRadius=0,
                  maxRadius=0)
    
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
     
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            cv2.putText(srcA, "Ball ({}:{}-{})".format(x-320,y-240,r), (x,y-(r+10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
            cv2.circle(srcA, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(srcA, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            tracked = Tracked_Object()
            tracked.x = x-320
            tracked.y = y - 240
            tracked.r = r
            pub.publish(tracked)
    
    cv2.imshow('Result',srcA)
    cv2.waitKey(30)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        loop = 1
        print('Starting')
        rospy.init_node('Track_Marker')
        createHSVTrackbars()
        createErodeDialateTrackbars()
        bridge = cv_bridge.CvBridge()
        image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, image_cb)
        
        pub = rospy.Publisher('/good_jackal/object', Tracked_Object, queue_size=10)
        
        while(loop):
            cv2.waitKey(0)
            
            
    except rospy.ROSInterruptException:
        pass


