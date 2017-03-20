#!/usr/bin/env python

# Countour code based on http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
# Trackbar code based on https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/object-tracking-tut/objectTrackingTut.cpp

# imports
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge
from collections import deque
import argparse

import filtering

from good_jackal.msg import Tracked_Object

obj_x = 0
obj_y = 0
obj_r = 0

# HSV Presets
H_MIN = 0
H_MAX = 255
S_MIN = 0
S_MAX = 255
V_MIN = 0
V_MAX = 255

# Erode/Dialate Presets
ERODE_X = 4
ERODE_Y = 4
DIALATE_X = 7
DIALATE_Y = 7

MAX_ER_DI = 20

windowSteps = "Filtering Steps"

windowHSVTrackbars = "HSV Trackbars"
windowErodeDialateTrackbars = "Erode Dialate Trackbars"


# Callback for opencv High GUI trackbar changes
def on_trackbar(a):
    global H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX,ERODE_X,ERODE_Y,DIALATE_X,DIALATE_Y
    
    # Read current HSV trackbar positions, update positions
    H_MIN = cv2.getTrackbarPos("H_MIN",windowHSVTrackbars)
    H_MAX = cv2.getTrackbarPos("H_MAX",windowHSVTrackbars)
    S_MIN = cv2.getTrackbarPos("S_MIN",windowHSVTrackbars)
    S_MAX = cv2.getTrackbarPos("S_MAX",windowHSVTrackbars)
    V_MIN = cv2.getTrackbarPos("V_MIN",windowHSVTrackbars)
    V_MAX = cv2.getTrackbarPos("V_MAX",windowHSVTrackbars)

    # Read current Erode/Dialate trackbar positions, update positions
    ERODE_X = cv2.getTrackbarPos("Erode X",windowErodeDialateTrackbars)
    ERODE_Y = cv2.getTrackbarPos("Erode Y",windowErodeDialateTrackbars)
    DIALATE_X = cv2.getTrackbarPos("Dialate X",windowErodeDialateTrackbars)
    DIALATE_Y = cv2.getTrackbarPos("Dialate Y",windowErodeDialateTrackbars)    

    # Protect Erode/Dialate trackbars from being < 1
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
        
    
# Create HSV bounds window and trackbars
def createHSVTrackbars():
    cv2.namedWindow(windowHSVTrackbars,0)
    cv2.createTrackbar("H_MIN", windowHSVTrackbars, H_MIN, 255, on_trackbar)
    cv2.createTrackbar("H_MAX", windowHSVTrackbars, H_MAX, 255, on_trackbar)
    cv2.createTrackbar("S_MIN", windowHSVTrackbars, S_MIN, 255, on_trackbar)
    cv2.createTrackbar("S_MAX", windowHSVTrackbars, S_MAX, 255, on_trackbar)
    cv2.createTrackbar("V_MIN", windowHSVTrackbars, V_MIN, 255, on_trackbar)
    cv2.createTrackbar("V_MAX", windowHSVTrackbars, V_MAX, 255, on_trackbar)

# Create Erode Dialate element window and trackbars
def createErodeDialateTrackbars():
    cv2.namedWindow(windowErodeDialateTrackbars,0)
    cv2.createTrackbar("Erode X", windowErodeDialateTrackbars, ERODE_X, MAX_ER_DI, on_trackbar)
    cv2.createTrackbar("Erode Y", windowErodeDialateTrackbars, ERODE_Y, MAX_ER_DI, on_trackbar)
    cv2.createTrackbar("Dialate X", windowErodeDialateTrackbars, DIALATE_X, MAX_ER_DI, on_trackbar)
    cv2.createTrackbar("Dialate Y", windowErodeDialateTrackbars, DIALATE_Y, MAX_ER_DI, on_trackbar) 
    
# Callback for object tracker from headless node
def object_cb(msg):
    global obj_x, obj_y, obj_r
    obj_x = msg.x + 320
    obj_y = msg.y + 240
    obj_r = msg.r
    
# Callback for image
def image_cb(msg):
    try:
        srcA = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except cv_bridge.CvBridgeError as e:
        print(e)

    hsvMat = filtering.cvtHSV(srcA)
    blur1Mat = filtering.doBlur(hsvMat, 9, 9)
    threshMat = filtering.doThresh(blur1Mat, H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX)
    erodeMat = filtering.doErode(threshMat, ERODE_X, ERODE_Y, 2)
    dialateMat = filtering.doDialate(erodeMat, DIALATE_X, DIALATE_Y, 2)
    blur2Mat = filtering.doBlur(dialateMat, 2, 2)

    (cal_x, cal_y), cal_r = filtering.findContours(blur2Mat)

    if cal_r > 10:
        cv2.circle(srcA, (cal_x, cal_y), cal_r, (0, 255, 255), 2)
        cv2.putText(srcA, "Local(X:{} Y:{} R:{})".format(cal_x-320,cal_y-240,cal_r), (cal_x, cal_y-cal_r-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)

    if obj_r > 10:
        cv2.circle(srcA, (obj_x, obj_y), obj_r, (255, 0, 255), 2)
        cv2.putText(srcA, "Jackal(X:{} Y:{} R:{})".format(obj_x-320,obj_y-240,obj_r), (obj_x, obj_y+obj_r+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)

    dispMat = filtering.makeFrame(hsvMat, blur1Mat, threshMat, erodeMat, dialateMat, blur2Mat)
    
    cv2.imshow(windowSteps, cv2.resize(dispMat, (0,0), fx=0.5, fy=0.5))
    
    cv2.imshow('Result', srcA)

# Load in parameters on the parameter server if available
def update_params():
    global H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX,ERODE_X,ERODE_Y,DIALATE_X,DIALATE_Y
    
    # H_MIN Parameter
    if rospy.has_param("/good_jackal/tracker/H_MIN"):
        H_MIN = rospy.get_param("/good_jackal/tracker/H_MIN")
        
    # H_MAX Parameter
    if rospy.has_param("/good_jackal/tracker/H_MAX"):
        H_MAX = rospy.get_param("/good_jackal/tracker/H_MAX")
        
    # S_MIN Parameter
    if rospy.has_param("/good_jackal/tracker/S_MIN"):
        S_MIN = rospy.get_param("/good_jackal/tracker/S_MIN")
        
    # S_MAX Parameter
    if rospy.has_param("/good_jackal/tracker/S_MAX"):
        S_MAX = rospy.get_param("/good_jackal/tracker/S_MAX")
        
    # V_MIN Parameter
    if rospy.has_param("/good_jackal/tracker/V_MIN"):
        V_MIN = rospy.get_param("/good_jackal/tracker/V_MIN")
        
    # V_MAX Parameter
    if rospy.has_param("/good_jackal/tracker/V_MAX"):
        V_MAX = rospy.get_param("/good_jackal/tracker/V_MAX")
          
    # ERODE_X Parameter
    if rospy.has_param("/good_jackal/tracker/ERODE_X"):
        ERODE_X = rospy.get_param("/good_jackal/tracker/ERODE_X")
        
    # ERODE_Y Parameter
    if rospy.has_param("/good_jackal/tracker/ERODE_Y"):
        ERODE_Y = rospy.get_param("/good_jackal/tracker/ERODE_Y")    
        
    # DIALATE_X Parameter
    if rospy.has_param("/good_jackal/tracker/DIALATE_X"):
        DIALATE_X = rospy.get_param("/good_jackal/tracker/DIALATE_X")
        
    # DIALATE_Y Parameter
    if rospy.has_param("/good_jackal/tracker/DIALATE_Y"):
        DIALATE_Y = rospy.get_param("/good_jackal/tracker/DIALATE_Y")

# standard ros boilerplate
if __name__ == "__main__":
    try:
        print("OpenCV Version {}".format(cv2.__version__))
        
                
        rospy.init_node('Track_Calibration')
        rate = rospy.Rate(100)
        
        bridge = cv_bridge.CvBridge()
        image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, image_cb)
        object_sb = rospy.Subscriber("/good_jackal/object", Tracked_Object, object_cb)
        
        update_params()
        
        createHSVTrackbars()
        createErodeDialateTrackbars()
        

        while not rospy.is_shutdown():
            key = (cv2.waitKey(30) & 0xFF)
            if(key ==  ord('a')):
                print("Hit A")
            rate.sleep()
            
            
    except rospy.ROSInterruptException:
        pass


