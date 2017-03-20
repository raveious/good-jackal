#!/usr/bin/env python

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

# HSV Presets
H_MIN = 36
H_MAX = 48
S_MIN = 115
S_MAX = 255
V_MIN = 116
V_MAX = 255

# Erode/Dialate Presets
ERODE_X = 4
ERODE_Y = 4
DIALATE_X = 7
DIALATE_Y = 7


# Callback for image publisher
def image_cb(msg):
    try:
        print("-Image Rxd")
        srcA = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except cv_bridge.CvBridgeError as e:
        print("-Failed")
        print(e)

    hsvMat = filtering.cvtHSV(srcA)
    blur1Mat = filtering.doBlur(hsvMat, 9, 9)
    threshMat = filtering.doThresh(blur1Mat, H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX)
    erodeMat = filtering.doErode(threshMat, ERODE_X, ERODE_Y, 2)
    dialateMat = filtering.doDialate(erodeMat, DIALATE_X, DIALATE_Y, 2)
    blur2Mat = filtering.doBlur(dialateMat, 2, 2)

    (ball_x, ball_y), ball_r = filtering.findContours(blur2Mat)

    tracked = Tracked_Object()

    if (ball_r > 20) & (ball_y > 90):
        tracked.x = ball_x - 320
        tracked.y = ball_y - 240
        tracked.r = ball_r
    else:
        tracked.x = 0
        tracked.y = 0
        tracked.r = -1
        
    pub.publish(tracked)


def update_params():
    global H_MIN,H_MAX,S_MIN,S_MAX,V_MIN,V_MAX,ERODE_X,ERODE_Y,DIALATE_X,DIALATE_Y,CIRCLE_RATIO,CIRCLE_MIN_DIST,CIRCLE_C1,CIRCLE_C2,CIRCLE_MIN_R,CIRCLE_MAX_R
    
    # H_MIN Parameter
    if rospy.has_param("/good_jackal/tracker/H_MIN"):
        H_MIN = rospy.get_param("/good_jackal/tracker/H_MIN")
    else:
        rospy.set_param("/good_jackal/tracker/H_MIN", H_MIN)
        
    # H_MAX Parameter
    if rospy.has_param("/good_jackal/tracker/H_MAX"):
        H_MAX = rospy.get_param("/good_jackal/tracker/H_MAX")
    else:
        rospy.set_param("/good_jackal/tracker/H_MAX", H_MAX)
        
    # S_MIN Parameter
    if rospy.has_param("/good_jackal/tracker/S_MIN"):
        S_MIN = rospy.get_param("/good_jackal/tracker/S_MIN")
    else:
        rospy.set_param("/good_jackal/tracker/S_MIN", S_MIN)
        
    # S_MAX Parameter
    if rospy.has_param("/good_jackal/tracker/S_MAX"):
        S_MAX = rospy.get_param("/good_jackal/tracker/S_MAX")
    else:
        rospy.set_param("/good_jackal/tracker/S_MAX", S_MAX)
        
    # V_MIN Parameter
    if rospy.has_param("/good_jackal/tracker/V_MIN"):
        V_MIN = rospy.get_param("/good_jackal/tracker/V_MIN")
    else:
        rospy.set_param("/good_jackal/tracker/V_MIN", V_MIN)
        
    # V_MAX Parameter
    if rospy.has_param("/good_jackal/tracker/V_MAX"):
        V_MAX = rospy.get_param("/good_jackal/tracker/V_MAX")
    else:
        rospy.set_param("/good_jackal/tracker/V_MAX", V_MAX)
          
    # ERODE_X Parameter
    if rospy.has_param("/good_jackal/tracker/ERODE_X"):
        ERODE_X = rospy.get_param("/good_jackal/tracker/ERODE_X")
    else:
        rospy.set_param("/good_jackal/tracker/ERODE_X", ERODE_X)
        
    # ERODE_Y Parameter
    if rospy.has_param("/good_jackal/tracker/ERODE_Y"):
        ERODE_Y = rospy.get_param("/good_jackal/tracker/ERODE_Y")
    else:
        rospy.set_param("/good_jackal/tracker/ERODE_Y", ERODE_Y)        
        
    # DIALATE_X Parameter
    if rospy.has_param("/good_jackal/tracker/DIALATE_X"):
        DIALATE_X = rospy.get_param("/good_jackal/tracker/DIALATE_X")
    else:
        rospy.set_param("/good_jackal/tracker/DIALATE_X", DIALATE_X)
        
    # DIALATE_Y Parameter
    if rospy.has_param("/good_jackal/tracker/DIALATE_Y"):
        DIALATE_Y = rospy.get_param("/good_jackal/tracker/DIALATE_Y")
    else:
        rospy.set_param("/good_jackal/tracker/DIALATE_Y", DIALATE_Y)


# standard ros boilerplate
if __name__ == "__main__":
    try:            
        print(cv2.__version__)
                
        rospy.init_node('Headless_Tracker')
        rate = rospy.Rate(100)
        
        update_params()
        
        bridge = cv_bridge.CvBridge()
        image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, image_cb)
        pub = rospy.Publisher('/good_jackal/object', Tracked_Object, queue_size=1)

        while not rospy.is_shutdown():
            cv2.waitKey(3)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass


