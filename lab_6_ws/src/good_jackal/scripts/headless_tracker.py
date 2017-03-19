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
    
def image_cb(msg):
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
    
    threshMat = blurMat
    
    circles = cv2.HoughCircles(threshMat, cv2.HOUGH_GRADIENT, 1, 20,
                  param1=50,
                  param2=25,
                  minRadius=10,
                  maxRadius=0)
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        
        for (x, y, r) in circles:
            cv2.putText(srcA, "Ball ({}:{}-{})".format(x-320,y-240,r), (x,y-(r+10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
            cv2.circle(srcA, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(srcA, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            tracked = Tracked_Object()
            tracked.x = x-320
            tracked.y = y - 240
            tracked.r = r
            pub.publish(tracked)

    cv2.waitKey(30)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        print(cv2.__version__)
        loop = 1
        rospy.init_node('Headless_Tracker')
        bridge = cv_bridge.CvBridge()
        image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, image_cb)
        
        pub = rospy.Publisher('/good_jackal/object', Tracked_Object, queue_size=10)
        
        while(loop):
            cv2.waitKey(0)
            
            
    except rospy.ROSInterruptException:
        pass


