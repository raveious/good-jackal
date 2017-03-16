#!/usr/bin/env python

# imports
import rospy
import numpy as np
import cv2

H_MIN = 0
H_MAX = 255
S_MIN = 0
S_MAX = 255
V_MIN = 0
V_MAX = 255

ERODE_X = 1
ERODE_Y = 1
DIALATE_X = 1
DIALATE_Y = 1

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

def runit():
    capA = cv2.VideoCapture(0)

    capA.set(4,FRAME_WIDTH)
    capA.set(5,FRAME_HEIGHT)

    createHSVTrackbars()
    createErodeDialateTrackbars()

    while(1):
        _, rawA = capA.read()
        
        srcA = cv2.resize(rawA, (0,0), fx=0.5, fy=0.5) 
        
        hsvMat = cv2.cvtColor(srcA, cv2.COLOR_BGR2HSV)
        
        lower = np.array([H_MIN,S_MIN,V_MIN])
        upper = np.array([H_MAX,S_MAX,V_MAX])
        
        threshMat = cv2.inRange(hsvMat,lower,upper)
        
        erodeElement = np.ones((ERODE_X,ERODE_Y),np.uint8)
        dilateElement = np.ones((DIALATE_X,DIALATE_Y),np.uint8)
        
        erodeMat = cv2.erode(threshMat,erodeElement,iterations = 2)
        dialateMat = cv2.dilate(erodeMat,dilateElement,iterations = 2)
        
        cv2.imshow(windowColorRaw, srcA)
        cv2.imshow(windowColorHSV, hsvMat)
        cv2.imshow(windowColorErode, erodeMat)
        cv2.imshow(windowColorDialate, dialateMat)
        
        threshMat = dialateMat
        
        res = cv2.bitwise_and(srcA,srcA, mask=threshMat)
        
        
        cv2.imshow('Result',res)
        cv2.waitKey(30)
        
    cap.release()
    cv2.destroyAllWindows()

# standard ros boilerplate
if __name__ == "__main__":
    try:
        runit()
    except rospy.ROSInterruptException:
        pass


