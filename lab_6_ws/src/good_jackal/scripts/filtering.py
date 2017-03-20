#!/usr/bin/env python

# imports
import numpy as np
import cv2
import copy

def cvtHSV(src):
    return cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    
def doThresh(src, H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX):
    lower = np.array([H_MIN,S_MIN,V_MIN])
    upper = np.array([H_MAX,S_MAX,V_MAX])
    return cv2.inRange(src,lower,upper)

def doErode(src, ERODE_X, ERODE_Y, iterations):
    erodeElement = np.ones((ERODE_X,ERODE_Y),np.uint8)
    return cv2.erode(src, erodeElement, iterations = iterations)

def doDialate(src, DIALATE_X, DIALATE_Y, iterations):
    dilateElement = np.ones((DIALATE_X,DIALATE_Y),np.uint8)
    return cv2.dilate(src, dilateElement, iterations = iterations)

def doBlur(src, BLUR_X, BLUR_Y):
    return cv2.blur(src, (BLUR_X, BLUR_Y));
		
		
def findContours(src):
    x = -1
    y = -1
    radius = -1
    
    countours = copy.copy(src)
    
    cnts = cv2.findContours(countours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    return [(int(x),int(y)),int(radius)]


def makeFrame(hsv, blur1, thresh, erode, dialate, blur2):
    
    threshMat = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    erodeMat = cv2.cvtColor(erode, cv2.COLOR_GRAY2BGR)
    dialateMat = cv2.cvtColor(dialate, cv2.COLOR_GRAY2BGR)
    blur2Mat = cv2.cvtColor(blur2, cv2.COLOR_GRAY2BGR)
    
    
    masterFrameTop = np.concatenate((hsv, blur1, threshMat), axis=1)
    masterFrameBottom = np.concatenate((erodeMat, dialateMat, blur2Mat), axis=1)
    masterFrame = np.concatenate((masterFrameTop, masterFrameBottom), axis=0)
    
    return masterFrame
    
    
