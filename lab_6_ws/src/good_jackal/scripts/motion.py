#!/usr/bin/env python

#subscribes data from camera and publishes it to cmd_vel
#to run the jackal bot according to the tuned data

# Intro to Robotics - EE5900 - Spring 2017
# Project 6 : Who's a good jackal

# Team 3

import rospy
import std_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
#something for camera

def callback(data):
    global prop_const
    global intr_const
    # PI controller tunning

def position_sub():
    rospy.Subscriber('tracked_pos',Int32,callback) #for openCv

def jackal_move():
    pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.init_node('jackal_move',anonymous=True)

    rate = rospy.Rate(50) #updates at 1hz rate
    position_sub()
