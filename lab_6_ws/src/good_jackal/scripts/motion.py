#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
# Project 6 : Who's a good jackal
# By: Ian Wakely

import rospy
import numpy
from time import time
from geometry_msgs.msg import Twist
from good_jackal.msg import Tracked_Object

class BallFollower(object):
    turn_p_gain = 0.0035        #0.0035
    turn_i_gain = 0.027         #0.027

    speed_p_gain = 0.5
    speed_i_gain = 0.045

    def __init__(self, distance, timeout = 500):
        self.prev_t_i_val = 0
        self.prev_s_i_val = 0
        self.prev_time = time()

        self.target_distance = distance

        self.posted_turning = 0
        self.posted_speed = 0

        # init node
        rospy.init_node("motion", anonymous=False)
        self.object_sub = rospy.Subscriber("/good_jackal/object", Tracked_Object, self._ballLocation)
        self.motion_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():   
            twist = Twist()

            speed_decay = 0
            turn_decay = 0

            # do a timeout
            if (time() - self.prev_time) > timeout:
                #rospy.logwarn("No recent image?")
                self.prev_t_i_val = 0
                self.prev_s_i_val = 0
                
                speed_decay = 0.01
                turn_decay = 0.01
                
            else:
                speed_decay = 0.00035
                turn_decay = 0.00025

            if self.posted_turning > 0.035:
                self.posted_turning = self.posted_turning - turn_decay
            elif self.posted_turning < -0.035:
                self.posted_turning = self.posted_turning + turn_decay
            else:
                self.posted_turning = 0

            if self.posted_speed > 0.06:
                self.posted_speed = self.posted_speed - speed_decay
            elif self.posted_speed < -0.06:
                self.posted_speed = self.posted_speed + speed_decay
            else:
                self.posted_speed = 0
                
            twist.linear.x = self.posted_speed
            twist.angular.z = self.posted_turning
            
            #rospy.loginfo(twist)
            self.motion_pub.publish(twist)
            rate.sleep()

    def _findRange(self, pixels):
        return numpy.interp(pixels,
                [27, 30, 34, 39, 48, 62, 89],
                [2.4384, 2.1336, 1.8288, 1.524, 1.2192, 0.9144, 0.6096])

    def _ballLocation(self, data):
        curr_time = time()
        deltat = curr_time - self.prev_time
        rospy.loginfo("Delta T: {}".format(deltat))

        if data.r < 0:
            self.prev_t_i_val = 0
            self.prev_s_i_val = 0
            return

        t_p = -data.x * self.turn_p_gain
        t_i = (-data.x - self.prev_t_i_val) * deltat * self.turn_i_gain

        turning = t_p + t_i

        #rospy.loginfo("Turning: {}".format(turning))

        # save latest value for next I eval
        self.prev_t_i_val = -data.x
        self.prev_time = curr_time

        distance = self._findRange(data.r)
        distance_err = distance - self.target_distance
        #rospy.loginfo("Distance: {} (Err: {}, R: {})".format(distance, distance_err, data.r))

        s_p = distance_err * self.speed_p_gain
        s_i = distance_err * deltat * self.speed_i_gain

        speed = s_p + s_i

        #rospy.loginfo("Speed: {}".format(speed))

        self.posted_turning = turning
        self.posted_speed = speed

# standard ros boilerplate
if __name__ == "__main__":
    try:
        motion = BallFollower(1.5, 2)
    except rospy.ROSInterruptException:
        pass
