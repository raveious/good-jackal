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
    turn_p_gain = 0.003
    turn_i_gain = 0.02
    turn_d_gain = 0.0

    speed_p_gain = 0.0
    speed_i_gain = 0.0

    def __init__(self, distance, timeout = 500):
        # init node
        rospy.init_node("motion", anonymous=False)
        self.object_sub = rospy.Subscriber("/good_jackal/object", Tracked_Object, self._ballLocation)
        self.motion_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.prev_t_i_val = 0
        self.prev_s_i_val = 0
        self.prev_time = time()

        self.target_distance = distance

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if time() - self.prev_time > timeout:
                rospy.logwarn("No recent image?")
                self.motion_pub.publish(Twist())
                self.prev_t_i_val = 0
                self.prev_s_i_val = 0

            rate.sleep()

    def _findRange(self, pixels):
        return numpy.interp(pixels,
                [27, 30, 34, 39, 48, 62, 89],
                [2.4384, 2.1336, 1.8288, 1.524, 1.2192, 0.9144, 0.6096])

    def _ballLocation(self, data):
        curr_time = time()
        deltat = curr_time - self.prev_time
        rospy.loginfo("Delta T: {}".format(deltat))

        t_p = -data.x * self.turn_p_gain
        t_i = (-data.x - self.prev_t_i_val) * deltat * self.turn_i_gain

        # Make a dead zone...
        if abs(data.x) < 3:
            turning = 0
        else:
            turning = t_p + t_i

        rospy.loginfo("Turning: {}".format(turning))

        # save latest value for next I eval
        self.prev_t_i_val = -data.x
        self.prev_time = curr_time

        distance = self._findRange(data.r)
        distance_err = distance - self.target_distance
        rospy.loginfo("Distance: {} (Err: {}, R: {})".format(distance, distance_err, data.r))

        s_p = distance_err * self.speed_p_gain
        s_i = 0

        speed = s_p + s_i

        rospy.loginfo("Speed: {}".format(speed))

        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = turning

        rospy.logdebug(twist)
        self.motion_pub.publish(twist)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        motion = BallFollower(1.5, 500)
    except rospy.ROSInterruptException:
        pass
