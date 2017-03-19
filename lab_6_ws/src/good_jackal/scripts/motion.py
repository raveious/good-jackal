#!/usr/bin/env python

# Intro to Robotics - EE5900 - Spring 2017
# Project 6 : Who's a good jackal
# By: Ian Wakely

import rospy
from time import time
from geometry_msgs.msg import Twist
from good_jackal.msg import Tracked_Object

class BallFollower(object):
    turn_p_gain = 0.003
    turn_i_gain = 0.0

    speed_p_gain = 0.0
    speed_i_gain = 0.0

    def __init__(self, radius = 18):
        # init node
        rospy.init_node("motion", anonymous=False)
        self.object_sub = rospy.Subscriber("/good_jackal/object", Tracked_Object, self._ballLocation)
        self.motion_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.prev_t_i_val = 0
        self.prev_d_i_val = 0
        self.prev_time = time()

        self.target_radius = radius

        # standard node spin
        rospy.spin()

    def _ballLocation(self, data):
        curr_time = time()
        rospy.loginfo(data)

        # clear stale I values
        # if curr_time - self.prev_time > 3000:
        #     self.prev_i_val = 0

        t_p = -data.x * self.turn_p_gain
        t_i = (-data.x - self.prev_t_i_val) * (curr_time - self.prev_time) * self.turn_i_gain
        turning = t_p + t_i

        # save latest value for next I eval
        self.prev_t_i_val = -data.x


        distance = self.target_radius - data.r


        s_p = data.r * self.speed_p_gain
        s_i = 0

        speed = s_p + s_i

        rospy.loginfo("Turning: {}".format(turning))
        rospy.loginfo("Distance: {}".format(distance))
        rospy.loginfo("Speed: {}".format(speed))

        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = turning

        rospy.logdebug("Twist: ", twist)
        self.motion_pub.publish(twist)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        motion = BallFollower()
    except rospy.ROSInterruptException:
        pass
