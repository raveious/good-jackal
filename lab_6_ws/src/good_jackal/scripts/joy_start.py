#!/usr/bin/env python

# joy_trigger_start.py
# Use joystick input to launch exploration nodes in jackal
# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #5

#       Project #5 Group #3
#         Ian (Team Lead)
#            Phillip
#            Akhil
#
# /blueooth_teleop/joy
# sensor_msgs/Joy
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32[] axes
# int32[] buttons
#
# axes: [ lft - l/r, lft - up/down, L2 (1/-1), rgt - l/r, rgt - u/d, R2 (1/-1)]
# buttons: [ x, circle, sq, tri, L1, R1, share, options, play, L3, R3, DL, DR, DU, DD]
#

import rospy
import roslaunch
from sensor_msgs.msg import Joy

class ControlledStart(object):
    def __init__(self, package, executable):
        rospy.init_node("controller_starter", anonymous=False)
        sub = rospy.Subscriber("/bluetooth_teleop/joy", Joy, self._joy_callback)

        # init launch api
        self.launch_api = roslaunch.scriptapi.ROSLaunch()
        self.launch_api.start()
        self.node = roslaunch.core.Node(package, executable)

        self.proc = None
        self.start = False

        while not rospy.is_shutdown():
            if self.start:
                self.proc = self.launch_api.launch(self.node)
                self.start = False

    def _joy_callback(self, data):
        x, circ, sq, tri, L1, R1, share, options, p4, L3, R3, DL, DR, DU, DD = data.buttons
        llr, lud, L2, rlr, rud, R2 = data.axes

        if circ == 1 and not self.proc:
            rospy.loginfo("Process started")
            self.start = True
        elif x == 1 and self.proc:
            rospy.loginfo("Process stopped")
            self.proc.stop()
            self.proc = None

if __name__ == "__main__":
    try:
        run = ControlledStart("good_jackal", "motion.py")
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
