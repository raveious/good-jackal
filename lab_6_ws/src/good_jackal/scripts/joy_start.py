#!/usr/bin/env python

import rospy
import roslaunch
import sys
from sensor_msgs.msg import Joy

class ControlledStart(object):
    def __init__(self, package, executable):
        # init launch api
        self.launch_api = roslaunch.scriptapi.ROSLaunch()
        self.launch_api.start()
        self.node = roslaunch.core.Node(package, executable)

        self.proc = None
        self.start = False

        rospy.init_node("controller_starter", anonymous=False)
        sub = rospy.Subscriber("/bluetooth_teleop/joy", Joy, self._joy_callback)

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
        run = ControlledStart(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
