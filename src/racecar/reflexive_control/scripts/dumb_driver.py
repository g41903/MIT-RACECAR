#!/usr/bin/env python
"""
A racecar that goes forward forever.
Use with caution.
"""

import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from math import pi
import time

from rospyext import *

class DumbDriverNode(Node):
    pub = rospy.Publisher("/racecar/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

    def __init__(self):
        super(DumbDriverNode, self).__init__()
        rospy.Timer(rospy.Duration(.1), self.timer_callback)
        rospy.loginfo("Start")

    def timer_callback(self, event):
        rospy.loginfo("Tick")
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = pi/8.
        cmd.drive.speed = 1.0
        self.pub.publish(cmd)

    def on_shutdown(self):
        # Send a stop command before exiting.
        self.pub.publish(AckermannDriveStamped())

if __name__ == "__main__":
    rospy.init_node("dumb_driver_node")
    node = DumbDriverNode()
    rospy.on_shutdown(lambda: node.on_shutdown())
    rospy.spin()
