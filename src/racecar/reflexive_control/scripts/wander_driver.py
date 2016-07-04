#!/usr/bin/env python
"""
Drives towards the angle prescribed to wander_finder.
"""

import rospy
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Point
import numpy as np

from rospyext import *

class WanderDriverNode(Node):
    speed = Param(float, default=1.0)

    def __init__(self):
        super(WanderDriverNode, self).__init__()
        self.go = False
        self.angle = 0
        self.too_close = False
        rospy.Timer(rospy.Duration(.1), self.timer_callback)

    pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)

    @Subscriber('/tokyo/wander_angle', Float32)
    def wander_angle_callback(self, msg):
        self.go = True
        self.angle = msg.data

    @Subscriber('/tokyo/wander_too_close', Bool)
    def wander_too_close_callback(self, msg):
        self.go = True
        self.too_close = msg.data

    def timer_callback(self, event):
        cmd = AckermannDriveStamped()
        if self.go:
            if not self.too_close:
                cmd.drive.steering_angle = self.angle
                cmd.drive.speed = self.speed
            else:
                cmd.drive.steering_angle = np.deg2rad(30)
                cmd.drive.speed = -0.4
        else:
            cmd.drive.steering_angle = 0.0
            cmd.drive.speed = 0.0

        self.pub.publish(cmd)

    def on_shutdown(self):
        # Send a stop command before exiting.
        self.pub.publish(AckermannDriveStamped())

if __name__ == '__main__':
    rospy.init_node("wander_driver_node")
    node = WanderDriverNode()
    rospy.on_shutdown(lambda: node.on_shutdown())
    rospy.spin()
