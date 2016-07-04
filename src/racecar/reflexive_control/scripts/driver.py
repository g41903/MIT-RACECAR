#!/usr/bin/env python
"""
A responsible racecar driver.
"""

import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Point
import numpy as np

from rospyext import *

class DriverNode(Node):
    target_theta = Param(float, default=-np.pi/2)
    target_dist = Param(float, default=.3)
    gain_theta = Param(float, default=.8)
    gain_dist = Param(float, default=-.4)


    def __init__(self):
        super(DriverNode, self).__init__()
        self.object_close = False
        self.wall_theta = None
        self.wall_dist = None
        rospy.Timer(rospy.Duration(.1), self.timer_callback)

    pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)

    @Subscriber('/tokyo/object_detection', Bool)
    def object_detection_callback(self, msg):
        self.object_close = msg.data

    @Subscriber('/tokyo/wall_detection/right', PointStamped)
    def wall_detection_callback(self, ps):
        self.wall_theta = np.arctan2(ps.point.y, ps.point.x)
        self.wall_dist = np.hypot(ps.point.x, ps.point.y)
        #rospy.loginfo(self.wall_theta)
        #rospy.loginfo(self.wall_dist)

    def timer_callback(self, event):
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = 0.0
        if self.object_close:
            # Go sort of backwards if there is something in the way.
            cmd.drive.steering_angle = -np.pi/12
            cmd.drive.speed = -0.3
        elif self.wall_theta is not None:
            # Charge ownards (and slightly sideways) if it's all clear.
            cmd.drive.steering_angle = (
                self.gain_theta * (self.wall_theta - self.target_theta) +
                self.gain_dist * (self.wall_dist - self.target_dist)
            )
            cmd.drive.speed = 0.6
        else:
            pass
        self.pub.publish(cmd)

    def on_shutdown(self):
        # Send a stop command before exiting.
        self.pub.publish(AckermannDriveStamped())

if __name__ == '__main__':
    rospy.init_node("driver_node")
    node = DriverNode()
    rospy.on_shutdown(lambda: node.on_shutdown())
    rospy.spin()
