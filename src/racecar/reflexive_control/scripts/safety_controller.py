#!/usr/bin/env python
"""
A safety controller node that acts as a kill-switch.
If something gets too close to the robot, the robot will stop
until the object moves out of the way.
This works by publishing to the ackermann_cmd_mux/input/safety topic.

This module is required for lab 3.
Winter Guerra
"""

import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped, Point
import numpy as np

from rospyext import *

class SafetyControllerNode(Node):
    # The closest an object is allowed to approach the front of the robot before it will stop.
    minimum_safe_distance = Param(float, default=0.6096) # 2 feet in meters

    def __init__(self):
        super(SafetyControllerNode, self).__init__()
        self.kill_switch_enabled = False
        rospy.Timer(rospy.Duration(.1), self.timer_callback)

    pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=10)

    @Subscriber('/tokyo/wall_detection/front', PointStamped)
    def update_killswitch_state_callback(self, ps):
        self.wall_theta = np.arctan2(ps.point.y,ps.point.x)
        self.wall_dist = np.sqrt(pow(ps.point.x,2)+pow(ps.point.y,2))
        self.kill_switch_enabled = (self.wall_dist <= self.minimum_safe_distance)

    # Check every second for if we are going to hit something.
    def timer_callback(self, event):
        if self.kill_switch_enabled:
            cmd = AckermannDriveStamped()
            self.pub.publish(cmd)



    def on_shutdown(self):
        # Send a stop command before exiting.
        self.pub.publish(AckermannDriveStamped())

if __name__ == '__main__':
    rospy.init_node("safety_controller_node")
    node = SafetyControllerNode()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
