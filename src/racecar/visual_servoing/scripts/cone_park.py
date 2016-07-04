#!/usr/bin/env python
"""
A responsible racecar driver.
"""

import rospy
from std_msgs.msg import Bool, Duration
from ackermann_msgs.msg import AckermannDriveStamped
from visual_servoing.msg import CameraObjectsStamped
import numpy as np

from rospyext import *

class DriverNode(Node):
    target_dist = Param(float, default=0.3)
    gain_angle = Param(float, default=-.25)
    gain_speed = Param(float, default=-2)

    pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=10)

    # How old the consumed message is.
    pub_delay = Publisher('~debug/delay', Duration, queue_size=1)

    @Subscriber('~cone', CameraObjectsStamped, queue_size=1)
    def sub_cone(self, msg):
        header_age = rospy.rostime.Time.now() - msg.header.stamp
        self.pub_delay.publish(Duration(header_age))

        cmd = AckermannDriveStamped()

        if msg.objects:
            cone = msg.objects[0]
            cmd.drive.steering_angle = self.gain_angle*cone.center.x
            cmd.drive.speed = self.gain_speed*(cone.size.x-self.target_dist)
        else:
            # no cone
            pass

        self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("driver")
    node = DriverNode()
    rospy.on_shutdown(lambda: node.on_shutdown())
    rospy.spin()

'msg.objects[0].center.x if msg.objects else 0'
