#!/usr/bin/env python
"""
Find the best angle to wander towards.
Finds the angle which is in front of the car and towards which stuff is
farthest away according to the laser.
"""

import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float32, Int32, Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection
from rospy.numpy_msg import numpy_msg
import numpy as np
import scipy.ndimage

from rospyext import *

class WanderFinderNode(Node):
    close_thresh = Param(float, default=1.2)

    pub_point = Publisher('~point', PointStamped, queue_size=1)
    pub_angle = Publisher('~angle', Float32, queue_size=1)
    pub_too_close = Publisher('~too_close', Bool, queue_size=1)
    pub_blur = Publisher('~blur', Int32, queue_size=1)

    @Subscriber('/racecar/laser/scan', numpy_msg(LaserScan), queue_size=1)
    def scan_callback(self, scan):
        # Calculate angles.
        angles = scan.angle_min + np.arange(scan.ranges.shape[0]) * scan.angle_increment

        # Blur ranges.
        blur_width_angle = np.deg2rad(30)
        blur_width = blur_width_angle / scan.angle_increment
        use_ranges = scipy.ndimage.filters.gaussian_filter(scan.ranges, blur_width)

        # Nan angles which are out of steering range.
        # Angles go from -2.something to +2.something.
        min_angle = np.deg2rad(-60)
        max_angle = np.deg2rad(60)
        use_ranges[np.where((angles < min_angle) | (angles > max_angle))] = np.nan

        index = np.nanargmax(use_ranges)
        dist = scan.ranges[index]
        too_close = dist < self.close_thresh
        theta = angles[index]

        view_dist = 1.
        self.pub_point.publish(
            PointStamped(
                scan.header,
                Point(
                    np.cos(theta) * view_dist,
                    np.sin(theta) * view_dist,
                    0
                )
            )
        )
        self.pub_angle.publish(Float32(theta))
        self.pub_blur.publish(Int32(blur_width))
        self.pub_too_close.publish(Bool(too_close))


if __name__ == '__main__':
    rospy.init_node("wander_finder_node")
    node = WanderFinderNode()
    rospy.spin()
