#!/usr/bin/python

import rospy
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from numpy.linalg import lstsq
from rospy.numpy_msg import numpy_msg
import ros_numpy

from rospyext import *

class WallDetectorNode(Node):
    pub_detection = Publisher('wall_detection', PointStamped, queue_size=10)

    @Subscriber('laser/scan', numpy_msg(LaserScan))
    def right_scan_callback(self, scan):
        index = np.argmin(scan.ranges)
        min_dist = scan.ranges[index]
        theta = scan.angle_min + index*scan.angle_increment

        self.pub_detection.publish(
            PointStamped(
                scan.header,
                Point(
                    np.cos(theta) * min_dist,
                    np.sin(theta) * min_dist,
                    0
                )
            )
        )


if __name__ == "__main__":
    rospy.init_node("wall_detector_node")
    node = WallDetectorNode()
    rospy.spin()
