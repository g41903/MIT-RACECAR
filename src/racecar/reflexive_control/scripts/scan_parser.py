#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection
from rospy.numpy_msg import numpy_msg
import numpy as np

from rospyext import *

def slice_scan(scan, slice):
    """Slice a laser scan like a python list"""
    start, end, step = slice.indices(len(scan.ranges))
    # TODO - update the timestamp in the header
    return LaserScan(
        header=scan.header,
        angle_min=scan.angle_min + scan.angle_increment * start,
        angle_max=scan.angle_min + scan.angle_increment * end,
        angle_increment=scan.angle_increment*step,
        time_increment=scan.time_increment*step,
        scan_time=scan.scan_time,
        range_min=scan.range_min,
        range_max=scan.range_max,
        ranges=scan.ranges[start:end],
        intensities=scan.intensities[start:end]
    )

def index_of_angle(scan, angle):
    """Calculates the index corresponding to a given scan angle"""
    return int((angle - scan.angle_min) / scan.angle_increment)

class ScanParserNode(Node):
    pub_right = Publisher('/tokyo/laser/scan/right', LaserScan, queue_size=10)
    pub_front = Publisher('/tokyo/laser/scan/front', LaserScan, queue_size=10)
    pub_left  = Publisher('/tokyo/laser/scan/left', LaserScan, queue_size=10)

    pub_cloud       = Publisher('/tokyo/laser/pointcloud', PointCloud2, queue_size=10)
    pub_cloud_right = Publisher('/tokyo/laser/pointcloud/right', PointCloud2, queue_size=10)
    pub_cloud_left  = Publisher('/tokyo/laser/pointcloud/left', PointCloud2, queue_size=10)

    right_angle_thresh = Param(float, default=-np.pi/6)
    left_angle_thresh = Param(float, default=np.pi/6)
    right_angle_far_thresh = Param(float, default=-np.pi/2)
    left_angle_far_thresh = Param(float, default=np.pi/2)

    projector = LaserProjection()

    @Subscriber('/racecar/laser/scan', numpy_msg(LaserScan), queue_size=1)
    def scan_callback(self, scan):
        far_index_right = int((self.right_angle_far_thresh-scan.angle_min)/scan.angle_increment)
        far_index_left = int((self.left_angle_far_thresh-scan.angle_min)/scan.angle_increment)

        index_right     = index_of_angle(scan, self.right_angle_thresh)
        index_left      = index_of_angle(scan, self.left_angle_thresh)

        scan_right = slice_scan(scan, np.s_[far_index_right:index_right   ])
        scan_front = slice_scan(scan, np.s_[    index_right:index_left    ])
        scan_left  = slice_scan(scan, np.s_[     index_left:far_index_left])

        cloud       = self.projector.projectLaser(scan)
        cloud_right = self.projector.projectLaser(scan_right)
        cloud_left  = self.projector.projectLaser(scan_left)

        self.pub_front.publish(scan_front)
        self.pub_right.publish(scan_right)
        self.pub_left.publish(scan_left)
        self.pub_cloud.publish(cloud)
        self.pub_cloud_right.publish(cloud_right)
        self.pub_cloud_left.publish(cloud_left)


if __name__ == '__main__':
    rospy.init_node("scan_parser_node")
    node = ScanParserNode()
    rospy.spin()
