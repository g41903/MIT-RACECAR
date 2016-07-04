#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np

from rospyext import *

class ObjectDetectorNode(Node):
    pub_detect = Publisher('/tokyo/object_detection', Bool, queue_size=10)

    distance_thresh  = Param(float, default=0.25)
    count_thresh     = Param(int, default=0)
    detect_angle_min = Param(float, default=-1*np.pi/4)
    detect_angle_max = Param(float, default=np.pi/4)

    @Subscriber('/tokyo/laser/scan/front', numpy_msg(LaserScan))
    def sub_scan(self, scan):
        # process scan data
        #index_min = int((self.detect_angle_min-scan.angle_min)/scan.angle_increment)
        #index_max = int((self.detect_angle_max-scan.angle_min)/scan.angle_increment)

        ranges = scan.ranges#[index_min:index_max]c
        out_of_range = (ranges >= scan.range_max) | (ranges <= scan.range_min)

        close_count = np.sum(
            ~out_of_range & (ranges < self.distance_thresh)
        )

        is_object = close_count > self.count_thresh

        rospy.loginfo(is_object)
        self.pub_detect.publish(is_object)


if __name__ == '__main__':
    rospy.init_node("object_detector_node")
    node = ObjectDetectorNode()
    rospy.spin()
