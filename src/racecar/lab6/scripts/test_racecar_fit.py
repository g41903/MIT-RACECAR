#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Point, Quaternion, Vector3
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from tf import transformations
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

import monte_carlo_localization as mcl
from rospyext import *
import ros_numpy
from lab6 import model


class RacecarFitTest(Node):
    """
    A node that provides an interactive draggable marker, which is green if the robot fits without colliding, and false otherwise
    Also provide
    """

    pub_color = Publisher('~color', ColorRGBA, queue_size=1)

    base_frame = Param(str, default='base_link')

    def __init__(self):
        self.map = None

        super(RacecarFitTest, self).__init__()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    @Subscriber('~map', rospy.numpy_msg.numpy_msg(OccupancyGrid))
    def sub_map(self, map):
        self.map = mcl.Map(map)

    @Timer(rospy.Duration(0.05))
    def timer_callback(self, event):
        if self.map is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.map.frame,
                source_frame=self.base_frame,
                time=event.current_real,
                timeout=rospy.Duration(0.05)
            ).transform
        except tf2_ros.LookupException:
            return
        except tf2_ros.ExtrapolationException:
            return

        x = tf.translation.x
        y = tf.translation.y
        _, _, theta = transformations.euler_from_quaternion(
            ros_numpy.numpify(tf.rotation)
        )

        at = model.pose(x, y, theta)
        ok = model.robot_fits(self.map, at, center_only=False)

        print ok

        self.pub_color.publish(ColorRGBA(0, 1, 0, 0.5) if ok else ColorRGBA(1, 0, 0, 0.5))


if __name__ == '__main__':
    rospy.init_node("racecar_fit_test")
    node = RacecarFitTest()
    rospy.spin()
