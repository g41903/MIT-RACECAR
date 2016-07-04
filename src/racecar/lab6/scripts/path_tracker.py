#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Transform, TransformStamped, Point, Quaternion, Vector3
from std_msgs.msg import Header
from nav_msgs.msg import Path
import numpy as np
import tf2_ros
from tf import transformations
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

import monte_carlo_localization as mcl
from rospyext import *
import ros_numpy
from lab6 import model


class PathTracker(Node):
    """
    A node that provides an interactive draggable marker, which is green if the robot fits without colliding, and false otherwise
    Also provide
    """

    pub_path = Publisher('/lab5/actual_path', Path, queue_size=1)
    base_frame = Param(str, default='base_link')
    map_frame = Param(str, default='map')

    def __init__(self):
        super(PathTracker, self).__init__()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path = Path()
        self.path.header.frame_id = self.map_frame

    @Timer(rospy.Duration(0.05))
    def timer_callback(self, event):

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.map_frame,
                source_frame=self.base_frame,
                time=event.current_real,
                timeout=rospy.Duration(0.05)
            ).transform
        except tf2_ros.LookupException:
            return
        except tf2_ros.ExtrapolationException:
            return
        except tf2_ros.ConnectivityException:
            return

        p = PoseStamped(
            pose=Pose(
                position=tf.translation,
                orientation=tf.rotation
            )
        )
        pn = ros_numpy.numpify(p.pose)
        if np.isnan(pn).any():
            rospy.logwarn("Got a bad tf of {!r}".format(p.pose))
            return

        if self.path.poses:
            lastn = ros_numpy.numpify(self.path.poses[-1].pose)
            if np.linalg.norm(lastn[:,-1] - pn[:,-1]) < 0.05:
                return

        self.path.poses.append(p)

        self.pub_path.publish(self.path)


if __name__ == '__main__':
    rospy.init_node("path_tracker")
    node = PathTracker()
    rospy.spin()
