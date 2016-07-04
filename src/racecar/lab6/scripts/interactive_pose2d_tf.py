#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, TransformStamped, Quaternion, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback

import numpy as np
import tf2_ros
from tf import transformations
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from rospyext import *
import ros_numpy


class InteractivePose2DTF(Node):
    """
    A node that provides an interactive draggable marker, which publishes a tf
    """

    pub_marker = Publisher('/marker', MarkerArray, queue_size=1)

    world_frame = Param(str, default='map')
    target_frame = Param(str, default='base_link')

    def __init__(self):
        self.transform = None

        super(InteractivePose2DTF, self).__init__()

        self.pub_tf = tf2_ros.TransformBroadcaster()

        self.interact = InteractiveMarkerServer("interactive_markers")

        self.marker_pose = Pose()

        self._set_marker(ColorRGBA(0.5, 0.5, 0.5, 0.5))

    def _set_marker(self, color):
        self.interact.clear()

        marker = InteractiveMarker(
            header=Header(
                frame_id=self.world_frame
            ),
            scale=1,
            name="Robot",
            controls=[
                InteractiveMarkerControl(
                    interaction_mode=InteractiveMarkerControl.MOVE_ROTATE,
                    orientation=Quaternion(*transformations.quaternion_from_euler(0, np.pi/2, 0)),
                    markers=[
                        Marker(
                            type=Marker.ARROW,
                            scale=Vector3(0.5, 0.05, 0.05),
                            color=color,
                            pose=Pose(
                                orientation=Quaternion(0, 0, 0, 1)
                            )
                        )
                    ]
                )
            ],
            pose=self.marker_pose
        )

        self.interact.insert(marker, self.processFeedback)
        self.interact.applyChanges()

    def processFeedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return

        self.marker_pose = feedback.pose


    @Timer(rospy.Duration(0.1))
    def timer_tf_pub(self, event):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.target_frame
        t.transform.translation = self.marker_pose.position
        t.transform.rotation = self.marker_pose.orientation
        self.pub_tf.sendTransform(t)

    @Subscriber('~color', ColorRGBA)
    def sub_color(self, color):
        self._set_marker(color)

    @Subscriber('/initialpose', PoseWithCovarianceStamped)
    def sub_initialpose(self, pose):
        """ The default topic used by rviz """
        self.marker_pose = pose.pose.pose
        self._set_marker(ColorRGBA(0.5, 0.5, 0.5, 0.5))

if __name__ == '__main__':
    rospy.init_node("interactive_pose2d_tf")
    node = InteractivePose2DTF()
    rospy.spin()

