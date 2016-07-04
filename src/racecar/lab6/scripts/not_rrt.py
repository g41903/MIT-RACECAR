#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Point
from std_msgs.msg import Header
from lab6 import model

import tf2_ros
from tf import transformations

from rospyext import *
import ros_numpy

import numpy as np

def make_pose(point3, orientation=Quaternion(0, 0, 0, 1)):
    return PoseStamped(
        header=Header(frame_id="map"),
        pose=Pose(
            position=Point(*point3),
            orientation=orientation
        )
    )

def make_path(points, **kwargs):
    step_size = kwargs.pop('step_size', 0.1)
    path = []
    last_point = None
    points_np = [np.array([p[0], p[1], 0]) for p in points]
    for point in points_np:
        if last_point is not None:
            diff = point - last_point

            diff_len = np.linalg.norm(diff)
            angle = np.arctan2(diff[1], diff[0])
            orientation = Quaternion(
                0,
                0,
                np.sin(angle/2),
                np.cos(angle/2)
            )

            step_n = diff_len // step_size
            fracs = np.arange(step_n+1) * step_size / diff_len
            fracs = fracs[::-1]  # order from 1 to 0

            for f in fracs:
                path.append(make_pose(last_point*f + point*(1-f), orientation=orientation))
        else:
            path.append(make_pose(point))

        last_point = point
    return path

class NotRRTNode(Node):

    pub_path = Publisher('~path', Path, queue_size=1)

    def __init__(self):
        super(NotRRTNode, self).__init__()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path = None
        self.goal_changed = False

    @Subscriber('~goal', PoseStamped)
    def sub_pose(self, pose):
        rospy.loginfo("notrrt: received goal")
        trans = self.tf_buffer.lookup_transform(
                target_frame="map",
                source_frame="base_link",
                time=pose.header.stamp,
                timeout=rospy.Duration(1)
            )
        at = model.pose_from_ros(trans)
        points = [(at.x,at.y),(pose.pose.position.x, pose.pose.position.y)]
        rospy.loginfo("making path between: {}".format(points))
        self.path = make_path(points)
        self.goal_changed = True

    @Timer(rospy.Duration(0.1))
    def timer_callback(self, event):
        if self.path is None:
            rospy.logwarn("rrt planner: No goal")
            return

        if not self.goal_changed:
            return

        self.goal_changed = False

        path = Path()
        path.header.frame_id = 'map'
        path.poses = self.path
        self.pub_path.publish(path)
        rospy.loginfo('Planned!')

if __name__ == '__main__':
    rospy.init_node("not_rrt_node")
    node = NotRRTNode()
    rospy.spin()
