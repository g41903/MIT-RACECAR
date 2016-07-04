import numpy as np

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf import transformations

from . import AckermannRRT, RRT

delete_marker_array_msg = MarkerArray(markers=[
    Marker(action=3) # Marker.DELETEALL, which is missing in indigo
])

def point_for_node(node):
    if isinstance(node, (np.record, np.recarray)):
        node = node.xy

    return Point(node[0], node[1], 0)


def pose_for_node(node):
    '''
    Converts nodes into Poses.
    '''
    pose_stamped = PoseStamped()
    point = point_for_node(node)
    pose_stamped.pose.position = point
    if isinstance(node, (np.record, np.recarray)):
        pose_stamped.pose.orientation = Quaternion(
            *transformations.quaternion_from_euler(0, 0, node.theta)
        )
    return pose_stamped


def marker_for_edge(edge, id, frame):
    if isinstance(edge, AckermannRRT.Edge):
        return Marker(
            header=Header(
                frame_id=frame
            ),
            ns="Tree",
            id=id,
            type=Marker.LINE_STRIP,
            color=ColorRGBA(1, 1, 0, 0.5) if edge.is_forward else ColorRGBA(0, 1, 1, 0.5),
            points=[
                point_for_node(p)
                for p in edge.interpolate(step=0.1)
            ],
            scale=Vector3(0.02, 0, 0)  # line thickness
        )

    else:
        return Marker(
            header=Header(
                frame_id=frame
            ),
            ns="Tree",
            id=id,
            type=Marker.ARROW,
            color=ColorRGBA(1, 1, 0, 0.5),
            points=[point_for_node(edge.src), point_for_node(edge.dest)],
            scale=Vector3(0.025, 0.05, 0)  # shaft diameter, head diameter
        )


class RvizHooks(RRT.Hooks):
    """ A default set of hooks that publishes messages for use in rviz """
    def __init__(self, pub_tree_wip, pub_sample, frame):
        self.pub_tree_wip = pub_tree_wip
        self.pub_sample = pub_sample
        self.frame = frame

        self.tree_msg = MarkerArray()
        self.sample_marker = Marker(
            header=Header(
                frame_id=self.frame,
            ),
            ns='sample',
            type=Marker.SPHERE,
            color=ColorRGBA(1, 1, 0, 1),
            scale=Vector3(0.2, 0.2, 0.2),
            lifetime=rospy.Duration(0.5)
        )
        self.origin_marker = Marker(
            header=Header(
                frame_id=self.frame,
            ),
            ns='origin',
            type=Marker.SPHERE,
            color=ColorRGBA(1, 0, 0, 0.5),
            scale=Vector3(0.1, 0.1, 0.1)
        )
        self.full_publish_period = rospy.Duration(10)
        self.next_full_publish = rospy.Time.now() + self.full_publish_period
        # clear any old markers
        self.pub_tree_wip.publish(delete_marker_array_msg)

    def pre_extend(self, rrt, src, dest):
        m = self.sample_marker
        m.header.stamp = rospy.Time.now()
        m.pose.position = point_for_node(dest)
        self.pub_sample.publish(m)

    def post_extend(self, rrt, edge):
        new_marker = marker_for_edge(edge, id=len(self.tree_msg.markers), frame=self.frame)
        self.tree_msg.markers.append(new_marker)

        # only publish the full tree every 5 seconds
        now = rospy.Time.now()
        if now > self.next_full_publish:
            self.next_full_publish = rospy.Time.now() + self.full_publish_period
            self.pub_tree_wip.publish(delete_marker_array_msg)
            self.pub_tree_wip.publish(self.tree_msg)
        else:
            self.pub_tree_wip.publish(MarkerArray(markers=[new_marker]))

    def rerooted(self, rrt, old_root, new_root):
        # rebuild the entire visualization after a reroot
        self.tree_msg.markers = [
            marker_for_edge(edge, i, frame=self.frame)
            for i, edge in enumerate(rrt.edges)
        ]
        self.pub_tree_wip.publish(delete_marker_array_msg)
        self.pub_tree_wip.publish(self.tree_msg)
        self.next_full_publish = rospy.Time.now() + self.full_publish_period

        m = self.origin_marker
        m.pose.position = point_for_node(rrt.origin)
        self.pub_sample.publish(m)
