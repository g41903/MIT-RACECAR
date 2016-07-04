#!/usr/bin/env python
from __future__ import division

import cProfile
import os

import rospy
import rospkg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, Point, Vector3, PoseStamped, Quaternion
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import tf2_ros
from tf import transformations

import monte_carlo_localization as mcl

from rospyext import *
import ros_numpy

from lab6 import AckermannRRT, OmniRRT, AckermannRRT2, RRT, model, rrtutils


class RRTNode(Node):

    pub_vis_tree = Publisher('/vis/tree', MarkerArray, queue_size=1)
    pub_vis_tree_wip = Publisher('/vis/tree_wip', MarkerArray, queue_size=1)
    pub_vis_sample = Publisher('/vis/new_sample', Marker, queue_size=1)

    # WARNING. DEBUG ONLY! VERY CPU INTENSIVE!
    pub_merged_map = Publisher('/vis/merged_map', ros_numpy.numpy_msg(OccupancyGrid), queue_size=1)


    pub_path = Publisher('~path', Path, queue_size=1)

    base_frame = Param(str, default='base_link')
    replan = Param(str, default='always')
    allow_reverse = Param(bool, default=False)

    reroute_threshold = Param(float, default=0.2)
    debug_merged_map = Param(bool, default=False)
    use_ackermann_rrt = Param(bool, default=False)
    use_ackermann_rrt2 = Param(bool, default=False)
    do_profile = Param(bool, default=False)

    def __init__(self):
        self.map = None

        super(RRTNode, self).__init__()

        assert self.replan in ('on_goal_change', 'always')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal = None
        self.goal_changed = False

        if self.use_ackermann_rrt2:
            rospy.loginfo("Using AckermannRRT2")
        elif self.use_ackermann_rrt:
            rospy.loginfo("Using AckermannRRT")
        else:
            rospy.loginfo("Using OmniRRT")

        self.rrt = None

    def start_over(self, pose):
        hooks = rrtutils.RvizHooks(
            pub_sample=self.pub_vis_sample,
            pub_tree_wip=self.pub_vis_tree_wip,
            frame=self.map.frame
        )
        if self.use_ackermann_rrt2:
            self.rrt = AckermannRRT2(self.map, pose, self.goal, allow_reverse=self.allow_reverse, hooks=hooks)
        elif self.use_ackermann_rrt:
            self.rrt = AckermannRRT(self.map, pose, self.goal, allow_reverse=self.allow_reverse, hooks=hooks)
        else:
            self.rrt = OmniRRT(self.map, pose, self.goal, hooks=hooks)


    @Subscriber('~goal', PoseStamped)
    def sub_pose(self, pose):
        if self.use_ackermann_rrt2:
            goal = model.pose_from_ros(pose)
        else:
            goal = ros_numpy.numpify(pose.pose.position)[:2]
        if self.goal is None:
            self.goal = goal
        else:
            # update in place, so the rrt gets the new value automatically
            self.goal[...] = goal
        self.goal_changed = True
        rospy.loginfo("Updated goal to {}".format(self.goal))

    @Subscriber('~map', rospy.numpy_msg.numpy_msg(OccupancyGrid))
    def sub_map(self, map):
        self.map = mcl.HybridMap(map)

    @Subscriber('~local_map', rospy.numpy_msg.numpy_msg(OccupancyGrid))
    def sub_localmap(self, map):
        if self.map is not None:
            self.map.merge_transient(mcl.Map(map))

    # For debugging, we should publish our merged map every once in a while
    @Timer(rospy.Duration(2.0))
    def map_timer_callback(self, event):
        if self.debug_merged_map:
            map_msg = ros_numpy.msgify(ros_numpy.numpy_msg(OccupancyGrid), self.map.grid, info=self.map.info)
            map_msg.header = Header()
            map_msg.header.stamp = rospy.Time.now()
            map_msg.header.frame_id = self.map.frame

            self.pub_merged_map.publish(map_msg)

    @Timer(rospy.Duration(0.1))
    def timer_callback(self, event):
        if self.map is None:
            rospy.logwarn("No map")
            return
        if self.goal is None:
            rospy.logwarn("No goal")
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.map.frame,
                source_frame=self.base_frame,
                time=event.current_real,
                timeout=rospy.Duration(0.1)
            ).transform
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error getting robot pose", exc_info=True)
            return

        # skip times when we are already off-map
        curr = self.map[self.map.index_at(ros_numpy.numpify(tf.translation))]
        if curr > 0 or curr is np.ma.masked:
            rospy.logwarn("Current node is not valid")
            return

        # don't do anything if we requested single-shot planning
        if self.replan == 'on_goal_change' and not self.goal_changed:
            return
        self.goal_changed = False

        # determine our pose
        if self.use_ackermann_rrt or self.use_ackermann_rrt2:
            pose = model.pose_from_ros(tf)
        else:
            pose = ros_numpy.numpify(tf.translation)[:2]

        # either reuse or throw out the RRT
        if not self.rrt:
            self.start_over(pose)
        else:
            if self.use_ackermann_rrt2:
                pose_lookup = pose
            elif self.use_ackermann_rrt:
                pose_lookup = pose.xy
            else:
                pose_lookup = pose

            # decide whether we can reroot the rrt
            nearest, dist, _ = self.rrt.get_nearest_node(pose_lookup, exchanged=True)
            if dist > self.reroute_threshold:
                rospy.loginfo("Nearest node is {}m away, ignoring".format(dist))
                self.start_over(pose)
            else:
                rospy.loginfo("Nearest node is {}m away, rerooting and pruning".format(dist))
                nearest.make_root()
                self.rrt.prune()


        self.pub_vis_tree.publish(rrtutils.delete_marker_array_msg)

        if self.do_profile:
            # run and profile the rrt
            pr = cProfile.Profile()
            pr.enable()

        path = list(self.rrt.run())

        if self.do_profile:
            pr.disable()
            pr.dump_stats(os.path.join(
                rospkg.RosPack().get_path('lab6'), 'rrt_stats.txt'
            ))

        # process the results
        self.send_debug_tree(path)
        self.publish_path(path)
        rospy.loginfo('Planned!')

    def publish_path(self, edges):
        # Send the path to the path_follower node
        cmd = Path()

        poses = np.concatenate([
            edge.interpolate(step=0.2)[:-1]
            for edge in edges
        ] + [[edges[-1].dest]])

        cmd.poses = [
            rrtutils.pose_for_node(pose) for pose in poses
        ]
        cmd.header.frame_id = self.map.frame
        #rospy.loginfo("New Poses {}.".format(cmd.poses))
        self.pub_path.publish(cmd)

    def send_debug_tree(self, path=[]):
        """
        Renders the tree. Edges are red unless they appear in path, in which case they are green
        """
        msg = MarkerArray()

        for i, edge in enumerate(path):

            marker = rrtutils.marker_for_edge(edge, id=i, frame=self.map.frame)

            marker.color.r *= 0.5
            marker.color.b *= 0.5
            marker.color.g *= 0.75
            marker.color.a = 1
            marker.scale.x *= 2

            msg.markers.append(marker)

        self.pub_vis_tree.publish(msg)


if __name__ == '__main__':
    rospy.init_node("rrt_node")
    node = RRTNode()
    rospy.spin()
