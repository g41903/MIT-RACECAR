#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf import transformations
import monte_carlo_localization as mcl
from rospyext import *
from ros_numpy import numpify, msgify, numpy_msg


import rospkg
from lab6 import model

class PlanNode(Node):
    max_diff = Param(int, default=20)
    target_pose_pub = Publisher('~target_pose', PoseStamped, queue_size=1)

    TARGET_DIST = 2
    TURN_RADIUS = 1
    UPDATE_HZ = 4
    CLOSE_THRESH = 5
    TARGET_OFFSET = 0 # the location on the car that we try to match with reality

    def __init__(self):
        rospy.loginfo("created node")
        self.global_mcl_map = None
        self.map_frame = None
        self.scan_target_pose = None
        self.map_target_pose = None
        self.marker_target_pose = None
        self.path = None
        self.path_index = None
        self.pub_path_point = True
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        super(PlanNode, self).__init__()

    @Subscriber('~map', numpy_msg(OccupancyGrid), queue_size=1)
    def sub_map(self, map):
        self.map_frame = map.header.frame_id
        rospy.loginfo("received map")

    @Subscriber('~path', Path, queue_size=1)
    def sub_path(self, path):
        self.path = path.poses
        self.path_index = 0
        self.pub_path_point = True
        rospy.loginfo([(i.pose.position.x,i.pose.position.y) for i in path.poses])
        rospy.loginfo("received path")

    @Subscriber('~scan', numpy_msg(LaserScan), queue_size=1)
    def sub_scan(self, scan):
        if self.global_mcl_map is None or self.map_frame is None:
            return

        delay = (scan.header.stamp-rospy.Time.now()).to_sec()
        #rospy.loginfo('scan stamp, now = {}'.format(delay))
        if delay < -.2:
            return

        far = scan.ranges>10
        buffer_size = 10
        far_buffered = np.convolve(far, np.ones(buffer_size).astype(bool))[0:len(far)]
        changes = np.where(far_buffered[:-1] != far_buffered[1:])[0]
        if len(changes) == 0:
            return
        group_sizes = np.diff(changes)[::2]
        max_group = np.argmax(group_sizes)
        target_index = (changes[2*max_group]+changes[2*max_group+1]-buffer_size)/2
        rel_angle = scan.angle_min + target_index*scan.angle_increment

        trans = self.tf_buffer.lookup_transform(
            target_frame=self.map_frame,
            source_frame=scan.header.frame_id,
            time=scan.header.stamp,
            timeout=rospy.Duration(1)
        )

        pos = trans.transform.translation
        orient = trans.transform.rotation

        # transform from scan to map
        transform = numpify(trans.transform)

        target_vec = self.TARGET_DIST * np.array([
            np.cos(rel_angle),
            np.sin(rel_angle),
            0,
            1
        ]).dot(transform.T)
        target_angle = model.pose_from_ros(trans).theta + rel_angle

        scan_target_pose = PoseStamped()
        scan_target_pose.header = scan.header
        scan_target_pose.header.frame_id = self.map_frame
        scan_target_pose.pose = Pose(
            Point(*target_vec[0:3]),
            Quaternion(0,0,np.sin(.5*target_angle),np.cos(.5*target_angle))
        )
        self.scan_target_pose = scan_target_pose
        rospy.loginfo("updated target pose")

    @Subscriber('~marker_poses', PointStamped, queue_size=1)
    def sub_marker(self, marker):
        marker_loc = numpify(marker.point, hom=True)
        if np.isnan(marker_loc).any():
            self.marker_target_pose = None
            return
        trans = self.tf_buffer.lookup_transform(
            target_frame="base_link",
            source_frame=self.map_frame,
            time=marker.header.stamp,
            timeout=rospy.Duration(1)
        )
        transform = numpify(trans.transform)

        marker_loc = marker_loc.dot(transform.T)
        marker_target_pose = PoseStamped()
        marker_target_pose.header = marker.header
        marker_target_pose.header.frame_id = 'map'
        marker_target_pose.pose = Pose(
            msgify(Point, marker_loc),
            Quaternion(0,0,0,1)
        )
        self.marker_target_pose = marker_target_pose

    @Timer(rospy.Duration(.25))
    def timer_callback(self, timer):
        rospy.loginfo("Top Planner pub loop")
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame=self.map_frame,
                source_frame="base_link",
                time=timer.current_real,
                timeout=rospy.Duration(1)
            )
        except Exception:
            rospy.loginfo("Top Planner pub loop: tf from base link to map not found")
            return

        at = model.pose_from_ros(trans)
        if np.isnan(at.x) or np.isnan(at.y) or np.isnan(at.theta):
            return

        if self.path is None:
            return

        while self.path_index < len(self.path):
            map_target_pose = self.path[self.path_index]
            t = model.pose_from_ros(map_target_pose)
            ## If already achieved a point or if a point is not ahead and within turning radius, look at future points in the path
            if self.is_achieved(at, t) or not self.is_feasible(at, t):
                rospy.loginfo("passed index {}".format(self.path_index))
                self.path_index += 1
                self.pub_path_point = True
            else:
                break

        if self.marker_target_pose is not None and (self.is_achieved(at,
            model.pose(
                self.marker_target_pose.pose.position.x,
                self.marker_target_pose.pose.position.y,
                at.theta
            )) or not self.is_feasible(at,
            model.pose(
                self.marker_target_pose.pose.position.x,
                self.marker_target_pose.pose.position.y,
                at.theta
            ))):
           self.marker_target_pose = None 

        rospy.loginfo("path index {} publish? {}".format(self.path_index, self.pub_path_point))

        if self.marker_target_pose is not None:
            self.target_pose_pub.publish(self.marker_target_pose)
            self.pub_path_point = True

        elif self.path is not None and self.path_index < len(self.path):
            if self.pub_path_point:
                rospy.loginfo("publishing path target pose")
                self.target_pose_pub.publish(self.path[self.path_index])
                self.pub_path_point = False
        
        elif self.scan_target_pose is not None:
            self.target_pose_pub.publish(self.scan_target_pose)

    def is_feasible(self, rx, ry, rt, tx, ty, tt):
        dist = np.hypot(rx-tx, ry-ty)
        angle = (np.arctan2(ty-ry, tx-rx)-rt)%(2*np.pi)

    def is_feasible(self, r, t):
        dist = np.hypot(r.x-t.x, r.y-t.y)
        angle = (np.arctan2(t.y-r.y, t.x-r.x)-r.theta)%(2*np.pi)
        if angle > np.pi:
            angle -= 2*np.pi

        ## TODO: better feasibility metric?
        if angle > np.pi/2 or angle < -np.pi/2:
            return False
        return dist/2/np.cos(np.pi/2-np.abs(angle)) > self.TURN_RADIUS

    def is_achieved(self, r, t):
        nr = np.array([r.x,r.y]) + self.TARGET_OFFSET*np.array([np.cos(r.theta),np.sin(r.theta)])
        nt = np.array([t.x,t.y]) + self.TARGET_OFFSET*np.array([np.cos(t.theta),np.sin(t.theta)])
        dist = np.hypot(nr[0]-nt[0], nr[1]-nt[1])

        rospy.loginfo("rxy {} {} txy {} {} dist {}".format(nr[0], nr[1], nt[0], nt[1], dist))
        return dist < self.CLOSE_THRESH

if __name__ == '__main__':
    rospy.init_node("planner")
    node = PlanNode()
    rospy.spin()
