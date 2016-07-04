#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Point, Quaternion, Pose2D, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header

import numpy as np
import tf2_ros
import tf2_kdl
from tf import transformations

from rospyext import *
import ros_numpy

import monte_carlo_localization as mcl
from threading import Lock
import PyKDL
from tf_conversions import posemath


class LocalizerNode(Node):
    publish_cloud = Param(bool, default=False)
    num_parts = Param(int, default=100)

    pose_array = Publisher('~pose_array', PoseArray)

    def __init__(self, init_pose=Pose2D(0, 0, 0)):
        # convert pose to our custom np type
        init_pose_np = np.recarray((), dtype=mcl.particle_filter.pose_dtype)
        init_pose_np.x = init_pose.x
        init_pose_np.y = init_pose.y
        init_pose_np.theta = init_pose.theta
        self.map = None
        self.base_frame = None
        self.odom_frame = None
        self.first_odom = None
        self.num_odom = 0

        self.particle_lock = Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        with self.particle_lock:
            # this might cause some handlers to fire, so make sure we already have the lock
            super(LocalizerNode, self).__init__()

            # initialize all the particles
            self.particles = np.recarray(self.num_parts, init_pose.dtype)
            self.particles[:] = init_pose

    @Subscriber('~map', OccupancyGrid)
    def sub_map(self, map):
        rospy.loginfo('Map recieved')
        self.map = mcl.Map(map)

    @Subscriber('~odom', Odometry, queue_size=1)
    def sub_odom(self, odom):
        # base link frame
        self.base_frame = odom.child_frame_id
        # odom frame
        self.odom_frame = odom.header.frame_id
        self.odom = odom
        if self.first_odom is None:
            self.first_odom = odom.header.stamp.to_sec()
            rospy.loginfo('Received first odometry message')
            return

        self.num_odom += 1

        # estimate the time change
        dt = (odom.header.stamp.to_sec()-self.first_odom)/self.num_odom
        with self.particle_lock:
            self.particles = mcl.ros_motion_update.odom_motion_update(odom, self.particles, dt, 1)
        self._publish_tf(odom.header.stamp)

    @Subscriber('~scan', numpy_msg(LaserScan))
    def sub_scan(self, scan):
        if self.map is None or self.base_frame is None:
            rospy.loginfo('Not processing data until map frame found')
            return

        # get the transformation matrix from base_link to laser
        trans = self.tf_buffer.lookup_transform(self.base_frame, scan.header.frame_id, scan.header.stamp)
        trans = ros_numpy.numpify(trans.transform)

        rospy.loginfo('Doing update')
        weights = mcl.ros_sensor_update.sensor_update(self.map, scan, self.particles, trans, 10)
        rospy.loginfo('Resampling')

        with self.particle_lock:
            # we don't actually need the lock when calculating weights, unless our implementation allows the number of particles to increase
            self.particles = mcl.particle_filter.resample(
                self.particles,
                weights,
                self.num_parts
            )

        self._publish_tf(scan.header.stamp)

    def _publish_tf(self, stamp):
        # check that we know which frames we need to publish from
        if self.map is None or self.base_frame is None:
            rospy.loginfo('Not publishing until map and odometry frames found')
            return

        # calculate the mean position
        x = np.mean([p.x for p in self.particles])
        y = np.mean([p.y for p in self.particles])
        theta = np.mean([p.theta for p in self.particles]) #TODO - wraparound

        # map to base_link
        map2base_frame = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(*transformations.quaternion_from_euler(0, 0, theta)),
            PyKDL.Vector(x,y,0)
        )

        odom2base_frame = tf2_kdl.transform_to_kdl(self.tf_buffer.lookup_transform(
            target_frame=self.odom_frame,
            source_frame=self.base_frame,
            time=stamp,
            timeout=rospy.Duration(4.0)
        ))

        # derive frame according to REP105
        map2odom = map2base_frame * odom2base_frame.Inverse() 

        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = self.map.frame
        t.child_frame_id = self.odom_frame
        t.transform.translation = Vector3(*map2odom.p)
        t.transform.rotation = Quaternion(*map2odom.M.GetQuaternion())
        br.sendTransform(t)


        # for Debugging
        if False:
            t.header.stamp = stamp
            t.header.frame_id = self.map.frame
            t.child_frame_id = self.base_frame+"_old"
            t.transform.translation = Vector3(*map2base_frame.p)
            t.transform.rotation = Quaternion(*map2base_frame.M.GetQuaternion())
            br.sendTransform(t)

        if self.publish_cloud:
            msg = PoseArray(
                header=Header(stamp=stamp, frame_id=self.map.frame),
                poses=[
                    Pose(
                        position=Point(p.x, p.y, 0),
                        orientation=Quaternion(*transformations.quaternion_from_euler(0, 0, p.theta))
                    )
                    for p in self.particles
                ]
            )
            self.pose_array.publish(msg)


if __name__ == '__main__':
    rospy.init_node("localizer")
    node = LocalizerNode()
    rospy.spin()
