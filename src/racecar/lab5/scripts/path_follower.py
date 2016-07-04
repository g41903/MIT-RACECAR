#!/usr/bin/env python

'''
This script takes in a path, a current position estimate (from the odometry or particle filter) and outputs steering commands.
Note: This script has been updated to use tf transforms for localization. This
allows for use of AMCL.
Winter Guerra
'''

import rospy
from geometry_msgs.msg import PoseStamped, Point, Pose, PointStamped
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
import numpy as np
import tf.transformations as transformations
import tf2_ros
from lab6 import model

from rospyext import *
import ros_numpy

def angle_of(vector):
    """ counterclockwise angle from the x axis """
    return np.arctan2(*vector[::-1])

def norm_angle(angle):
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi
    return angle

def pose_forward(p, amt):
    """ Return the point, a ndarray of len 2, `amt` forward from `p` """
    return p.xy + amt*np.array([np.cos(p.theta), np.sin(p.theta)])

class PathFollowerNode(Node):
    # Navigation settings
    max_speed = Param(float, default=0.5)
    min_speed = 0 # Will be overwritten later
    p_speed = Param(float, default=2.0) # Higher values mean we slow down more around corners

    max_angle = 0.5
    min_angle = -0.5

    CLOSE_THRESH = 0.25 # how close is close enough to a waypoint
    TARGET_OFFSET = 0.5 # the location on the car that we try to match with reality

    # Input channels
    base_frame = Param(str, default='base_link')

    # Output channels
    # Steering commands
    pub_plant = Publisher('/racecar/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
    # Are we done executing our path?
    pub_progress = Publisher('/lab6/path_follower/execution/complete', Header, queue_size=1)
    # Did we encounter an E-Stop during our path execution?
    pub_unexpected_halt = Publisher('/lab6/path_follower/execution/failed', Header, queue_size=1)

    # for debugging
    pub_target_pose = Publisher('~target', PoseStamped)
    pub_target_front = Publisher('~front/target', PointStamped)
    pub_robot_front = Publisher('~front/robot', PointStamped)

    def __init__(self):
        self.path_frame = 'map'
        self.robot_pose = None
        self.current_path = np.recarray(0, dtype=model.pose_dtype)

        self.next_waypoint_index = 0

        # Init the TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        super(PathFollowerNode, self).__init__()
        # Init vars

        self.min_speed = -self.max_speed

    def _closeness(self, pose1, pose2):
        """
        Measure how close together two poses are
        In this case, we choose to do this by comparing how far apart the front
        of the car would be in the two poses
        """
        front_curr = pose_forward(pose1, self.TARGET_OFFSET)
        front_dest = pose_forward(pose2, self.TARGET_OFFSET)
        return np.linalg.norm(front_dest - front_curr)


    def update_waypoint(self):
        '''
        Update self.next_waypoint_index, by finding the next waypoint that
        is sufficiently far away
        '''
        # Check if we have a valid path
        if not len(self.current_path):
            return

        # Otherwise, find the next waypoint we haven't achieved
        at = self.next_waypoint_index
        i_path = iter(enumerate(self.current_path))

        try:
            i, node = next(i_path)
            # skip all the waypoints we've already done
            while i < self.next_waypoint_index:
                i, node = next(i_path)

            try:
                # skip all waypoints that are too close
                while self._closeness(node, self.robot_pose) > self.CLOSE_THRESH:
                    i, node = next(i_path)
            except StopIteration:
                # none of the nodes are close, so keep the old waypoint
                return

            # until we find the next one which is not too close
            while self._closeness(node, self.robot_pose) <= self.CLOSE_THRESH:
                i, node = next(i_path)

            self.next_waypoint_index = i
        except StopIteration:
            # if we ran out of waypoints, return the last one
            self.next_waypoint_index = len(self.current_path)

    def publish_debug_target(self):
        ''' log the debug location '''
        msg = PoseStamped(
            header=Header(
                frame_id=self.path_frame,
                stamp=rospy.Time.now()
            ),
            pose=model.pose_to_ros(Pose, self.waypoint_pose)
        )
        self.pub_target_pose.publish(msg)

    @property
    def waypoint_pose(self):
        '''
        Returns the next waypoint. If there is no waypoint to go to, it returns the current pose of the robot.
        '''
        return self.current_path[self.next_waypoint_index] if self.next_waypoint_index < len(self.current_path) else self.robot_pose

    def get_steering_params(self):
        '''
        Return the current steering the speed parameters for the robot to
        reach its next waypoint -- regardless of whether it needs to reverse or not.
        '''
        # find where the front of the car is and should be
        front_curr = pose_forward(self.robot_pose, self.TARGET_OFFSET)
        front_dest = pose_forward(self.waypoint_pose, self.TARGET_OFFSET)

        self.pub_robot_front.publish(PointStamped(
            header=Header(frame_id=self.path_frame),
            point=Point(front_curr[0], front_curr[1], 0)
        ))
        self.pub_target_front.publish(PointStamped(
            header=Header(frame_id=self.path_frame),
            point=Point(front_dest[0], front_dest[1], 0)
        ))

        diff = front_dest - front_curr
        distance_error = np.linalg.norm(diff)
        bearing = angle_of(diff)
        angle_error = norm_angle(bearing - self.robot_pose.theta)

        # Update steering_angle
        steering_angle = self.clamp_angle(angle_error)

        # Update speed
        speed = self.clamp_speed( self.max_speed / (1.0 + self.p_speed*abs(steering_angle) ))

        return steering_angle, speed

    def update_navigation(self):
        '''
        This is triggered by updates in robot position and updates to the path.
        '''
        # Publish a new navigation update
        cmd = AckermannDriveStamped()

        self.update_waypoint()
        # self.publish_debug_target()

        # Check that we actually have a path to follow.
        # If not, stop the car.
        if not len(self.current_path):
            # no path
            pass
        elif self.next_waypoint_index < len(self.current_path):
            steering_angle, speed = self.get_steering_params()

            # Build our commands
            cmd.drive.speed = speed
            cmd.drive.steering_angle = steering_angle

            rospy.loginfo('Following waypt #{} at speed {}, angle {}'.format(self.next_waypoint_index, speed, steering_angle))

        # If we have reached the end of our path, report this.
        else:
            # Clear our path
            self.current_path = np.recarray(0,dtype=model.pose_dtype)
            self.next_waypoint_index = 0

            # Report that our execution is done!
            completion_signal = Header()
            self.pub_progress.publish(completion_signal)

            rospy.loginfo('Done')

        self.pub_plant.publish(cmd)

    def clamp_angle(self, angle):
        ''' Clamp steering to sane values. '''
        return np.clip(angle, self.min_angle, self.max_angle)

    def clamp_speed(self, speed):
        ''' Limit robot speed'''
        return np.clip(speed, self.min_speed, self.max_speed)

    # Update our controls based on published tf data at 30hz
    @Timer(rospy.Duration(0.1))
    def sub_robot_pose_update(self, event):
        '''
        Every time the particle filter updates our current position, we should first check if we have reached a waypoint in our path. Then, we should tell the PID controller that our position and targets have changed.
        '''

        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=self.path_frame,
                source_frame=self.base_frame,
                time=event.current_real,
                timeout=rospy.Duration(0.1)
            ).transform
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn("Error getting robot position", exc_info=True)
            return

        self.robot_pose = model.pose_from_ros(tf)

        # we have no path yet to give us a frame to refer to
        if self.path_frame is None:
            rospy.loginfo('No path yet')
        else:
            # Update navigation
            self.update_navigation()


    @Subscriber('~path', Path, queue_size=1)
    def sub_robot_path_update(self, msg):
        '''
        Every time our higher level logic gives us a new path to follow, we should replace our current planned path with the new path.

        We should also assume that our next waypoint is now the first point on the new path.
        '''
        rospy.loginfo('Recieved a path')

        # Assert that we have received a TF before trying to take on a path.
        if self.robot_pose is None:
            rospy.loginfo('Ignoring path since path_follower has not yet received a TF.')
            return


        self.path_frame = msg.header.frame_id
        self.current_path = np.recarray(len(msg.poses), dtype=model.pose_dtype)
        for i in range(len(msg.poses)):
            p = msg.poses[i]
            self.current_path[i] = model.pose_from_ros(p)
        self.next_waypoint_index = 0
        # Update navigation
        self.update_navigation()


if __name__ == '__main__':
    rospy.init_node("path_follower")
    node = PathFollowerNode()
    rospy.spin()
