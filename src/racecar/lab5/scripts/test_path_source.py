#!/usr/bin/env python

'''
This script outputs a constant test path for the robot to follow. This simulates a working path planner.
Winter Guerra
'''

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
import numpy as np

from rospyext import *

def make_pose(point3, orientation=Quaternion(0, 0, 0, 1)):
    return PoseStamped(
        header=Header(frame_id="map"),
        pose=Pose(
            position=Point(*point3),
            orientation=orientation
        )
    )

def make_path(*points, **kwargs):
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

def as_path(*points):
    path = [make_pose([p[0],p[1],0], Quaternion(0, 0, np.sin(p[2]/2), np.cos(p[2]/2))) for p in points]
    return path

class TestPathSource(Node):
    # in global frame
    paths = {
        # Move in a square loop. 18ft by 10ft to make it around a lab column.
        # This loop is clockwise starting on the 18ft run.
        "column": make_path((0,0), (5.486,0), (5.486,3.048), (0,3.048), (0,0)),
        "tunnel_v4": as_path(
            (2.5, .5, 0),
            (4, 7.5, np.pi/2),
            (8, 11.45, np.pi/4),
            (17.05, 23.15, np.pi/2),
            (16.35, 26, 3*np.pi/4),
            (14, 27, np.pi),
            (3.75, 27, np.pi),
            (3.05, 33.3, np.pi/2),
            (1.15, 35.2, np.pi),
            (-28.3, 35.2, np.pi),
            (-30.5, 33, -np.pi/2),
            (-31, 2.35, -np.pi/2),
            (-28.3, 0.05, 0),
            (-4.55, -.5, 0)),
        "tunnels": make_path(
            ( 0,    0),
            (0.025, 0),
            (0.05,0),
            (0.075,0),
            (0.1, 0),
            (0.125, 0),
            (0.015, 0),
            (0.0175, 0),
            (0.2, 0),
            (0.4, 0),
            (0.5, 0),
            (0.6, 0),
            (0.7, 0),
            (0.8, 0),
            (1, 0),
            (1.2, 0.05),
            (1.4, 0.1),
            ( 1.5,  .15),
            (1.6, 0.2),
            (1.7, 0.225),
            (1.8, 0.225),
            (1.9, 0.2),
            (2, 0.15),
            (2.1, 0.1),
            (2.2, 0.05),
            (2.3, 0),
            (2.4, -0.05),
            (2.5, -.1),
            (2.6, -.125),
            (2.7, -.125),
            (3, -.125),
            ( 3.69, 7.45),
            (16.5, 24),
            (13.94,26),
            ( 2.28,25.88),
            ( 1.73,34.3),
            (-31.5,34.4),
            (-31.7,-0.61),
            (0,0)
        ),
        "kturn": make_path((0,0), (1.5,0), (3, 7.45), (4, 0), (10,0)),
        "rle": make_path((1.5,0), (4.5,0), (5, -0.5), (7,1), (10,1), (1.5,0)),
    }

    pub_path = Publisher('~robot_path', Path, queue_size=1, latch=True)

    which = Param('~which', str, default='tunnel_v4')

    # Constructor
    def __init__(self):
        super(TestPathSource, self).__init__()
        overall_plan = self.paths[self.which]
        rospy.loginfo("The length of the path is {}".format(len(overall_plan)))

        #half = len(overall_plan) // 2

        # Divide up the path into 2 segments so that the overall path is not a loop.
        self.currentPath = overall_plan
        self.nextPathSegment = overall_plan[-1:]


        # Let's broadcast the first portion of the test path
        self.broadcast(Header())

    # Publishes the next portion of the path (if there is any to publish)
    #@Subscriber('/lab6/path_follower/execution/complete', Header, queue_size=1)
    def broadcast(self, msg):
        if len(self.currentPath):
            # Publish the path
            cmd = Path()
            cmd.poses = self.currentPath
            cmd.header.frame_id = 'map'
            self.pub_path.publish(cmd)

            # Prep the next path portion
            self.currentPath = self.nextPathSegment
            self.nextPathSegment = []

if __name__ == '__main__':
    rospy.init_node("test_path_source")
    node = TestPathSource()
    rospy.spin()
