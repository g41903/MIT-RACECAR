#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from tf import transformations
import monte_carlo_localization as mcl
from rospyext import *
from ros_numpy import numpify, msgify, numpy_msg
from copy import deepcopy

import cProfile
import os
import rospkg

class FakeCone(Node):
    map_pub = Publisher('~local_map', numpy_msg(OccupancyGrid), queue_size=1)

    def __init__(self):
        rospy.loginfo("created node")
        self.mcl_map = None
        self.mcl_local_map = None
        self.map_frame = None
        self.map_msg = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        super(FakeCone, self).__init__()

    @Subscriber('~global_map', numpy_msg(OccupancyGrid), queue_size=1)
    def sub_map(self, map):
        """
        We need the global map in order to set up the coordinate frame
        correctly for the local map
        """
        rospy.loginfo("received map")
        self.mcl_map = mcl.Map(map)
        self.map_frame = map.header.frame_id
        self.map_info_glob = map.info

        self.width = 10
        self.height = 10
        ros_map = numpy_msg(OccupancyGrid)()
        ros_map.header.frame_id = 'map'
        ros_map.info = deepcopy(map.info)
        ros_map.info.width = self.width
        ros_map.info.height = self.height
        ros_map.info.origin.position.x = -1*ros_map.info.resolution*self.height/2.0
        ros_map.info.origin.position.y = -1*ros_map.info.resolution*self.width/2.0
        self.map_info = ros_map.info
        ros_map.data = np.ones(self.width*self.height, dtype=np.int8) * -1
        self.ros_map = ros_map

    def _make_local_map(self, point):
        #target frame is map_frame, source frame is laser_link

        self.map_info.origin.position = point.point
        self.ros_map.info = self.map_info
        self.ros_map.data[...] = -1
        mcl_local_map = mcl.Map(self.ros_map)
        mcl_local_map.grid[:,:] = 100

        ### TODO: figure out faster serialization issue
        map_msg = msgify(numpy_msg(OccupancyGrid), mcl_local_map.grid, info=self.map_info)
        map_msg.header = point.header
        map_msg.header.frame_id = 'map'
        return map_msg

    @Subscriber('/clicked_point', PointStamped, queue_size=1)
    def sub_scan(self, point):
        if self.mcl_map is None:
            return

        pr = cProfile.Profile()
        pr.enable()
        self.map_msg = self._make_local_map(point)
        rospy.loginfo("updated map")
        self.map_pub.publish(self.map_msg)
        rospy.loginfo('published map')

        pr.disable()
        pr.dump_stats(os.path.join(
            rospkg.RosPack().get_path('lab6'), 'grid_stats.txt'
        ))
        rospy.loginfo("wrote stats")

if __name__ == '__main__':
    rospy.init_node("fake_cone")
    node = FakeCone()
    rospy.spin()
