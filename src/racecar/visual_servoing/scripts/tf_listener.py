
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
import numpy as np
import tf.transformations as transformations
import tf2_ros

from rospyext import *
import ros_numpy

class tf_listener(Node):
    def __init__(self):
        self.map = None

        super(RRTNode, self).__init__()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        tf = self.tf_buffer.lookup_transform(
                target_frame=self.base_link,
                source_frame=self.zed_optical_frame,
                time=event.current_real,
                timeout=rospy.Duration(0.1)
            ).transform
        print(tf)
if __name__ == '__main__':
    rospy.init_node("tf_listener")
    node = tf_listener()
    rospy.spin()