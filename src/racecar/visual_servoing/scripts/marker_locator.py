#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Vector3, PointStamped
from visual_servoing.msg import CameraObject, CameraObjectsStamped
from std_msgs.msg import Header
import numpy as np
from ros_numpy import numpify, msgify
import collections
import scipy.ndimage.measurements
import message_filters
import tf2_ros
from rospyext import *
from numpy.linalg import inv


BoundingBox = collections.namedtuple("BoundingBox", ["minx", "miny", "maxx", "maxy"])


class MarkerLocator(Node):
    pub_marker = Publisher('~location', PointStamped, queue_size=1)
    pub_debug_overlay = Publisher('~debug/marker_overlay', Image, queue_size=1)

    # How old the consumed message is.
    pub_packet_entered_node = Publisher('~debug/packet_entered_node', Header, queue_size=1)
    pub_packet_left_node = Publisher('~debug/packet_left_node', Header, queue_size=1)

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        super(MarkerLocator, self).__init__()

    # subscribe to synced messages
    @Subscriber(lambda:
        message_filters.ApproximateTimeSynchronizer([
            message_filters.Subscriber('~mask', Image, queue_size=1),
            message_filters.Subscriber('~mask_info', CameraInfo, queue_size=10)
        ], queue_size=1,slop=1.0)
    )
    def sub_image(self, im_msg, image_info):
        """
        Take a thresholded image and output marker position metrics.
        """
        print("sub_image in marker_locator")
        # Announce that we have received a packet
        self.pub_packet_entered_node.publish(im_msg.header)

        # Extract the image
        im = numpify(im_msg)
        height, width = im.shape[:2]

        # Label each connected blob in the image.
        im2 = (im > 128).astype(bool)
        labels, num_labels = scipy.ndimage.measurements.label(im2)

        # no blobs?
        if num_labels == 0:
            self.pub_marker.publish(PointStamped(
                header=Header(frame_id="base_link", stamp=im_msg.header.stamp),
                point=Point(x=np.nan, y=np.nan, z=0)
            ))
            return

        # Calculate the size of each blob.
        blob_sizes = scipy.ndimage.measurements.sum(im2, labels=labels, index=xrange(0,num_labels+1))

        # Find the biggest blob.
        biggest_blob = np.argmax(blob_sizes) - 1
        pp = scipy.ndimage.measurements.find_objects(labels)[biggest_blob]
        bbox = BoundingBox(pp[1].start, pp[0].start, pp[1].stop, pp[0].stop)

        # Calculate centroid (units of pixels).
        cx, cy = np.mean([bbox.maxx, bbox.minx]), np.mean([bbox.maxy, bbox.miny])

        p_matrix = np.array(image_info.P).reshape((3, 4))
        marker_position = self.get_position(cx, cy, p_matrix, im_msg.header)
        if marker_position is None:
            return

        self.pub_marker.publish(PointStamped(
            header=Header(frame_id="base_link", stamp=im_msg.header.stamp),
            point=msgify(Point, marker_position)
        ))

        # Make overlay visualizations.
        debug_overlay = np.zeros((height, width, 3), dtype=np.uint8)
        debug_overlay[...,0] = im
        debug_overlay[...,1] = im
        debug_overlay[...,2] = im
        self.draw_bbox(debug_overlay, bbox)
        self.draw_point(debug_overlay, cx, cy)
        debug_overlay_msg = msgify(Image, debug_overlay, encoding='rgb8')
        debug_overlay_msg.header = im_msg.header
        self.pub_debug_overlay.publish(debug_overlay_msg)

        # Announce that a packet has left the node.
        self.pub_packet_left_node.publish(im_msg.header)

    def get_position(self, cx, cy, p_matrix, header):
        """
        given the image location, camera matrix, and frame/stamp, get the 3D position

        Assumes the point lies on the ground plane
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame=header.frame_id.lstrip("/"),
                source_frame="base_link",
                time=header.stamp
            ).transform
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error %s", e)
            return

        tf = numpify(tf)

        p_prime = p_matrix.dot(tf)
        p_prime = np.hstack((p_prime[:,0:2],p_prime[:,3:4]))

        pixel_mat = np.array([cx, cy, 1])

        marker_position = inv(p_prime).dot(pixel_mat)
        marker_position /= marker_position[2]
        if marker_position[0]<=0:
            return np.array([np.nan,np.nan,np.nan])
        marker_position[2] = 0  # z = 0

        return marker_position

    def draw_bbox(self, im, bbox):
        linerad = 1
        green = np.array([0, 127, 0])
        green2 = np.array([127, 255, 127])
        im[bbox.miny-linerad:bbox.miny+linerad,:] = green
        im[bbox.maxy-linerad:bbox.maxy+linerad,:] = green2
        im[:,bbox.minx-linerad:bbox.minx+linerad] = green
        im[:,bbox.maxx-linerad:bbox.maxx+linerad] = green2

    def draw_point(self, im, x, y):
        rad = 3
        green = np.array([0, 255, 0])
        im[y-rad:y+rad, x-rad:x+rad] = green


if __name__ == '__main__':
    rospy.init_node("marker_locator")
    node = MarkerLocator()
    rospy.spin()
