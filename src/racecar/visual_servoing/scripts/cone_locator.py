#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Vector3
from visual_servoing.msg import CameraObject, CameraObjectsStamped
from std_msgs.msg import UInt32, Duration, Header
import numpy as np
from ros_numpy import numpify, msgify
import collections
import scipy.ndimage.measurements
import message_filters

from rospyext import *

BoundingBox = collections.namedtuple("BoundingBox", ["minx", "miny", "maxx", "maxy"])

def remap(value, fromMin, fromMax, toMin, toMax, clamp=False):
    """Remap a value from one range to another. Floats!"""
    z = (value - fromMin) * (toMax - toMin) / float((fromMax - fromMin)) + toMin
    if clamp:
        # Clamp z to the output range.
        z = max(toMin, min(toMax, z))
    return z


class ConeLocator(Node):
    HEIGHT = 0.17
    WIDTH = 0.15
    pub_obj = Publisher('~cone', CameraObjectsStamped, queue_size=1)

    pub_debug_overlay  = Publisher('~debug/cone_overlay', Image, queue_size=1)

    # How old the consumed message is.
    pub_packet_entered_node = Publisher('~debug/packet_entered_node', Header, queue_size=1)
    pub_packet_left_node = Publisher('~debug/packet_left_node', Header, queue_size=1)

    # subscribe to synced messages
    @Subscriber(lambda:
        message_filters.TimeSynchronizer([
            message_filters.Subscriber('~cone_mask', Image, queue_size=1),
            message_filters.Subscriber('~cone_mask_info', CameraInfo, queue_size=10)
        ], queue_size=10)
    )
    def sub_image(self, im_msg, image_info):
        """
        Take a thresholded image and output cone position metrics.
        """
        # Announce that we have received a packet
        self.pub_packet_entered_node.publish(im_msg.header)


        # Extract the image
        im = numpify(im_msg)
        im2 = (im > 160).astype(bool)

        height, width = im.shape[:2]

        # Label each connected blob in the image.
        labels, num_labels = scipy.ndimage.measurements.label(im2)

        msg = CameraObjectsStamped(header=im_msg.header)

        if num_labels == 0:
            self.pub_obj.publish(msg)
            # No objects.
            return

        # Calculate the size of each blob.
        blob_sizes = scipy.ndimage.measurements.sum(im2, labels=labels, index=xrange(0,num_labels+1))

        # Find the biggest blob.
        biggest_blob = np.argmax(blob_sizes) - 1
        pp = scipy.ndimage.measurements.find_objects(labels)[biggest_blob]
        bbox = BoundingBox(pp[1].start, pp[0].start, pp[1].stop, pp[0].stop)

        # Calculate centroid (units of pixels).
        cx, cy = np.mean([bbox.maxx, bbox.minx]), np.mean([bbox.maxy, bbox.miny])

        sx, sy = (bbox.maxx - bbox.minx), (bbox.maxy - bbox.miny)
        # geometric mean

        p_matrix = np.array(image_info.P).reshape((3, 4))

        uvw = p_matrix.dot([self.WIDTH, self.HEIGHT, 0, 0])
        u, v, w = uvw

        cz = np.min([(u / sx), (v / sy)])


        # load the depth image message
        # d_im = numpify(d_im_msg) * 0.001

        # TODO
        # cz = np.mean(d_im[(labels == biggest_blob) & (d_im != 0)]) # get_depth_for(biggest_blob)

        # print u / sx, v / sy, sx, sy, uvw #, np.max(d_im)

        # projection matrix
        # [px, py, 1] = P.dot([X, Y, Z, 1])

        # solve the simultaneous equations
        # [cx, cy, 1] =                        P.dot([X, Y, Z, 1])
        #          cz = Z
        #       =>  0 = np.array([0, 0, 1, -cz]).dot([X, Y, Z, 1])
        try:
            position = np.linalg.solve(
                np.vstack((
                    p_matrix,
                    [0, 0, 1, -cz]
                )),
                np.array([cx, cy, 1, 0])
            )
        except np.linalg.LinAlgError:
            rospy.logwarn("Matrix was singular %s", np.vstack((
                    p_matrix,
                    [0, 0, 1, -cz]
            )))
            return

        # theses are homogeneous coordinates
        position /= position[-1]
        position = Point(*position[:3])

        msg.objects = [
            CameraObject(
                label='cone',
                center=position,
                size=Vector3(
                    x=remap((bbox.maxx - bbox.minx), 0, width, 0, 1),
                    y=remap((bbox.maxy - bbox.miny), 0, height, 0, 1),
                    z=0
                )
            )
        ]
        self.pub_obj.publish(msg)

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

    def bounds_by_minmax(self, im):
        """Find the bounding box of a threshold image.
        im: numpy image
        Returns a BoundingBox
        """
        yis, xis = np.nonzero(im > 127)
        return BoundingBox(min(*xis), min(*yis), max(*xis), max(*yis))

    def bounds_by_walk(self, im, cx, cy):
        """Find the bounding box a threshold object centered at cx, cy.
        im: numpy image
        Returns a BoundingBox
        """
        raise NotImplementedError()

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
    rospy.init_node("cone_locator")
    node = ConeLocator()
    rospy.spin()
