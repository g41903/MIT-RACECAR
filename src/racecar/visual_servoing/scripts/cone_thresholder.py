#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Duration, Header
import numpy as np
from ros_numpy import numpify, msgify
import scipy.misc
import scipy.ndimage.morphology
import time
from copy import deepcopy

from rospyext import *

import os

thetas = np.array([
    [0, -0.8, 0.6, -0.12],
    [0, -0.8, 0.4, -0.08]
])


class ConeThresholder(Node):
    # lookup or linear
    mode = Param(str, default='lookup')
    object_data=Param(str)
    print object_data
    # downsampling factor (disabled by default)
    downsample = Param(float, default=1.0)

    # Whether to perform morphological opening to eliminate noise.
    # Morphological opening is an erosion followed by a dilation.
    # It basically means to shrink the white portion so that small
    # clusters disappear, and then grow it again to fill in gaps.
    # Something's up with params. This is disabled.
    # morph_open = Param(int, default=2)
    morph_open = False
    morph_size = Param(int, default=3)
    morph_iter = Param(int, default=1)

    # number of bits per channel to use for lookup. Lower is faster
    nbits = Param(int, default=3)

    pub_mask = Publisher('~mask', Image, queue_size=1)

    # How old the consumed message is.
    pub_packet_entered_node = Publisher('~debug/packet_entered_node', Header, queue_size=1)
    pub_packet_left_node = Publisher('~debug/packet_left_node', Header, queue_size=1)

    def __init__(self):
        super(ConeThresholder, self).__init__()

        self.lookup = None

        if self.mode == 'lookup':
            self._init_lookup()

    def _init_lookup(self):
        """ prepare the lookup data """
        if self.lookup is not None:
            return
        
        # load pixels' npy files
        calib = np.load(self.object_data)
        lookup = np.zeros(2**(3*self.nbits), dtype=np.float32)
        indices = self.lookup_idx(calib)

        # lookup[indices] += 1 doesn't work for repeated indices
        np.add.at(lookup, indices, 1)

        # normalize
        lookup /= np.max(lookup)

        # this makes the pixels brighter - better for debugging
        lookup = lookup ** 0.5

        # push back into a uint8
        self.lookup = np.clip((255 * lookup).astype(np.uint8), 0, 255)

    def lookup_idx(self, pixel):
        """ convert [r, g, b] to a single integer to be used as an index """
        pixel = (pixel >> (8 - self.nbits)).astype(np.uint32)
        return pixel[...,0] | pixel[...,1] << self.nbits | pixel[...,2] << (self.nbits*2)

    @Subscriber('~image', Image, queue_size=1)
    def sub_image(self, im_msg):
        """
        Take in an image, and publish a black and white image indicatng which pixels are cone-like
        """
        old_message_header = deepcopy(im_msg.header)

        # Announce that we have received a packet
        self.pub_packet_entered_node.publish(old_message_header)

        # extract the image
        im = numpify(im_msg)

        # Downsample.
        if self.downsample < 1:
            im = scipy.misc.imresize(im, self.downsample)

        if self.mode == 'linear':
            is_cone = np.ones(im.shape[:-1], dtype=np.bool)
            im = im.astype(np.float32)
            # red channel
            for theta in thetas:
                is_cone = is_cone & (im.dot(theta[:-1]) + theta[-1] > 0)
            im_mask = np.uint8(is_cone * 255)

        elif self.mode == 'lookup':
            self._init_lookup()
            im_mask = self.lookup[self.lookup_idx(im)]

        else:
            raise ValueError("Unrecognized thresholding mode.")

        if self.morph_open:
            open_size = self.morph_size
            open_structure = np.ones((open_size, open_size))
            im_mask = scipy.ndimage.morphology.binary_opening(
                im_mask, iterations=self.morph_iter, structure=open_structure)
            im_mask = (im_mask * 255).astype(np.uint8)

        # build the message
        im_mask_msg = msgify(Image, im_mask, encoding='mono8')
        im_mask_msg.header = old_message_header
        self.pub_mask.publish(im_mask_msg)
        
        # Announce that a packet has left the node.
        self.pub_packet_left_node.publish(old_message_header)


if __name__ == '__main__':
    rospy.init_node("cone_thresholder")
    node = ConeThresholder()
    rospy.spin()
