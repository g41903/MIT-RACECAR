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


class ColorThresholder(Node):
    pixel_file = Param(str, default=None)
    lookup_file = Param(str, default=None)
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
        self.lookup = None
        super(ColorThresholder, self).__init__()

        self._init_lookup()

    def _init_lookup(self):
        """ prepare the lookup data """
        if self.lookup is not None:
            return

        if self.lookup_file is not None:
            lookup = np.load(self.lookup_file)
            assert lookup.ndim == 3
            assert lookup.dtype == np.bool
            rospy.loginfo("Loaded lookup table from lookup_file")

        elif self.pixel_file is not None:
            # load pixels' npy files
            calib = np.load(self.pixel_file)
            lookup = np.zeros((2**(self.nbits),)*3, dtype=np.float32)
            indices = self.lookup_idx(calib)

            # lookup[indices] += 1 doesn't work for repeated indices
            np.add.at(lookup, indices, 1)

            # normalize
            lookup /= np.max(lookup)

            # this makes the pixels brighter - better for debugging
            lookup = lookup ** 0.5
            rospy.loginfo("Generated lookup table from pixel_file")

        else:
            raise ValueError("Either the lookup_file or pixel_file params must be set")

        # push back into a uint8
        self.lookup = np.clip((255 * lookup).astype(np.uint8), 0, 255)

    def lookup_idx(self, pixel):
        """ convert [r, g, b] to an index into self.lookup """
        pixel = (pixel >> (8 - self.nbits))
        return pixel[...,2], pixel[...,1], pixel[...,0]

    @Subscriber('~image', Image, queue_size=1)
    def sub_image(self, im_msg):
        """
        Take in an image, and publish a black and white image indicatng which pixels are cone-like
        """
        if self.lookup is None: return

        old_message_header = deepcopy(im_msg.header)

        # Announce that we have received a packet
        self.pub_packet_entered_node.publish(old_message_header)

        # extract the image
        im = numpify(im_msg)

        # Downsample.
        if self.downsample < 1:
            im = scipy.misc.imresize(im, self.downsample)

        im_mask = self.lookup[self.lookup_idx(im)]

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
    rospy.init_node("color_thresholder")
    node = ColorThresholder()
    rospy.spin()
