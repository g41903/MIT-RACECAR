#!/usr/bin/env python
import os

import rospy
from sensor_msgs.msg import Image
import numpy as np
from ros_numpy import numpify, msgify

from rospyext import *

import scipy.ndimage


class ConeThresholdCalibrator(Node):
    mask_fname = Param(str)
    data_fname = Param(str)

    def __init__(self):
        super(ConeThresholdCalibrator, self).__init__()
        # load template image, and make boolean
        mask = scipy.ndimage.imread(self.mask_fname)
        if len(mask.shape) == 3:
             mask = np.mean(mask.astype(np.float32), axis=-1)
        self.mask = mask >= 128
        self.cone_color_list = []
        self.skip = 5

    @Subscriber('image', Image)
    def sub_image(self, im_msg):
        """
        add pixel color from image 
        """
        # the first few frames are bad
        if self.skip > 0:
            self.skip -= 1
            return

        # extract the image
        im = numpify(im_msg)

        cone_colors = im[self.mask,:]

        self.cone_color_list.append(cone_colors)

    def save_data(self):
        data = np.concatenate(self.cone_color_list, axis=0)
        np.save(self.data_fname, data)
        print "Saved samples from {} frames, containing a total of {} pixels".format(
            len(self.cone_color_list), data.shape[0]
        )


if __name__ == '__main__':
    rospy.init_node("cone_thresholder_calib")
    node = ConeThresholdCalibrator()
    rospy.on_shutdown(node.save_data)
    rospy.spin()
