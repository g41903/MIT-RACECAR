import os

import rosbag
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
import ros_numpy

from images2gif import writeGif
import PIL.Image
import numpy as np

fname = os.path.expanduser("~/team-ws/bags/2016-02-26-20-16-28-coneshadow.bag")

frames = []
durations = []
last_t = None

with rosbag.Bag(fname) as bag:
    for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/camera/zed/rgb/image_rect_color'])):
        if i < 5: continue
        print i

        if not last_t:
            dt = t - t
        else:
            dt = t - last_t

        msg.__class__ = Image
        frame = ros_numpy.numpify(msg)[...,::-1]
        frame = PIL.Image.fromarray(frame)
        frame.thumbnail((320, 180), PIL.Image.ANTIALIAS)
        frame = np.array(frame)

        frames.append(frame)
        durations.append(dt.to_sec())

        last_t = t

writeGif('sample-data-shadow.gif', frames, durations)
