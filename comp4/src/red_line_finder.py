#!/usr/bin/env python
from math import atan
from image_processing import get_red_mask, lowest_object_coord
import rospy, cv2, cv_bridge, numpy
import numpy as np

from comp2.msg import Centroid
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from config_globals import *


class RedLineFinder:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "usb_cam/image_raw", Image, self.image_callback
        )
        self.red_line_pub = rospy.Publisher(
            "red_line_distance", Centroid, queue_size="1"
        )

    def image_callback(self, msg):
        mask = get_red_mask(msg)
        cx, cy = lowest_object_coord(mask)
        height, width = mask.shape

        centroid_msg = Centroid()
        centroid_msg.cx = cx
        centroid_msg.cy = cy
        centroid_msg.err = cx - width / 2

        self.red_line_pub.publish(centroid_msg)


if __name__ == "__main__":
    rospy.init_node("red_line_finder")
    follower = RedLineFinder()
    rospy.spin()

