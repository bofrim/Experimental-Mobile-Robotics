#!/usr/bin/env python
from math import atan
from image_processing import get_red_mask, lowest_object_coord
import rospy, cv2, cv_bridge, numpy
import numpy as np

from comp2.msg import Centroid
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

TOP_CROP_FRAC = 0.0
BOTTOM_CROP_FRAC = 0.0
MASS_THRESHOLD = 100000
BOTTOM_DISTANCE_OFFSET = 0
HEIGHT = 0.5
BOTTOM_DISTANCE = -0.05
TOP_DISTANCE = 1.0

# Image Procesing Constants
H_MAX = 20
H_MIN = 317
S_MAX = 255
S_MIN = 180
V_MAX = 255
V_MIN = 80


def threshold_hsv_360(
    hsv, h_max=H_MAX, h_min=H_MIN, s_max=S_MAX, s_min=S_MIN, v_max=V_MAX, v_min=V_MIN
):
    """Taken from:
    
    https://eclass.srv.ualberta.ca/pluginfile.php/4909552/mod_folder/content/0/Lab5/CMPUT%20412%20Lab%205.pdf?forcedownload=1
    """
    lower_color_range_0 = np.array([0, s_min, v_min], dtype=float)
    upper_color_range_0 = np.array([h_max / 2.0, s_max, v_max], dtype=float)
    lower_color_range_360 = np.array([h_min / 2.0, s_min, v_min], dtype=float)
    upper_color_range_360 = np.array([360 / 2.0, s_max, v_max], dtype=float)
    mask0 = cv2.inRange(hsv, lower_color_range_0, upper_color_range_0)
    mask360 = cv2.inRange(hsv, lower_color_range_360, upper_color_range_360)
    mask = mask0 | mask360
    return mask


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

