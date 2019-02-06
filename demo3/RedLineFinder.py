#!/usr/bin/env python
from math import atan
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np

TOP_CROP_FRAC = 0.0
BOTTOM_CROP_FRAC = 0.0
MASS_THRESHOLD = 1
BOTTOM_DISTANCE_OFFSET = 0
HEIGHT = 0.5
BOTTOM_ANGLE = 10
TOP_ANGLE = 70

# Image Procesing Constants
H_MAX = 48
H_MIN = 322
S_MAX = 255
S_MIN = 97
V_MAX = 250
V_MIN = 175


def interpolate_map(value, orig_min, orig_max, map_min, map_max):
    print()
    orig_range = orig_max - orig_min
    map_range = map_max - map_min
    range_frac = float(value - orig_min) / float(orig_range)
    return map_min + (range_frac * map_range)


def point_to_distance(y_value, y_min, y_max):
    """Use trig to try to calculate the distance to a point."""
    angle = interpolate_map(y_value, y_min, y_max, BOTTOM_ANGLE, TOP_ANGLE)
    print("angle:", angle)
    return atan(angle) * HEIGHT


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
            "camera/rgb/image_raw", Image, self.image_callback
        )
        self.red_line_pub = rospy.Publisher(
            "red_line_distance", Float32, queue_size="1"
        )

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = threshold_hsv_360(hsv=hsv)

        height, width, _ = image.shape
        search_top = height * TOP_CROP_FRAC
        search_bot = height * 1 - BOTTOM_CROP_FRAC
        mask[0:search_top, 0:width] = 0
        mask[search_bot:height, 0:width] = 0

        moment = cv2.moments(mask)
        if moment["m00"] > MASS_THRESHOLD:
            cx = int(moment["m10"] / moment["m00"])
            cy = int(moment["m01"] / moment["m00"])

            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            cv2.circle(mask, (cx, cy), 20, (0, 0, 255), -1)
            print("cy", cy)

            self.red_line_pub.publish(Float32(point_to_distance(float(cy), 0, height)))
        else:
            self.red_line_pub.publish(Float32(1000))

        cv2.imshow("window", mask)
        cv2.waitKey(3)


rospy.init_node("red_line_finder")
follower = RedLineFinder()
rospy.spin()
