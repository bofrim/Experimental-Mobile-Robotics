#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from comp2.msg import Centroid
from sensor_msgs.msg import Image

from image_processing import right_most_object_coord, hsv_bound, WHITE_UPPER, WHITE_LOWER


class WhiteLineRampTracker:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "usb_cam/image_raw", Image, self.image_callback
        )
        self.ramp_centroid_pub = rospy.Publisher(
            "white_line_ramp_centroid", Centroid, queue_size="1"
        )

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = hsv_bound(hsv, WHITE_UPPER, WHITE_LOWER, denoise=2, fill=6)

        h, w, d = image.shape
        search_top = h * 0.6
        search_bot = search_top + 200
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        cx, cy = right_most_object_coord(mask)

        cv2.circle(mask, (cx, cy), 20, (0,0,255), -1)
        cv2.imshow("white_ramp", mask)
        cv2.waitKey(3)

        centroid_msg = Centroid()
        centroid_msg.cx = cx
        centroid_msg.cy = cy
        centroid_msg.err = cx - w / 2

        self.ramp_centroid_pub.publish(centroid_msg)


rospy.init_node("white_line_ramp")
follower = WhiteLineRampTracker()
rospy.spin()

