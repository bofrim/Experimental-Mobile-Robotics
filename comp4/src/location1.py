#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Led, Sound

from image_processing import (
    get_red_mask,
    count_objects,
    detect_shape,
    get_red_mask_image_det,
)

from utils import display_count, simple_turn

class Detect1(smach.State):
    def __init__(self, rate, sound_pub, light_pubs):
        smach.State.__init__(self, outcomes=["turn_right", "exit"])
        self.rate = rate
        self.sound_pub = sound_pub
        self.light_pubs = light_pubs

    def execute(self, userdata):
        max_count = 0
        for _ in range(7):
            image = rospy.wait_for_message("camera/rgb/image_raw", Image)
            red_mask = get_red_mask_image_det(image)
            h, w = red_mask.shape
            search_top = h * 0.7
            search_bot = h
            red_mask[0:search_top, 0:w] = 0
            red_mask[search_bot:h, 0:w] = 0
            count = count_objects(red_mask)
            max_count = max(count, max_count)

        cv2.imshow("redmask", red_mask)
        cv2.waitKey(1000)

        display_count(max_count, self.light_pubs)

        sound_msg = Sound()
        sound_msg.value = Sound.ON
        for i in range(max_count - 1):
            self.sound_pub.publish(sound_msg)
            rospy.sleep(0.3)

        self.sound_pub.publish(sound_msg)

        return "turn_right"


if __name__ == "__main__":
    print("init")
    rospy.init_node("loc1")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("Wait for image")
        image = rospy.wait_for_message("camera/rgb/image_raw", Image)
        red_mask = get_red_mask_image_det(image)
        canvas = cv_bridge.CvBridge().imgmsg_to_cv2(image, desired_encoding="bgr8")
        shapes, moments = detect_shape(red_mask, canvas)
        count_objects(red_mask)
        big_shape_count = 0
        for m in moments:
            print("moment size:", m["m00"])
            if m["m00"] > 2300:
                big_shape_count += 1
        print(big_shape_count)
        cv2.imshow("window", canvas)
        cv2.waitKey(3)
        print("count", big_shape_count)
        rospy.sleep(0.1)
