#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Led

from image_processing import get_red_mask, count_objects, detect_shape


class TurnLeft1(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["detect1", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(75, self.vel_pub)

        # Remove this
        for _ in range(25):
            self.vel_pub.publish(Twist())
            self.rate.sleep()

        return "detect1"


class Detect1(smach.State):
    def __init__(self, rate, light_pubs):
        smach.State.__init__(self, outcomes=["turn_right", "exit"])
        self.rate = rate
        self.light_pubs = light_pubs

    def execute(self, userdata):
        # TODO Add Detection
        max_count = 0
        for _ in range(7):
            image = rospy.wait_for_message("camera/rgb/image_raw", Image)
            red_mask = get_red_mask(image)
            # canvas = cv_bridge.CvBridge().imgmsg_to_cv2(image, desired_encoding="bgr8")
            # shapes, moments = detect_shape(red_mask, canvas)
            count = count_objects(red_mask)
            max_count = max(count, max_count)

        # TODO: Show detection in lights
        led_on_msg = Led()
        led_on_msg.value = Led.ORANGE
        led_off_msg = Led()
        led_off_msg.value = Led.BLACK
        if max_count & 0x01:
            self.light_pubs[1].publish(led_on_msg)
        else:
            self.light_pubs[1].publish(led_off_msg)

        if max_count & 0x02:
            self.light_pubs[0].publish(led_on_msg)
        else:
            self.light_pubs[0].publish(led_off_msg)

        return "turn_right"


if __name__ == "__main__":
    print("init")
    rospy.init_node("loc1")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("Wait for image")
        image = rospy.wait_for_message("camera/rgb/image_raw", Image)
        red_mask = get_red_mask(image)
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
        rate.sleep()
