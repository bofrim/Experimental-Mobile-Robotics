#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from operations import simple_turn
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from time import time
from image_processing import get_white_mask, crop, path_mass_center


class Drive(smach.State):
    def __init__(self, rate, pub_node, outcomes):
        smach.State.__init__(self, outcomes=outcomes)  # ["stop", "exit"])
        # self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node
        self.stop_distance = 1000
        self.prev_err = 0
        self.twist = Twist()

    def red_line_callback(self, msg):
        self.stop_distance = msg.data

    def image_callback(self, msg):
        mask = get_white_mask(msg)
        curr_err = path_mass_center(mask)

        if curr_err is not None:
            delta_err = curr_err - self.prev_err
            self.twist.linear.x = 0.4
            self.twist.angular.z = (-float(curr_err) / 200) + (-float(delta_err) / 200)
            self.prev_err = curr_err

            # h, w, d = image.shape
            # search_top = h * 0.75
            # search_bot = search_top + 60
            # mask[0:search_top, 0:w] = 0
            # mask[search_bot:h, 0:w] = 0
            # M = cv2.moments(mask)
            # if M["m00"] > 0:
            #     cx = int(M["m10"] / M["m00"])
            #     cy = int(M["m01"] / M["m00"])
            #     cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            #     curr_err = cx - w / 2
            #     delta_err = curr_err - self.prev_err

            # self.twist.linear.x = 0.4
            # self.twist.angular.z = (-float(curr_err) / 200) + (-float(delta_err) / 200)


class Driver(Drive):
    def __init__(self, rate, pub_node):
        super(Driver, self).__init__(rate, pub_node, ["drive_to_red_line", "exit"])

    def execute(self, userdata):
        self.stop_distance = 1000
        stop_sub = rospy.Subscriber(
            "red_line_distance", Float32, self.red_line_callback
        )
        image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        print("subbed")

        while not rospy.is_shutdown():
            print("drive...")

            # TODO: Tweak this based on red line detection
            if self.stop_distance < 0.4:
                stop_sub.unregister()
                image_sub.unregister()

                return "drive_to_red_line"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        stop_sub.unregister()
        image_sub.unregister()
        return "exit"


class DriveToRedLine(Drive):
    def __init__(self, rate, pub_node):
        super(DriveToRedLine, self).__init__(rate, pub_node, ["stop", "exit"])

    def execute(self, userdata):
        self.stop_distance = 1000
        stop_sub = rospy.Subscriber(
            "red_line_distance", Float32, self.red_line_callback
        )
        image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        while not rospy.is_shutdown():
            # TODO: Drive to red line centroid
            #       - get twist message from red_line_image_processing
            #       - publish twist message

            if self.stop_distance < 0.2:
                stop_sub.unregister()
                image_sub.unregister()
                return "stop"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        stop_sub.unregister()
        image_sub.unregister()
        return "exit"


class LineStop(smach.State):
    def __init__(self, rate, pub_node, led_pub_nodes):
        smach.State.__init__(self, outcomes=["advance", "exit"])
        self.rate = rate
        self.vel_pub = pub_node
        self.led_pubs = led_pub_nodes

    def execute(self, userdata):
        for _ in range(10):
            self.vel_pub.publish(Twist())
            self.rate.sleep()

        led_msg = Led()
        led_msg.value = Led.RED

        self.led_pubs[0].publish(led_msg)
        self.led_pubs[0].publish(led_msg)

        return "advance"


class Advancer(Drive):
    def __init__(self, rate, pub_node):
        super(Advancer, self).__init__(rate, pub_node, ["at_line", "exit"])
        self.old_stop_distance = 1000

    def execute(self, userdata):
        self.old_stop_distance = 1000
        self.twist = Twist()
        stop_sub = rospy.Subscriber(
            "red_line_distance", Float32, self.red_line_callback
        )
        image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.image_callback)
        # red_distance_change = self.old_stop_distance - self.stop_distance
        # if red_distance_change < -0.3:
        for _ in range(0, 25):
            twist = Twist()
            twist.linear.x = 0.2
            self.vel_pub.publish(twist)
            self.rate.sleep()

        stop_sub.unregister()
        image_sub.unregister()
        return "at_line"

        # self.old_stop_distance = self.stop_distance
        # self.vel_pub.publish(self.twist)
        # self.rate.sleep()


class AtLine(smach.State):
    def __init__(self, rate):
        smach.State.__init__(
            self,
            outcomes=[
                "drive",
                "turn_left_1",
                "turn_left_2_start",
                "turn_left_2_end",
                "turn_left_3",
                "exit",
            ],
        )
        self.rate = rate
        self.red_line_num = 0

        # IMPORTANT: These states currently assume that the red line at location2
        # is counted twice (for both directions)
        self.next_states = {
            1: "drive",
            2: "turn_left_1",
            3: "drive",
            4: "turn_left_2_start",
            5: "turn_left_2_end",
            6: "drive",
            7: "drive",
            7: "turn_left_3",
            8: "turn_left_3",
            9: "turn_left_3",
            10: "exit",
        }

    def execute(self, userdata):
        self.red_line_num += 1
        print("RED LINE NUMBER: ", self.red_line_num)
        return self.next_states[self.red_line_num]


class TurnRight(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(-90, self.vel_pub)
        return "drive"
