#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from operations import simple_turn, turn_to_line
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from comp2.msg import Centroid
from time import time
from image_processing import get_white_mask
from utils import display_count


class Drive(smach.State):
    def __init__(self, rate, pub_node, outcomes):
        smach.State.__init__(self, outcomes=outcomes)  # ["stop", "exit"])
        self.rate = rate
        self.vel_pub = pub_node
        self.stop_distance = -1
        self.prev_err = 0
        self.twist = Twist()
        self.path_centroid = Centroid()
        self.stop_centroid = Centroid()
        self.speed = 0.3

    def red_line_callback(self, msg):
        self.stop_distance = msg.cy
        self.stop_centroid = msg

    def image_callback(self, msg):
        curr_err = msg.err
        self.path_centroid = msg

        delta_err = curr_err - self.prev_err
        self.twist.linear.x = self.speed
        self.twist.angular.z = (-float(curr_err) / 200) + (-float(delta_err) / 250)
        self.prev_err = curr_err


class Driver(Drive):
    def __init__(self, rate, pub_node):
        super(Driver, self).__init__(rate, pub_node, ["advance", "exit"])

    def execute(self, userdata):
        self.stop_distance = -1
        stop_sub = rospy.Subscriber(
            "red_line_distance", Centroid, self.red_line_callback
        )
        image_sub = rospy.Subscriber(
            "white_line_centroid", Centroid, self.image_callback
        )

        while not rospy.is_shutdown():

            # TODO: Tweak this based on red line detection
            if self.stop_distance > 420:
                stop_sub.unregister()
                image_sub.unregister()

                return "advance"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        stop_sub.unregister()
        image_sub.unregister()
        return "exit"


class Advancer(Drive):
    def __init__(self, rate, pub_node):
        super(Advancer, self).__init__(rate, pub_node, ["at_line", "exit"])
        prev_stop_err = 0

    def execute(self, userdata):
        prev_stop_err = 0
        self.twist = Twist()
        red_line_sub = rospy.Subscriber(
            "red_line_distance", Centroid, self.red_line_callback
        )
        for _ in range(0, 11):
            twist = Twist()

            curr_err = self.stop_centroid.err
            delta_err = curr_err - prev_stop_err
            twist.linear.x = 0.3
            twist.angular.z = (-float(curr_err) / 1800) + (-float(delta_err) / 1800)

            prev_stop_err = curr_err
            self.vel_pub.publish(twist)
            self.rate.sleep()

        for _ in range(10):
            self.vel_pub.publish(Twist())
            self.rate.sleep()

        red_line_sub.unregister()
        return "at_line"


class AtLine(smach.State):
    def __init__(self, rate, initial_line=0):
        smach.State.__init__(
            self,
            outcomes=[
                "drive",
                "turn_left_1",
                "turn_left_2_start",
                "turn_left_2_end",
                "off_ramp",
                "turn_left_3",
                "exit",
            ],
        )
        self.rate = rate
        self.red_line_num = initial_line

        # IMPORTANT: These states currently assume that the red line at location2
        # is counted twice (for both directions)
        self.next_states = {
            1: "drive",
            2: "turn_left_1",
            3: "drive",
            4: "turn_left_2_start",
            5: "turn_left_2_end",
            6: "off_ramp",
            7: "drive",
            8: "turn_left_3",
            9: "turn_left_3",
            10: "turn_left_3",
            11: "exit",
        }

    def execute(self, userdata):
        self.red_line_num += 1
        print("RED LINE NUMBER: ", self.red_line_num)
        display_count(0)
        return self.next_states[self.red_line_num]


class TurnRight(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(-84, self.vel_pub)
        #turn_to_line(-84, self.vel_pub)
        return "drive"
