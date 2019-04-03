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
from utility_control import PID


class Drive(smach.State):
    def __init__(self, rate, pub_node, pid, outcomes):
        smach.State.__init__(self, outcomes=outcomes)  # ["stop", "exit"])
        self.rate = rate
        self.vel_pub = pub_node
        self.stop_distance = -1
        self.twist = Twist()
        self.path_centroid = Centroid()
        self.stop_centroid = Centroid()
        self.speed = 0.3
        self.pid = pid

    def red_line_callback(self, msg):
        self.stop_distance = msg.cy
        self.stop_centroid = msg

    def image_callback(self, msg):
        self.pid.update_state(msg.err)
        self.path_centroid = msg
        self.twist.linear.x = self.speed
        self.twist.angular.z = self.pid.output


class Driver(Drive):
    def __init__(self, rate, pub_node):
        pid = PID(kp=-0.005, ki=-0.00, kd=-0.004, reference_value=0)
        super(Driver, self).__init__(rate, pub_node, pid, ["advance", "exit"])

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
        pid = PID(kp=-0.00055, ki=-0.00, kd=-0.00055, reference_value=0)
        super(Advancer, self).__init__(rate, pub_node, pid, ["at_line", "exit"])

    def execute(self, userdata):
        red_line_sub = rospy.Subscriber(
            "red_line_distance", Centroid, self.red_line_callback
        )
        for _ in range(0, 11):
            self.vel_pub.publish(self.twist)
            self.rate.sleep()
        self.vel_pub.publish(Twist())
        red_line_sub.unregister()
        return "at_line"


class AtLine(smach.State):
    def __init__(self, rate, light_pubs, initial_line=0):
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
        self.light_pubs = light_pubs

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
        display_count(0, self.light_pubs)
        return self.next_states[self.red_line_num]


class TurnRight(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(-84, self.vel_pub)
        # turn_to_line(-84, self.vel_pub)
        return "drive"
