#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from general_states import Drive

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led, Sound
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from comp2.msg import Centroid
from time import time
from image_processing import get_white_mask
from utils import display_count, simple_turn
from config_globals import *


class AtLineSuperb(smach.State):
    def __init__(self, rate, light_pubs, sound_pub, initial_line=0):
        smach.State.__init__(
            self,
            outcomes=[
                "drive",
                "turn_left_1",
                "turn_left_2_start",
                "turn_left_2_end",
                "off_ramp",
                "180",
                "off_ramp_180",
                "exit",
            ],
        )
        self.rate = rate
        self.red_line_num = initial_line
        self.light_pubs = light_pubs
        self.sound_pub = sound_pub

        # IMPORTANT: These states currently assume that the red line at location2
        # is counted twice (for both directions)
        self.next_states = {
            1: "drive",
            2: "turn_left_1",
            3: "drive",
            4: "turn_left_2_start",
            5: "turn_left_2_end",
            6: "off_ramp",
        }

    def execute(self, userdata):
        self.red_line_num += 1
        print("RED LINE NUMBER: ", self.red_line_num)
        display_count(0, self.light_pubs)

        if self.red_line_num >= 7:
            if self.red_line_num % 2 == 0:
                return "off_ramp_180"
            else: 
                return "180"    

        return self.next_states[self.red_line_num]


class SuperbDriver(Drive):
    def __init__(self, rate, pub_node):
        super(SuperbDriver, self).__init__(rate, pub_node, ["advance", "exit"])

    def execute(self, userdata):
        self.path_centroid = Centroid()

        self.stop_distance = -1
        stop_sub = rospy.Subscriber(
            "red_line_distance", Centroid, self.red_line_callback
        )
        white_line_sub = rospy.Subscriber(
            "white_line_ramp_centroid", Centroid, self.image_callback
        )

        while not rospy.is_shutdown():

            if self.stop_distance > 420:
                stop_sub.unregister()
                white_line_sub.unregister()

                return "advance"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        stop_sub.unregister()
        white_line_sub.unregister()
        return "exit"