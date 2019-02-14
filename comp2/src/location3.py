#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Led

from location2 import the_shape
from image_processing import get_red_mask, detect_shape, Shapes


class TurnLeft3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["detect3", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(65, self.vel_pub)
        return "detect3"


class AdjustmentTurn3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(15, self, vel_pub)
        return "drive"


class Detect3(smach.State):
    def __init__(self, rate, light_pubs):
        smach.State.__init__(self, outcomes=["turn_right", "exit"])
        self.rate = rate

    def execute(self, userdata):
        global the_shape
        red_mask = get_red_mask()
        shape_moments = zip(detect_shape(red_mask, threshold=500))

        largest_shape = Shapes.unknown
        largest_mass = 0

        for (shape, moment) in shape_moments:
            if moment["m00"] > largest_mass:
                largest_mass = moment["m00"]
                largest_shape = shape

        if largest_shape == the_shape:
            """Found the correct shape."""
            led_msg = Led()
            led_msg.value = Led.GREEN
            self.light_pubs[0].publish(led_msg)
            self.light_pubs[1].publish(led_msg)
        else:
            led_msg = Led()
            led_msg.value = Led.RED
            self.light_pubs[0].publish(led_msg)
            self.light_pubs[1].publish(led_msg)

        return "turn_right"
