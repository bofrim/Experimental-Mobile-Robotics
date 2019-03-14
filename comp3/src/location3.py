#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

from location2 import the_shape
from image_processing import (
    get_red_mask,
    get_red_mask_image_det,
    detect_shape,
    Shapes,
    study_shapes,
)

from utils import display_count

from collections import defaultdict

found_flag = False

g_turns = [69, 92, 80]

class TurnLeft3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["detect3", "exit"])
        self.rate = rate
        self.vel_pub = pub_node
        self.count = 0

    def execute(self, userdata):
        simple_turn(g_turns[self.count], self.vel_pub)
        self.count += 1
        return "detect3"


class Detect3(smach.State):
    def __init__(self, rate, pub_node, sound_pub):
        smach.State.__init__(self, outcomes=["turn_right3", "exit"])
        self.rate = rate
        self.sound_pub = sound_pub
        self.pub_node = pub_node

    def execute(self, userdata):
        global found_flag

        twist = Twist()
        twist.linear.x = 0.2
        for _ in range(10):
            twist = Twist()
            twist.linear.x = 0.1
            self.pub_node.publish(twist)
            rospy.sleep(0.2)

        if the_shape == study_shapes(get_red_mask, min_samples=25):
            # Found
            sound_msg = Sound()
            sound_msg.value = Sound.ON
            self.sound_pub.publish(sound_msg)
            display_count(3)
        else:
            display_count(0)

        for _ in range(10):
            twist = Twist()
            twist.linear.x = -0.1
            self.pub_node.publish(twist)
            rospy.sleep(0.2)

        return "turn_right3"
        

class TurnRight3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node
        self.count = 0

    def execute(self, userdata):
        simple_turn(-g_turns[self.count], self.vel_pub)
        self.count += 1
        return "drive"
