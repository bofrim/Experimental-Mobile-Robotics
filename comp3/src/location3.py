#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

from location2 import the_shape
from image_processing import get_red_mask, get_red_mask_image_det, detect_shape, Shapes

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
        global the_shape
        twist = Twist()
        twist.linear.x = 0.2
        for _ in range(10):
            twist = Twist()
            twist.linear.x = 0.1
            self.pub_node.publish(twist)
            rospy.sleep(0.2)

        shape_count = defaultdict(int)
        for _ in range(20):
            red_mask = get_red_mask_image_det()

            shapes, moments = detect_shape(red_mask, threshold=500)
            shape_moments = zip(shapes, moments)

            largest_shape = Shapes.unknown
            largest_mass = 0

            for (shape, moment) in shape_moments:
                if moment["m00"] > largest_mass:
                    largest_mass = moment["m00"]
                    largest_shape = shape

            shape_count[largest_shape] += 1
            self.rate.sleep()

        discovered_shape = max(shape_count, key=shape_count.get)
        print("Accumulation:")
        print(shape_count)

        if discovered_shape == the_shape and not found_flag:
            """Found the correct shape."""
            found_flag = True
            display_count(3)
            sound_msg = Sound()
            sound_msg.value = Sound.ON
            self.sound_pub.publish(sound_msg)
        else:
            led_msg = Led()
            display_count(3, color_primary=Led.RED)

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
