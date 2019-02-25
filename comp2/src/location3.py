#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

from location2 import the_shape
from image_processing import get_red_mask, detect_shape, Shapes

from collections import defaultdict

found_flag = False


class TurnLeft3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["detect3", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(63, self.vel_pub)
        return "detect3"


class AdjustmentTurn3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(15, self.vel_pub)
        return "drive"


class Detect3(smach.State):
    def __init__(self, rate, light_pubs, sound_pub):
        smach.State.__init__(self, outcomes=["turn_right3", "exit"])
        self.rate = rate
        self.light_pubs = light_pubs
        self.sound_pub = sound_pub

    def execute(self, userdata):
        global found_flag
        global the_shape
        shape_count = defaultdict(int)
        for _ in range(20):
            red_mask = get_red_mask()
            cv2.imshow("red_mask_3", red_mask)
            cv2.waitKey(3)

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
            led_msg = Led()
            led_msg.value = Led.GREEN
            self.light_pubs[0].publish(led_msg)
            self.light_pubs[1].publish(led_msg)
            sound_msg = Sound()
            sound_msg.value = Sound.ON
            self.sound_pub.publish(sound_msg)
        else:
            led_msg = Led()
            led_msg.value = Led.RED
            self.light_pubs[0].publish(led_msg)
            self.light_pubs[1].publish(led_msg)

        return "turn_right3"


class TurnRight3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(-70, self.vel_pub)
        return "drive"
