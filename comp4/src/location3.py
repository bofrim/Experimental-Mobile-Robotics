#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from kobuki_msgs.msg import Led
from kobuki_msgs.msg import Sound

from location2 import get_the_shape
from image_processing import (
    get_red_mask,
    get_red_mask_image_det,
    detect_shape,
    study_shapes,
)

from utils import display_count, simple_turn
from config_globals import *
from collections import defaultdict


class Detect3(smach.State):
    def __init__(self, rate, pub_node, sound_pub, light_pubs):
        self.next_states = ["turn_right_3_1", "turn_right_3_2", "turn_right_3_3"]
        smach.State.__init__(self, outcomes=self.next_states + ["exit"])
        self.rate = rate
        self.sound_pub = sound_pub
        self.pub_node = pub_node
        self.light_pubs = light_pubs
        self.count = 0

    def execute(self, userdata):
        global g3_found_flag

        twist = Twist()
        twist.linear.x = 0.2
        
        for _ in range(2):
            twist = Twist()
            twist.linear.x = 0.1
            self.pub_node.publish(twist)
            rospy.sleep(0.2)
        

        new_shape = study_shapes(
            get_red_mask, min_samples=35, topic="usb_cam/image_raw", confidence=0.5
        )
        print(get_the_shape(), new_shape)

        if get_the_shape() == new_shape and g3_found_flag == False:
            g3_found_flag = True
            sound_msg = Sound()
            sound_msg.value = Sound.ON
            self.sound_pub.publish(sound_msg)
            display_count(3, self.light_pubs)
            for _ in range(8):
                self.rate.sleep()
        else:
            display_count(0, self.light_pubs)

        
        for _ in range(2):
            twist = Twist()
            twist.linear.x = -0.1
            self.pub_node.publish(twist)
            rospy.sleep(0.2)
        
        
        next_state = self.next_states[self.count % 3]
        self.count += 1
        return next_state
