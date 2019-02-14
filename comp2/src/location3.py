#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist

class TurnLeft3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["detect3", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(90, self.vel_pub)
        return "detect3"


class Detect3(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["turn_right", "exit"])
        self.rate = rate

    def execute(self, userdata):
        return "turn_right"