#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist

class TurnLeft1(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["detect1", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(75, self.vel_pub)

        # Remove this
        for _ in range(25):
            self.vel_pub.publish(Twist())
            self.rate.sleep()

        return "detect1"


class Detect1(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["turn_right", "exit"])
        self.rate = rate

    def execute(self, userdata):
        #TODO Add Detection 
        #TODO: Show detection in lights
        return "turn_right"