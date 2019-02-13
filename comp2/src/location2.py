#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist

class TurnLeft2Start(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_objects", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        return "drive_to_objects"


class DriveToObjects(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["detect_2", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        return "detect_2"

class Detect2(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["turn_180", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        return "turn_180"


class Turn180(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        return "drive"     


class TurnLeft2End(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        return "drive"
