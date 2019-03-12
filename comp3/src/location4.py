#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

class OffRamp(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        return "exit"
