#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist

class Location3(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute():
        print "Execute Location 3"