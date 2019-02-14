#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from geometry_msgs.msg import Twist

from comp2.msg import Centroid
from general_states import Drive


class TurnLeft2Start(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_objects", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(90, self.vel_pub)
        return "drive_to_objects"


class DriveToObjects(Drive):
    def __init__(self, rate, pub_node):
        super(DriveToObjects, self).__init__(rate, pub_node, ["detect2", "exit"])

    def execute(self, userdata):
        white_line_sub = rospy.Subscriber(
            "white_line_centroid", Centroid, self.image_callback
        )

        # Drive until we get the cardinal value
        while not rospy.is_shutdown() and self.path_centroid.cy != -1:
            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        white_line_sub.unregister()
        return "detect2"


class Detect2(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["turn_180", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate

    def execute(self, userdata):
        return "turn_180"


class Turn180(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(180, self.vel_pub)
        return "drive"


class TurnLeft2End(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(90, self.vel_pub)
        return "drive"
