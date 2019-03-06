#!/usr/bin/env python
import actionlib
import rospy
import smach
import smach_ros
import time

from collections import OrderedDict

from ar_track_alvar_msgs.msg import AlvarMarker
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led

START_POSITION = (Point(0.075, -0.025, 0.010), Quaternion(0.000, 0.000, -0.029, 1.000))
TAGS_VISITED = set()

class DriveToStart(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["survey"])
        self.pub_node = pub_node
        self.start_pose = MoveBaseGoal()
        self.start_pose.target_pose.header.frame_id = 'map'
        self.start_pose.target_pose.pose.position = START_POSITION[0]
        self.start_pose.target_pose.pose.orientation = START_POSITION[1]

    def execute(self, userdata):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        client.send_goal(self.start_pose)
        client.wait_for_result()
        
        return "survey"


class Survey(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["approach"],
                                   output_keys=["target_marker"])
        self.pub_node = pub_node
        self.target_marker = None

    def execute(self, userdata):
        target_marker = None
        ar_sub = rospy.Subscriber(
            "ar_pose_marker", AlvarMarker, self.ar_callback, queue_size=1
        )
        
        while True:
            if self.target_marker:
                userdata.target_marker = self.target_marker
                ar_sub.unregister()
                return "approach"

            twist_msg = Twist()
            twist_msg.angular = 0.2

            self.pub_node.publish(twist_msg)
            rospy.sleep(0.2)

    def ar_callback(self, msg):
        if msg.id not in TAGS_VISITED:
            self.target_marker = msg


class Approach(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["stop"],
                                   output_keys=["target_marker"])
        self.pub_node = pub_node
        self.target_marker = None

    def execute(self, userdata):
        self.target_marker = userdata.target_marker
        ar_sub = rospy.Subscriber(
            "ar_pose_marker", AlvarMarker, self.ar_callback, queue_size=1
        )

        # Approach
        while self.target_marker.pose.pose.position.x > 0.025:
            direction = self.target_marker.pose.pose.position.y / abs(self.target_marker.pose.pose.position.y)
            msg = Twist()
            msg.angular = 0.2 * direction
            msg.linear = 0.3

            self.pub_node.publish(msg)

        TAGS_VISITED.add(target_marker.id)
        ar_sub.unregister()
        return "stop"
    
    def ar_callback(self, msg):
        self.target_marker = msg


class Stop(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["drive_to_start"])
        self.rate = rate

    def execute(self, userdata):
        rospy.sleep(1)
        return "drive_to_start"