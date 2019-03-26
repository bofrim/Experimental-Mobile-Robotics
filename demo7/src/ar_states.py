#!/usr/bin/env python
import actionlib
import rospy
import smach
import smach_ros
import time

from collections import OrderedDict

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led, BumperEvent

MIDCAM_AR_TOPIC = "ar_pose_marker_mid"
TOPCAM_AR_TOPIC = "ar_pose_marker_top"
START_POSITION = (Point(-0.3, 0.000, 0.010), Quaternion(0.000, 0.000, 0.000, 1.000))


class FindTargetLogitech(smach.State):
    def __init__(self, rate, pub_node):
        smach.State._init_(self, outcomes=["approach"], output_keys=["target_marker"])
        self.pub_node = pub_node
        self.target_marker = None

    def excecute(self, userdata):
        pass


class FindTargetLogitech(smach.State):
    def __init__(self, rate, pub_node):
        super(FindTargetLogitech, self).__init__(rate, pub_node)

    def excecute(self, userdata):
        pass


class FindTargetAuto(smach.State):
    def __init__(self, rate, pub_node):
        super(FindTargetLogitech, self).__init__(rate, pub_node)

    def excecute(self, userdata):
        pass


class Survey(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["approach"], output_keys=["box_marker"])
        self.pub_node = pub_node
        self.box_marker = None
        self.ar_focus_id = -1
        self.target_repetitions = 0

    def execute(self, userdata):
        self.box_marker = None
        rate = rospy.Rate(10)
        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        while not rospy.is_shutdown():

            if (
                self.box_marker
                and self.target_repetitions >= 3
                and -0.3 < self.box_marker.pose.pose.position.y
                and self.box_marker.pose.pose.position.y < 0.3
            ):
                userdata.box_marker = self.box_marker
                ar_sub.unregister()
                return "approach"

            twist_msg = Twist()
            twist_msg.angular.z = 0.3

            self.pub_node.publish(twist_msg)
            rate.sleep()

    def ar_callback(self, msg):
        if not msg.markers:
            self.ar_focus_id = -1
            self.target_repetitions = 0
            return

        for marker in msg.markers:
            if marker.id == self.ar_focus_id:
                self.target_repetitions = self.target_repetitions + 1
            else:
                self.ar_focus_id = marker.id
                self.target_repetitions = 0

            self.box_marker = marker
            break


class Approach(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["stop"], input_keys=["box_marker"])
        self.pub_node = pub_node
        self.box_marker = None
        self.box_marker_id = None
        self.box_marker_frame = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        self.box_marker = userdata.box_marker
        self.box_marker_id = userdata.box_marker.id
        self.box_marker_frame = "ar_marker_" + str(self.box_marker_id)

        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        print("Approach with cmd_vel")
        while self.box_marker.pose.pose.position.x > 1.2 and not rospy.is_shutdown():
            direction = self.box_marker.pose.pose.position.y / abs(
                self.box_marker.pose.pose.position.y
            )
            msg = Twist()
            msg.angular.z = 0.2 * direction
            msg.linear.x = 0.3
            self.pub_node.publish(msg)
            print("Distance = ", self.box_marker.pose.pose.position.x)

        rospy.sleep(0.2)

        print("Approach with planner")
        self.client.send_goal(self.calculate_target())
        self.client.wait_for_result()

        ar_sub.unregister()
        return "stop"

    def calculate_target(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.box_marker_frame
        goal.target_pose.pose.position = Point(0, 0, -0.7)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        return goal

    def ar_callback(self, msg):
        for marker in msg.markers:
            if marker.id == self.box_marker_id:
                self.box_marker = marker


class Stop(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["push"])
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, userdata):
        rospy.sleep(1)

        # Sometimes DriveToStart doesn't work -> Backing up to give it a better shot
        for i in range(0, 3):
            msg = Twist()
            self.pub_node.publish(msg)

        return "push"


class Push(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["start"])
        self.rate = rate
        self.pub_node = pub_node
        self.bumped = False

    def execute(self, userdata):
        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )
        bumper_sub = rospy.Subscriber(
            "mobile_base/events/bumper", BumperEvent, self.bump_callback
        )

        twist = Twist()
        twist.linear.x = 0.3

        for _ in range(0, 10):
            self.pub_node.publish(twist)
            rospy.sleep(0.2)

        # Keep pushing if the robot bumped into something (hopefully the box!)
        if self.bumped:
            for _ in range(0, 30):
                self.pub_node.publish(twist)
                rospy.sleep(0.2)

        return "start"

    def ar_callback(self, msg):
        return

    def bump_callback(self, msg):
        self.bumped = True


class DriveToStart(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["survey"])
        self.pub_node = pub_node
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.start_pose = MoveBaseGoal()
        self.start_pose.target_pose.header.frame_id = "odom"
        self.start_pose.target_pose.pose.position = START_POSITION[0]
        self.start_pose.target_pose.pose.orientation = START_POSITION[1]

    def execute(self, userdata):
        self.client.send_goal(self.start_pose)
        self.client.wait_for_result()

        return "survey"
