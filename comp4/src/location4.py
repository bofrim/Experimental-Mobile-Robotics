#!/usr/bin/env python
import actionlib
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from utility_states import NavState, DriveState

from comp2.msg import Centroid
from general_states import Drive

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import (
    Twist,
    Point,
    Quaternion,
    PoseWithCovarianceStamped,
    PoseWithCovariance,
    Pose,
)

from image_processing import study_shapes, get_red_mask

from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led, Sound
from utils import display_count
from config_globals import *

from location2 import get_the_shape

class DriverRamp(Drive):
    def __init__(self, rate, pub_node):
        super(DriverRamp, self).__init__(rate, pub_node, ["start", "exit"])

    def execute(self, userdata):
        self.stop_distance = -1
        white_line_sub = rospy.Subscriber(
            "white_line_ramp_centroid", Centroid, self.image_callback
        )
        pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=1
        )

        while not rospy.is_shutdown():
            if self.path_centroid.cx == -1 or self.path_centroid.cy == -1:
                rospy.loginfo("Sending initial pose...")
                initial_pose_cov_stamp = PoseWithCovarianceStamped()
                initial_pose_cov = PoseWithCovariance()
                initial_pose = Pose()
                initial_pose = WAYPOINT_MAP["off_ramp"]
                initial_pose_cov.pose = initial_pose
                initial_pose_cov_stamp.pose = initial_pose_cov

                pose_pub.publish(initial_pose_cov_stamp)
                rospy.loginfo(" ...Sent.")

                pose_pub.unregister()
                white_line_sub.unregister()
                return "start"

            # TODO: Add in condition that robot takes the wrong path and sees a red line

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        pose_pub.unregister()
        white_line_sub.unregister()
        return "exit"


class BoxSurvey(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["tag_scan_1", "exit"])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, user_data):
        # Drive to vantage point (box 5 facing left)
        self.drive_to_vantage_point()
        # Record box position and number
        self.record_box()

    def drive_to_vantage_point(self):
        pass

    def record_box(self):
        pass


class TagScan1(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["tag_scan_2", "push_right", "exit"])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

        self.found_target = False

    def execute(self, user_data):
        self.drive_to_scan_point()
        self.scan()
        if self.because_i_still_havent_found_what_im_looking_for():
            return "tag_scan_2"
        else:
            return "push_right"

    def drive_to_scan_point(self):
        pass

    def scan(self):
        pass

    def because_i_still_havent_found_what_im_looking_for(self):
        return not self.found_target


class TagScan2(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["tag_scan_1", "push_left", "exit"])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

        self.found_target = False

    def execute(self, user_data):
        self.drive_to_scan_point()
        self.scan()
        if self.because_i_still_havent_found_what_im_looking_for():
            return "tag_scan_1"
        else:
            return "push_left"

    def drive_to_scan_point(self):
        pass

    def scan(self):
        pass

    def because_i_still_havent_found_what_im_looking_for(self):
        return not self.found_target


class PushRight(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["todo", "exit"])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, user_data):
        self.drive_to_push_point()
        self.push_to_goal()
        return "todo set as the next state"

    def drive_to_push_point(self):
        pass

    def push_to_goal(self):
        pass


class PushLeft(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["todo", "exit"])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, user_data):
        self.drive_to_push_point()
        self.push_to_goal()
        return "todo set as the next state"

    def drive_to_push_point(self):
        pass

    def push_to_goal(self):
        pass


class ShapeScan(smach.State):
    def __init__(self, rate, pub_node, led_nodes):
        smach.State.__init__(self, outcomes=["on_ramp", "exit"])
        self.rate = rate
        self.pub_node = pub_node
        self.led_nodes = led_nodes
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        for spot_num in range(8, 0, -1):
            parking_spot = MoveBaseGoal()
            parking_spot.target_pose.header.frame_id = "map"
            parking_spot.target_pose.pose = WAYPOINT_MAP[str(spot_num)]

            self.client.send_goal(parking_spot)
            self.client.wait_for_result()

            shape = study_shapes(
                get_red_mask, max_samples=60, confidence=0.5
            )        

            '''
            if shape == get_the_shape():
                display_count(2, self.led_nodes, color_primary=Led.RED)
                break 
            '''

        return "on_ramp"
        

class OnRamp(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["drive"])
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.rate = rate

    def execute(self, userdata):
        pose_prepare = MoveBaseGoal()
        pose_prepare.target_pose.header.frame_id = "map"
        pose_prepare.target_pose.pose = WAYPOINT_MAP["1"]

        self.client.send_goal(pose_prepare)
        self.client.wait_for_result()

        pose_ramp = MoveBaseGoal()
        pose_ramp.target_pose.header.frame_id = "map"
        pose_ramp.target_pose.pose = WAYPOINT_MAP["on_ramp"]

        self.client.send_goal(pose_ramp)
        self.client.wait_for_result()

        return "drive"

