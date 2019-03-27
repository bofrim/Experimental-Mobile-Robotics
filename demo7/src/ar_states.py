#!/usr/bin/env python
import actionlib
import rospy
import smach
import smach_ros
import time
import tf

from collections import OrderedDict

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led, BumperEvent
from nav_msgs.msg import Odometry
from utils import wait_for_odom_angle

MIDCAM_AR_TOPIC = "ar_pose_marker_mid"
TOPCAM_AR_TOPIC = "ar_pose_marker_top"

START_POSITION = (Point(0, 0, 0.010), Quaternion(0.000, 0.000, 0.000, 1.000))
TARGET_SURVEY_POSITION = (
    Point(2, 0.000, 0.010),
    Quaternion(0.000, 0.000, 0.3824995, 0.9239557),
)
BOX_FRONT_POSITION = (Point(0, 0, 0.1), Quaternion(0, 0, 0, 1))
BOX_BACK_POSITION = (Point(0, 0, -0.7), Quaternion(0, 0, 0, 1))
BOX_LEFT_POSITION = (Point(0, -0.4, 0), Quaternion(0, 0, 0, 1))
BOX_RIGHT_POSITION = (Point(0, 0.4, 0), Quaternion(0, 0, 0, 1))
TARGET_FRONT_POSITION = (Point(0, 0, 0.2), Quaternion(0, 0, 1, 0))

BOX_MARKER_ID = None

g_target_location = (None, None)


class FindTarget(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_start"])
        self.pub_node = pub_node
        self.rate = rate
        self.listener = tf.TransformListener()

    def execute(self, userdata):
        return "drive_to_start"


class FindTargetLogitech(FindTarget):
    def __init__(self, rate, pub_node):
        super(FindTargetLogitech, self).__init__(rate, pub_node)
        self.target_found = False

    def joy_callback(self, msg):
        global g_target_location
        # Buttons based on Controller "D" Mode
        if msg.buttons[0]:  # X
            g_target_location = self.listener.lookupTransform(
                "/odom", "/base_link", rospy.Time(0)
            )
            self.target_found = True
        elif msg.buttons[2]:  # B
            rospy.signal_shutdown("User quit")

    def execute(self, userdata):
        joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        while not self.target_found and not rospy.is_shutdown():
            self.rate.sleep()
        print(g_target_location)
        joy_sub.unregister()
        return "drive_to_start"


class FindTargetAuto(FindTarget):
    def __init__(self, rate, pub_node):
        super(FindTargetLogitech, self).__init__(rate, pub_node)
        self.ar_focus_id = -1
        self.target_repetitions = 0
        self.box_marker = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.scan_direction = 1
        self.disable_change_direction = True

    def execute(self, userdata):
        survey_pose = MoveBaseGoal()
        survey_pose.target_pose.header.frame_id = "base_link"
        survey_pose.target_pose.pose.position = TARGET_SURVEY_POSITION[0]
        survey_pose.target_pose.pose.orientation = TARGET_SURVEY_POSITION[1]
        self.client.send_goal(self.start_pose)
        self.client.wait_for_result()

        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        while not rospy.is_shutdown():
            if self.found_the_target():
                self.extract_target_location()
                ar_sub.unregister()
                return "drive_to_start"

            self.spin_a_bit()
            rate.sleep()

    def extract_target_location(self):
        self.drive_to_target()
        g_target_location = self.listener.lookupTransform(
            "/odom", "/base_link", rospy.Time(0)
        )

    def found_the_target(self):
        return (
            self.box_marker
            and self.target_repetitions >= 3
            and -0.3 < self.box_marker.pose.pose.position.y < 0.3
        )

    def spin_a_bit(self):
        curr_theta = wait_for_odom_angle()
        if abs(curr_theta) > 90 and not self.disable_change_direction:
            self.scan_direction *= -1
            self.disable_change_dir = True
        if abs(curr_theta) < 80:
            self.disable_change_dir = False

        twist_msg = Twist()
        twist_msg.angular.z = 0.3 * self.scan_direction

        self.pub_node.publish(twist_msg)

    def drive_to_target(self):
        global g_target_location
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.box_marker_frame
        goal.target_pose.pose.position = TARGET_FRONT_POSITION[0]
        goal.target_pose.pose.orientation = TARGET_FRONT_POSITION[1]
        self.client.send_goal(goal)
        self.client.wait_for_result()

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


class Survey(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(
            self, outcomes=["approach_par"], output_keys=["box_marker"]
        )
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
                TARGET_BOX_MARKER = self.box_marker
                ar_sub.unregister()
                return "approach_par"

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


class ApproachParallel(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["push_par"], input_keys=["box_marker"])
        self.rate = rate
        self.pub_node = pub_node
        self.box_marker = None
        self.box_marker_id = None
        self.box_marker_frame = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        self.box_marker = TARGET_BOX_MARKER
        self.box_marker_id = TARGET_BOX_MARKER.id
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

        rospy.sleep(0.2)

        print("Approach with planner")
        self.client.send_goal(self.calculate_target())
        self.client.wait_for_result()

        ar_sub.unregister()
        return "push_par"

    def calculate_target(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.box_marker_frame
        goal.target_pose.pose.position = BOX_FRONT_POSITION[0]
        goal.target_pose.pose.orientation = BOX_FRONT_POSITION[1]
        return goal

    def ar_callback(self, msg):
        for marker in msg.markers:
            if marker.id == self.box_marker_id:
                self.box_marker = marker


class PushParallel(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["approach_perp"])
        self.rate = rate
        self.pub_node = pub_node
        self.robot_pose = None

    def execute(self, userdata):
        odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        # TODO: Drive until you reach a certain position
        return "approach_perp"

    def odom_callback(self, msg):
        self.robot_pose.pose = msg.pose


class ApproachPerpendicular(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["push_perp"])
        self.rate = rate
        self.pub_node = pub_node
        self.robot_pose = None
        self.box_marker = None
        self.box_marker_id = None
        self.box_marker_frame = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # curr_box_position = get_robot_pos + offset
        curr_position = None
        goal_position = None

        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        # Backup (TODO: Till we see a marker)
        twist = Twist()
        twist.linear.x = -0.2
        for _ in range(0, 10):
            self.pub_node.publish(twist)
            rospy.sleep(0.2)

        print("Approach with planner")
        self.client.send_goal(self.calculate_target(curr_position, goal_position))
        self.client.wait_for_result()

        ar_sub.unregister()
        return "push_perp"

    def calculate_target(self, curr_position, goal_position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.box_marker_frame
        goal.target_pose.pose.position = Point(0, 0, 0.2)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        return goal

    def ar_callback(self, msg):
        for marker in msg.markers:
            if marker.id == self.box_marker_id:
                self.box_marker = marker

    def odom_callback(self, msg):
        self.robot_pose.pose = msg.pose


class PushPerpendicular(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["complete"])
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, userdata):
        return "complete"

