#!/usr/bin/env python
import actionlib
import rospy
import smach
import smach_ros
import time
import tf

from collections import OrderedDict

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led, BumperEvent
from nav_msgs.msg import Odometry
from utils import wait_for_odom_angle, broadcast_box_sides

MIDCAM_AR_TOPIC = "ar_pose_marker_mid"
TOPCAM_AR_TOPIC = "ar_pose_marker_top"

START_POSITION = (Point(0, 0, 0.010), Quaternion(0.000, 0.000, 0.000, 1.000))
SURVEY_DRIVE_BACK_POSITION = (Point(-2.0, 0.000, 0.010), Quaternion(0.000, 0.000, 0, 1))
BOX_FRONT_POSITION = (Point(0, 0, 0.35), Quaternion(0, 0, 1, 0))
BOX_BACK_POSITION = (Point(0, 0, -0.7), Quaternion(0, 0, 0, 1))
BOX_LEFT_POSITION = (Point(0, 0.5, -0.25), Quaternion(0, 0, -0.70710678, 0.70710678))
BOX_RIGHT_POSITION = (Point(0, -0.5, -0.25), Quaternion(0, 0, 0.70710678, 0.70710678))
TARGET_FRONT_POSITION = (Point(0, 0, 0.5), Quaternion(0, 0, 1, 0))

g_target_location = Pose()


class FindTarget(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["survey"])
        self.pub_node = pub_node
        self.rate = rate
        self.listener = tf.TransformListener()

    def execute(self, userdata):
        return "survey"


class FindTargetLogitech(FindTarget):
    def __init__(self, rate, pub_node):
        super(FindTargetLogitech, self).__init__(rate, pub_node)
        self.target_found = False

    def joy_callback(self, msg):
        global g_target_location
        # Buttons based on Controller "D" Mode
        if msg.buttons[0]:  # X
            (trans, rot) = self.listener.lookupTransform(
                "/odom", "/base_link", rospy.Time(0)
            )
            g_target_location.position = Point(*trans)
            g_target_location.orientation = Quaternion(*rot) 
            self.target_found = True
        elif msg.buttons[2]:  # B
            rospy.signal_shutdown("User quit")

    def execute(self, userdata):
        joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        while not self.target_found and not rospy.is_shutdown():
            self.rate.sleep()
        print(g_target_location)
        joy_sub.unregister()
        return "survey"


class FindTargetAuto(FindTarget):
    def __init__(self, rate, pub_node):
        super(FindTargetAuto, self).__init__(rate, pub_node)
        self.ar_focus_id = -1
        self.target_repetitions = 0
        self.target_marker = None
        self.target_marker_frame = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.scan_direction = 1
        self.disable_change_direction = True
        self.found_target = False

    def execute(self, userdata):
        # survey_pose = MoveBaseGoal()
        # survey_pose.target_pose.header.frame_id = "base_link"
        # survey_pose.target_pose.pose.position = SURVEY_DRIVE_BACK_POSITION[0]
        # survey_pose.target_pose.pose.orientation = SURVEY_DRIVE_BACK_POSITION[1]
        # self.client.send_goal(survey_pose)
        # self.client.wait_for_result()

        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        while not rospy.is_shutdown():
            self.found_target = self.found_the_target()
            if self.found_target:
                self.drive_within_range()
                self.drive_to_target()
                self.extract_target_location()
                ar_sub.unregister()
                return "survey"

            self.spin_a_bit()
            self.rate.sleep()

    def extract_target_location(self):
        global g_target_location
        (trans, rot) = self.listener.lookupTransform(
            "/odom", "/base_link", rospy.Time(0)
        )
        g_target_location.position = Point(*trans)
        g_target_location.orientation = Quaternion(*rot) 

    def found_the_target(self):
        return (
            self.target_marker
            and self.target_repetitions >= 3
            and -0.6 < self.target_marker.pose.pose.position.y < 0.666
        )

    def spin_a_bit(self):
        curr_theta = wait_for_odom_angle()
        if abs(curr_theta) > 90 and not self.disable_change_direction:
            self.scan_direction *= -1
            self.disable_change_direction = True
        if abs(curr_theta) < 80:
            self.disable_change_direction = False

        twist_msg = Twist()
        twist_msg.angular.z = 0.3 * self.scan_direction
        self.pub_node.publish(twist_msg)

    def drive_within_range(self):
        while self.target_marker.pose.pose.position.x > 1.2 and not rospy.is_shutdown():
            direction = self.target_marker.pose.pose.position.y / abs(
                self.target_marker.pose.pose.position.y
            )
            msg = Twist()
            msg.angular.z = 0.2 * direction
            msg.linear.x = 0.3
            self.pub_node.publish(msg)

    def drive_to_target(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.target_marker_frame
        goal.target_pose.pose.position = TARGET_FRONT_POSITION[0]
        goal.target_pose.pose.orientation = TARGET_FRONT_POSITION[1]
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def ar_callback(self, msg):
        # if not self.found_target:
        if not msg.markers:
            self.ar_focus_id = -1
            self.target_repetitions = 0
            print("No markers")
            return

        for marker in msg.markers:
            if marker.id == self.ar_focus_id:
                self.target_repetitions = self.target_repetitions + 1
                print("got the marker")
            else:
                self.ar_focus_id = marker.id
                self.target_repetitions = 0

            self.target_marker = marker
            self.target_marker_frame = "ar_marker_" + str(marker.id)
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


class Survey(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(
            self, outcomes=["approach_par"], output_keys=["box_marker"]
        )
        self.pub_node = pub_node
        self.rate = rate
        self.box_marker = None
        self.ar_focus_id = -1
        self.target_repetitions = 0
        self.scan_direction = 1
        self.disable_change_direction = 1
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        self.box_marker = None
        rate = rospy.Rate(10)
        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        print("Look back")
        # self.look_back()
        self.drive_back_a_bit()
        print("scan")
        self.scan_for_tag()
        userdata.box_marker = self.box_marker
        ar_sub.unregister()
        return "approach_par"

    def look_back(self):
        while not rospy.is_shutdown():
            angle = wait_for_odom_angle()
            if abs(angle) > 170:
                return
            twist_msg = Twist()
            twist_msg.angular.z = 0.8
            self.pub_node.publish(twist_msg)
            self.rate.sleep()

    def scan_for_tag(self):
        while not rospy.is_shutdown():
            self.spin_a_bit()
            if self.box_found():
                return

    def spin_a_bit(self):
        """Backwards: """
        # curr_theta = wait_for_odom_angle()
        # if abs(curr_theta) < 90 and not self.disable_change_direction:
        #     self.scan_direction *= -1
        #     self.disable_change_direction = True
        # if abs(curr_theta) > 100:
        #     self.disable_change_direction = False

        # twist_msg = Twist()
        # twist_msg.angular.z = 0.2 * self.scan_direction
        # self.pub_node.publish(twist_msg)

        """Forwards: """
        curr_theta = wait_for_odom_angle()
        if abs(curr_theta) > 90 and not self.disable_change_direction:
            self.scan_direction *= -1
            self.disable_change_direction = True
        if abs(curr_theta) < 80:
            self.disable_change_direction = False

        twist_msg = Twist()
        twist_msg.angular.z = 0.2 * self.scan_direction
        self.pub_node.publish(twist_msg)

    def drive_back_a_bit(self):
        survey_pose = MoveBaseGoal()
        survey_pose.target_pose.header.frame_id = "odom"
        survey_pose.target_pose.pose.position = SURVEY_DRIVE_BACK_POSITION[0]
        survey_pose.target_pose.pose.orientation = SURVEY_DRIVE_BACK_POSITION[1]
        self.client.send_goal(survey_pose)
        self.client.wait_for_result()

    def box_found(self):
        return (
            self.box_marker
            and self.target_repetitions >= 3
            and -0.6 < self.box_marker.pose.pose.position.y < 0.6
        )

    def ar_callback(self, msg):
        if not msg.markers:
            self.ar_focus_id = -1
            self.target_repetitions = 0
            return

        for marker in msg.markers:
            if marker.id == self.ar_focus_id:
                self.target_repetitions = self.target_repetitions + 1
                print("The marker", self.target_repetitions)
            else:
                print("Not the marker", self.target_repetitions)
                self.ar_focus_id = marker.id
                self.target_repetitions = 0

            self.box_marker = marker
            break


class ApproachParallel(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["push_par"], input_keys=["box_marker"], output_keys=["box_marker"])
        self.rate = rate
        self.pub_node = pub_node
        self.box_marker = None
        self.box_marker_id = None
        self.box_marker_frame = None
        self.listen = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.waiting_for_ar = True

    def execute(self, userdata):
        self.box_marker = userdata.box_marker
        self.box_marker_id = userdata.box_marker.id
        self.box_marker_frame = "ar_marker_" + str(self.box_marker_id)

        ar_sub = rospy.Subscriber(
            "ar_pose_marker_mid", AlvarMarkers, self.ar_callback, queue_size=1
        )

        while self.waiting_for_ar:
            rospy.sleep(0.1)

        print("Approach with planner")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "box_front"
        goal.target_pose.pose.position = Point(0, 0, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        ar_sub.unregister()
        return "push_par"

    def ar_callback(self, msg):
        for m in msg.markers:
            if m.id ==  self.box_marker_id:
                broadcast_box_sides(br, listen, "ar_marker_" + str(m.id))
                self.waiting_for_ar = False
                return


class PushParallel(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["approach_perp"], input_keys=["box_marker"], output_keys=["box_marker"])
        self.rate = rate
        self.pub_node = pub_node
        self.robot_pose = None

    def execute(self, userdata):
        odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.wait_for_message("odom", Odometry)

        twist = Twist()
        twist.linear.x = 0.3

        while self.target_distance() > 0.3:
            self.pub_node.publish(twist)
            ropsy.sleep(0.2)

        back_twist = Twist()
        back_twist.linear.x = -0.2
        for _ in range(0, 25):
            self.pub_node.publish(back_twist)
            rospy.sleep(0.2)

        odom_sub.unregister()
        return "approach_perp"

    def target_distance(self):
        difference = g_target_location.position.x - self.robot_pose.position.x
        print "PAR DISTANCE: " + str(difference)
        return difference

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose


class ApproachPerpendicular(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["push_perp"], input_keys=["box_marker"])
        self.rate = rate
        self.pub_node = pub_node
        self.robot_pose = None
        self.box_marker = None
        self.box_marker_id = None
        self.box_marker_frame = None
        self.waiting_for_ar = True
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        self.box_marker = userdata.box_marker
        self.box_marker_id = userdata.box_marker.id
        self.box_marker_frame = "ar_marker_" + str(self.box_marker_id)

        odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.wait_for_message("odom", Odometry)
        ar_sub = rospy.Subscriber(
            "ar_pose_marker_mid", AlvarMarkers, self.ar_callback, queue_size=1
        )

        while self.waiting_for_ar:
            rospy.sleep(0.1)

        print("Approach with planner")
        self.client.send_goal(self.calculate_target())
        self.client.wait_for_result()

        ar_sub.unregister()
        odom_sub.unregister()
        return "push_perp"

    def calculate_target(self):
        goal = MoveBaseGoal()
        # CURRENT THOUGHT: +Y IS LEFT OF TURTLEBOT
        if g_target_location.position.y > self.robot_pose.position.y:
            goal.target_pose.header.frame_id = "box_right"
        else:
            goal.target_pose.header.frame_id = "box_left"

        goal.target_pose.pose.position = Point(0, 0, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        return goal

    def ar_callback(self, msg):
        for marker in msg.markers:
            if marker.id == self.box_marker_id:
                broadcast_box_sides(br, listen, "ar_marker_" + str(m.id))
                self.waiting_for_ar = False
                return

    def odom_callback(self, msg):
        self.robot_pose.pose = msg.pose


class PushPerpendicular(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["complete"])
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, userdata):
        odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.wait_for_message("odom", Odometry)

        twist = Twist()
        twist.linear.x = 0.3

        while -0.3 > self.target_distance() or self.target_distance() > 0.3:
            self.pub_node.publish(twist.linear.x)
            ropsy.sleep(0.2)

        back_twist = Twist()
        back_twist.linear.x = -0.2
        for _ in range(0, 25):
            self.pub_node.publish(back_twist)
            rospy.sleep(0.2)
        
        print("Get outta heeeeaa.")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position = Point(0, 0, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        odom_sub.unregister()

        return "complete"

    def target_distance(self):
        return g_target_location.position.y - self.robot_pose.position.y

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose


