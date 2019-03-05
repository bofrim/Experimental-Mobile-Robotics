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

BUTTON_WAYPOINT_MAP = OrderedDict([
    ("B", (Point(1.623, -3.766, 0.010), Quaternion(0.000, 0.000, 0.242, 0.970))),
    ("Y", (Point(3.457, -3.328, 0.010), Quaternion(0.000, 0.000, 0.509, 0.861))),
    ("X", (Point(4.183, -1.455, 0.010), Quaternion(0.000, 0.000, 0.959, -0.282)))
])


class InitWaypoints(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["drive", "exit"],
                                   output_keys=["positions"])
        self.rate = rate
        self.positions = []

    def execute(self, userdata):
        joy_sub = rospy.Subscriber(
            "joy", Joy, self.joy_callback, queue_size=1
        )
        light_bit_1 = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1)
        light_bit_0 = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=1)
        light_off = Led(Led.BLACK)
        light_on = Led(Led.ORANGE)
        light_done = Led(Led.GREEN)
        while not rospy.is_shutdown():
            self.rate.sleep()

            if len(self.positions) & 0x01:
                light_bit_0.publish(light_on)
            else:
                light_bit_0.publish(light_off)
            if len(self.positions) & 0x02:
                light_bit_1.publish(light_on)
            else:
                light_bit_1.publish(light_off)

            if len(self.positions) == 3:
                light_bit_0.publish(light_done)
                light_bit_1.publish(light_done)
                userdata.positions = self.positions
                joy_sub.unregister()
                return "drive"

        joy_sub.unregister()
        return "exit"


    def joy_callback(self, msg):
        # Buttons based on Controller "D" Mode
        if msg.buttons[9]:          #START
            self.positions = BUTTON_WAYPOINT_MAP.values()
            print("START")
        if msg.buttons[0]:          #X
            self.positions.append(BUTTON_WAYPOINT_MAP["X"])
            print("X")
        elif msg.buttons[2]:        #B
            self.positions.append(BUTTON_WAYPOINT_MAP["B"])
            print("B")
        elif msg.buttons[3]:        #Y
            self.positions.append(BUTTON_WAYPOINT_MAP["Y"])
            print("Y")
        elif msg.buttons[8]:        #Back
            self.positions = []
            print("RESET")


class Drive(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["approach", "exit"],
                                   input_keys=["positions"])
        self.rate = rate
        self.pub_node = pub_node
        self.positions = []
        self.destination_index = 0
        self.light_bit_1 = rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1)
        self.light_bit_0 = rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=1)
        self.light_off = Led(Led.BLACK)
        self.light_on = Led(Led.RED)
        self.light_done = Led(Led.GREEN)

    def execute(self, userdata):
        """Send a goal to the goal server and wait until we get there.
        
        Go to the stop state if there are still positions left, otherwise exit.
        """
        if not self.positions:
            self.positions = userdata.positions

        if self.destination_index == len(self.positions):
            self.light_bit_0.publish(self.light_done)
            self.light_bit_1.publish(self.light_done)
            return "exit"
            
        if self.destination_index & 0x01:
            self.light_bit_0.publish(self.light_on)
        else:
            self.light_bit_0.publish(self.light_off)

        if self.destination_index & 0x02:
            self.light_bit_1.publish(self.light_on)
        else:
            self.light_bit_1.publish(self.light_off)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position = self.positions[self.destination_index][0]
        goal_pose.target_pose.pose.orientation = self.positions[self.destination_index][1]

        client.send_goal(goal_pose)
        client.wait_for_result()

        self.destination_index += 1
        return "approach"
      

class Approach(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["stop"])
        self.rate = rate
        self.pub_node = pub_node
        self.target_pose = None

    def execute(self, userdata):
        ar_sub = rospy.Subscriber(
            "ar_pose_marker", AlvarMarker, self.ar_callback, queue_size=1
        )
        start = time.time()
        while (time.time() - start < 5):
            if self.target_pose:
                print self.target_pose
                ar_sub.unregister()
                return "stop"
            
        ar_sub.unregister()
        return "stop"

    def ar_callback(self, msg):
        self.target_pose = msg.pose.pose


class Stop(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive"])
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, userdata):
        rospy.sleep(1)
        return "drive"
