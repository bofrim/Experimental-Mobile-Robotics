#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led

BUTTON_WAYPOINT_MAP = {
    "X": Point(4.014, -1.205, 0.010),
    "A": Point(2.065, -2.475, 0.010),
    "B": Point(1.702, -4.093, 0.010),
    "Y": Point(3.354, -3.299, 0.010),
}


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

            if len(self.positions) == 4:
                light_bit_0.publish(light_done)
                light_bit_1.publish(light_done)
                userdata.positions = self.positions
                joy_sub.unregister()
                return "drive"

        joy_sub.unregister()
        return "exit"


    def joy_callback(self, msg):
        # Buttons based on Controller "D" Mode
        if msg.buttons[0]:          #X
            self.positions.append(BUTTON_WAYPOINT_MAP["X"])
            print("X")
        elif msg.buttons[1]:        #A
            self.positions.append(BUTTON_WAYPOINT_MAP["A"])
            print("A")
        elif msg.buttons[2]:        #B
            self.positions.append(BUTTON_WAYPOINT_MAP["B"])
            print("B")
        elif msg.buttons[3]:        #Y
            self.positions.append(BUTTON_WAYPOINT_MAP["Y"])
            print("Y")
        elif msg.buttons[8]:        #Back
            self.positions = []
            print("back")


class Drive(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["stop", "exit"],
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
        goal_pose.target_pose.pose.position = self.positions[self.destination_index]
        goal_pose.target_pose.pose.orientation.w = 1
        client.send_goal(goal_pose)
        client.wait_for_result()
        self.destination_index += 1
        if self.destination_index == len(self.positions):
            self.light_bit_0.publish(self.light_done)
            self.light_bit_1.publish(self.light_done)
            return "exit"
        return "stop"
      


class Stop(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive"])
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, userdata):
        rospy.sleep(1)
        return "drive"
