#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy



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
        while not rospy.is_shutdown():
            self.rate.sleep()

            if len(self.positions) == 4:
                userdata.positions = self.positions
                joy_sub.unregister()
                return "drive"

        joy_sub.unregister()
        return "exit"


    def joy_callback(self, msg):
        # Buttons based on Controller "D" Mode
        if msg.buttons[0]:          #X
            self.positions.append((0,0))
        elif msg.buttons[1]:        #A
            self.positions.append((1,0))
        elif msg.buttons[2]:        #B
            self.positions.append((2,0))
        elif msg.buttons[3]:        #Y
            self.positions.append((3,0))
        elif msg.buttons[8]:        #Back
            self.positions = []

        print msg.buttons


class Drive(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["stop", "exit"],
                                   input_keys=["positions"])
        self.rate = rate
        self.pub_node = pub_node
        self.positions = []
        self.curr_destination = 0

    def execute(self, userdata):
        if not self.positions:
            self.positions = userdata.positions
            print self.positions

        # Pseudo code
        # while not rospy.is_shutdown:
            # if at destination:
            #    if self.curr_destination == 3:
            #       return "exit" 
            #    self.curr_destination += 1
            #    return "stop"
            #
            # else
            #    use acml and position[curr_destination] to 
            #       figure out twist message
            #    self.publish(twist)
            
      
        #return "exit"
        if self.curr_destination == 3:
            return "exit" 
        self.curr_destination += 1
        return "stop"


class Stop(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive"])
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, userdata):
        rospy.sleep(3)

        return "drive"