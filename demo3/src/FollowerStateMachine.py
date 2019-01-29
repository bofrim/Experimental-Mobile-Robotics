#!/usr/bin/env python
# Standard Python Imports

# ROS Python
import rospy
import smach
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# This Package Python
from demo3.msg import Vector
from GameController import LogitechGameController

# Constants
MOVER_FREQ_HZ = 10


class FollowerStateMachine:
    """A Stare Machine for the mover node."""

    def __init__(self):
        """Setup the state machine."""
        # Setup ROS functionality
        self.rate = rospy.Rate(MOVER_FREQ_HZ)
        self.object_decection_subscriber = rospy.Subscriber(
            "/focusedObject", Vector, self.update_target
        )
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Setup State Variables:
        self.should_stop = True
        self.should_track = False
        self.should_follow = False
        self.should_search = False

        # Setup State Machine
        self.state_machine = smach.StateMachine(outcomes=["EXIT"])
        all_states = {
            "stop": "IDLE",
            "track": "TRACKING",
            "search": "SEARCHING",
            "follow": "FOLLOW",
            "exit": "EXIT",
        }
        with state_machine:
            smach.StateMachine.add("IDLE", self.idle_state(), transitions=all_states)
            smach.StateMachine.add(
                "FOLLOWING", self.following_state(), transitions=all_states
            )
            smach.StateMachine.add(
                "TRACKING", self.tracking_state(), transitions=all_states
            )
            smach.StateMachine.add(
                "SEARCHING", self.searching_state(), transitions=all_states
            )

        # Setup Other Variables
        self.gameController = LogitechGameController()
        self.gameController.x_button_callback = self.set_should_follow
        self.gameController.b_button_callback = self.set_should_stop
        self.gameController.a_button_callback = self.set_should_track
        self.gameController.y_button_callback = self.set_should_search

        self.target_location = Vector()

    def startup(self):
        """Start the state machine."""
        self.state_machine.execute()

    def idle_state(self):
        """Do nothing."""
        self.rate.sleep()
        return self.next_state

    def following_state(self, user_data):
        """Follow another robot"""
        self.rate.sleep()
        return self.next_state

    def tracking_state(self, user_data):
        """Spin the robot so that it is allways looking at another robot."""
        output_twist = Twist()
        if self.target_location.angle > 0:
            output_twist.angular.z = 1
        else:
            output_twist.angular.z = -1
        self.cmd_vel_publisher.publish(output_twist)
        self.rate.sleep()
        return self.next_state

    def searching_state(self, user_data):
        """Quick try to find the other robot!"""
        self.rate.sleep()
        return self.next_state

    def update_target(self, vector_messge):
        """Recive a message from the object detection node."""
        self.target_location = vector_messge

    def set_should_stop():
        rospy.loginfo("Should Stop.")
        self.should_stop = True
        self.should_follow = self.should_search = self.should_track = False

    def set_should_follow():
        rospy.loginfo("Should Follow.")
        self.should_follow = True
        self.should_stop = self.should_search = self.should_track = False

    def set_should_search():
        rospy.loginfo("Should Search.")
        self.should_search = True
        self.should_stop = self.should_follow = self.should_track = False

    def set_should_track():
        rospy.loginfo("Should Track.")
        self.should_track = True
        self.should_stop = self.should_follow = self.should_search = False

    @property
    def next_state(self):
        """Choose which state should be transitioned to."""
        if self.should_stop:
            return "stop"
        elif self.should_follow:
            return "follow"
        elif self.should_track:
            return "track"
        elif self.should_search:
            return "search"

        rospy.logerr("Could not determine next state. Exiting.")
        return "exit"


if __name__ == "__main__":
    follower_sm = FollowerStateMachine()
