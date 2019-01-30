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
from VelocityCalculator import SimpleVelocityCalculator, RampedVelocityCalculator

# Constants
FOLLOWER_FREQ_HZ = 10


class wrapper_state(smach.State):
    def __init__(self, callable_func, outcomes):
        """Init the state."""
        smach.State.__init__(self, outcomes=outcomes)
        self.execute_func = callable_func

    def execute(self, callable_func):
        """Run the state."""
        return self.execute_func()


class FollowerStateMachine(object):
    """A Stare Machine for the mover node."""

    def __init__(self):
        """Setup the state machine."""
        # Setup ROS functionality
        self.rate = rospy.Rate(FOLLOWER_FREQ_HZ)
        self.object_decection_subscriber = rospy.Subscriber(
            "/focusedObject", Vector, self.update_target
        )
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Setup State Variables:
        self.should_stop = True
        self.should_track = False
        self.should_follow = False
        self.should_search = False
        self.should_exit = False
        self._next_state = "stop"

        # Setup State Machine
        self.state_machine = smach.StateMachine(outcomes=["EXIT"])
        all_states = {
            "stop": "IDLE",
            "track": "TRACKING",
            "search": "SEARCHING",
            "follow": "FOLLOWING",
            "exit": "EXIT",
        }
        with self.state_machine:
            smach.StateMachine.add(
                "IDLE",
                wrapper_state(self.idle_state, all_states),
                transitions=all_states,
            )
            smach.StateMachine.add(
                "FOLLOWING",
                wrapper_state(self.following_state, all_states),
                transitions=all_states,
            )
            smach.StateMachine.add(
                "TRACKING",
                wrapper_state(self.tracking_state, all_states),
                transitions=all_states,
            )
            smach.StateMachine.add(
                "SEARCHING",
                wrapper_state(self.searching_state, all_states),
                transitions=all_states,
            )

        # Setup Other Variables
        self.gameController = LogitechGameController()
        self.gameController.x_button_callback = self.set_should_follow
        self.gameController.b_button_callback = self.set_should_stop
        self.gameController.a_button_callback = self.set_should_track
        self.gameController.y_button_callback = self.set_should_search
        self.gameController.start_button_callback = self.set_should_exit
        self.gameController.update_callbacks()

        self.velocity_calculator = SimpleVelocityCalculator(
            linear_scale=0.5,
            angular_scale=0.7,
            target_distance_m=1.0,
            target_angle_deg=0,
        )
        self.target_location = Vector()

    def startup(self):
        """Start the state machine."""
        self.state_machine.execute()

    def idle_state(self):
        """Do nothing."""
        self.rate.sleep()
        return self.next_state

    def following_state(self):
        """Follow another robot"""
        output_twist = Twist()
        output_twist.angular = self.velocity_calculator.calculate_angular(
            self.target_location.angle
        )
        output_twist.linear = self.velocity_calculator.calculate_linear(
            self.target_location.magnitude
        )
        self.cmd_vel_publisher.publish(output_twist)
        self.rate.sleep()
        return self.next_state

    def tracking_state(self):
        """Spin the robot so that it is allways looking at another robot."""
        output_twist = Twist()
        output_twist.angular = self.velocity_calculator.calculate_angular(
            self.target_location.angle
        )
        self.cmd_vel_publisher.publish(output_twist)
        self.rate.sleep()
        return self.next_state

    def searching_state(self):
        """Quick try to find the other robot!"""
        self.rate.sleep()
        return self.next_state

    def update_target(self, vector_messge):
        """Recive a message from the object detection node."""
        self.target_location = vector_messge
        print("Updating target: ", self.target_location)

    def set_should_stop(self, *args, **kwargs):
        self.should_stop = bool(kwargs["value"])

    def set_should_follow(self, *args, **kwargs):
        self.should_follow = bool(kwargs["value"])

    def set_should_search(self, *args, **kwargs):
        self.should_search = bool(kwargs["value"])

    def set_should_track(self, *args, **kwargs):
        self.should_track = bool(kwargs["value"])

    def set_should_exit(self, *args, **kwargs):
        self.should_exit = bool(kwargs["value"])

    @property
    def next_state(self):
        """Choose which state should be transitioned to."""
        if self.should_stop:
            self._next_state = "stop"
        elif self.should_follow:
            self._next_state = "follow"
        elif self.should_track:
            self._next_state = "track"
        elif self.should_search:
            self._next_state = "search"
        elif self.should_exit:
            self._next_state = "exit"

        return self._next_state


if __name__ == "__main__":
    rospy.init_node("follower_state_machine")
    follower_sm = FollowerStateMachine()
    follower_sm.startup()

    while not rospy.is_shutdown():
        """Spin"""
