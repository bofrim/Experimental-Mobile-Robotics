#!/usr/bin/env python
# Standard Python Imports

# ROS Python Imports
import rospy
from sensor_msgs.msg import Joy


class GameController(object):
    """A generic game controller.

    Inherit from this class and fill out the callback maps with callables in the order
    that they are defined in the Joy message.
    See http://wiki.ros.org/joy#Nodes for more information on the order in which
    callback functions will be called.
    """

    def __init__(self):
        self.button_callback_map = []
        self.axis_callback_map = []
        self.subscriber = rospy.Subscriber("joy", Joy, callback=self.joy_callback)

    def joy_callback(self, message):
        """Callback for procesing a joy message."""
        for index, callback_func in enumerate(self.button_callback_map):
            callback_func(value=message.buttons[index])

        for index, callback_func in enumerate(self.axis_callback_map):
            callback_func(value=message.axes[index])

    def update_callbacks(self):
        """Call this to re-assign callback functions after they have been changed."""
        raise NotImplementedError


class LogitechGameController(GameController):
    """A skelaton implementation for a logitech game controller.

    Either inherit from this class and implement custom logic, or instantiate this
    class directly and assign new functions to the button callbacks. Example:
    
    def foo():
        ...
    ctrlr = LogitechGameController()
    ctrlr.x_button_callback = foo
    ctrlr.x_button_callback() # <-- calls foo
    """

    def __init__(self):
        super(LogitechGameController, self).__init__()
        self.update_callbacks()

    def update_callbacks(self):
        """Call this to re-assign callback functions after they have been changed."""

        self.button_callback_map = [
            self.x_button_callback,
            self.a_button_callback,
            self.b_button_callback,
            self.y_button_callback,
            self.lb_button_callback,
            self.rb_button_callback,
            self.lt_button_callback,
            self.rt_button_callback,
            self.start_button_callback,
            self.back_button_callback,
            self.left_stick_button_callback,
            self.right_stick_button_callback,
        ]

        self.axis_callback_map = [
            self.left_stick_horizontal_callback,
            self.left_stick_vertical_callback,
            self.right_stick_horizontal_callback,
            self.left_stick_vertical_callback,
            self.cross_pad_horizontal_callback,
            self.cross_pad_vertical_callback,
        ]

    def x_button_callback(self, value):
        """What happens when the x button is pressed"""
        rospy.loginfo("x Button Callback (Not re-mapped).")

    def a_button_callback(self, value):
        """What happens when the a button is pressed"""
        rospy.loginfo("a Button Callback (Not re-mapped).")

    def b_button_callback(self, value):
        """What happens when the b button is pressed"""
        rospy.loginfo("b Button Callback (Not re-mapped).")

    def y_button_callback(self, value):
        """What happens when the y button is pressed"""
        rospy.loginfo("y Button Callback (Not re-mapped).")

    def lb_button_callback(self, value):
        """What happens when the lb button is pressed"""
        rospy.loginfo("lb Button Callback (Not re-mapped).")

    def rb_button_callback(self, value):
        """What happens when the rb button is pressed"""
        rospy.loginfo("rb Button Callback (Not re-mapped).")

    def lt_button_callback(self, value):
        """What happens when the lt button is pressed"""
        rospy.loginfo("lt Button Callback (Not re-mapped).")

    def rt_button_callback(self, value):
        """What happens when the rt button is pressed"""
        rospy.loginfo("rt Button Callback (Not re-mapped).")

    def start_button_callback(self, value):
        """What happens when the start button is pressed"""
        rospy.loginfo("Start Button Callback (Not re-mapped).")

    def back_button_callback(self, value):
        """What happens when the back button is pressed"""
        rospy.loginfo("Back Button Callback (Not re-mapped).")

    def left_stick_button_callback(self, value):
        """What happens when the left stick button is pressed"""
        rospy.loginfo("Left Stick Button Callback (Not re-mapped).")

    def right_stick_button_callback(self, value):
        """What happens when the right stick button is pressed"""
        rospy.loginfo("Right Button Button Callback (Not re-mapped).")

    def left_stick_horizontal_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""
        rospy.loginfo("Left Stic Horizontal Callback (Not re-mapped).")

    def left_stick_vertical_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""
        rospy.loginfo("Left Stick Vertical Callback (Not re-mapped).")

    def right_stick_horizontal_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""
        rospy.loginfo("Right Stick Horizontal Callback (Not re-mapped).")

    def right_stick_vertical_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""
        rospy.loginfo("Right Stick Vertical Callback (Not re-mapped).")

    def cross_pad_horizontal_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""
        rospy.loginfo("Cross Pad Horizongal Callback (Not re-mapped).")

    def cross_pad_vertical_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""
        rospy.loginfo("Cross Pad Vertical Callback (Not re-mapped).")


if __name__ == "__main__":
    rospy.init_node("LogitechGameController")
    gc = LogitechGameController()

    def log_x(*args, **kwargs):
        rospy.loginfo("x")

    gc.x_button_callback = log_x
    gc.update_callbacks()

    rospy.spin()
