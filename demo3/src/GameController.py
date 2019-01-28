# Standard Python Imports

# ROS Python Imports
import rospy
from sensor_msgs.msg import Joy

# Constants
GAME_CONTROLLER_FREQ_HZ = 10


class GameController:
    """A generic game controller.

    Inherit from this class and fill out the callback maps with callables in the order
    that they are defined in the Joy message.
    See http://wiki.ros.org/joy#Nodes for more information on the order in which
    callback functions will be called.
    """

    self.button_callback_map = []
    self.axis_callback_map = []

    def joy_callback(self, message):
        """Callback for procesing a joy message."""
        for index, callback_func in enumerate(self.button_callback_map):
            callback_func(value=message.buttons[index])

        for index, callback_func in enumerate(self.axis_callback_map):
            callback_func(value=message.axes[index])


class LogitechGameController:
    """A skelaton implementation for a logitech game controller.

    Either inherit from this class and implement custom logic, or instantiate this
    class directly and assign new functions to the button callbacks. Example:
    
    def foo():
        ...
    ctrlr = LogitechGameController()
    ctrlr.x_button_callback = foo
    ctrlr.x_button_callback() # <-- calls foo
    """

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

    def a_button_callback(self, value):
        """What happens when the a button is pressed"""

    def b_button_callback(self, value):
        """What happens when the b button is pressed"""

    def y_button_callback(self, value):
        """What happens when the y button is pressed"""

    def lb_button_callback(self, value):
        """What happens when the lb button is pressed"""

    def rb_button_callback(self, value):
        """What happens when the rb button is pressed"""

    def lt_button_callback(self, value):
        """What happens when the lt button is pressed"""

    def rt_button_callback(self, value):
        """What happens when the rt button is pressed"""

    def start_button_callback(self, value):
        """What happens when the start button is pressed"""

    def back_button_callback(self, value):
        """What happens when the back button is pressed"""

    def left_stick_button_callback(self, value):
        """What happens when the left stick button is pressed"""

    def right_stick_button_callback(self, value):
        """What happens when the right stick button is pressed"""

    def left_stick_horizontal_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""

    def left_stick_vertical_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""

    def right_stick_horizontal_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""

    def right_stick_vertical_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""

    def cross_pad_horizontal_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""

    def cross_pad_vertical_callback(self, value):
        """Process the updated value of the horizontal axis on the left stick."""
