#!/usr/bin/env python
# Standard Python Imports

# ROS Python Imports
import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

# Constants
CONTROLS_FREQ_HZ = 300

def scalar_multiply_twist(linear_scale, angular_scale, twist):
    "Multiply all fields of a twist by a scalar value."
    output = Twist()
    output.linear.x = linear_scale * twist.linear.x
    output.linear.y = linear_scale * twist.linear.y
    output.linear.z = linear_scale * twist.linear.z
    output.angular.x = angular_scale * twist.angular.x
    output.angular.y = angular_scale * twist.angular.y
    output.angular.z = angular_scale * twist.angular.z
    return output

def add_twists(t1, t2):
    """Just calculate the sum of t1 and t2."""
    output = Twist()
    output.linear.x = t1.linear.x + t2.linear.x
    output.linear.y = t1.linear.y + t2.linear.y
    output.linear.z = t1.linear.z + t2.linear.z
    output.angular.x = t1.angular.x + t2.angular.x
    output.angular.y = t1.angular.y + t2.angular.y
    output.angular.z = t1.angular.z + t2.angular.z
    return output

def subtract_twists(t1, t2):
    """Just calculate the difference between t1 and t2."""
    t2_complement = scalar_multiply_twist(-1, -1, t2)
    return add_twists(t1, t2_complement)

def add_n_twists(twist_list):
    """Sum a list of twists."""
    output = Twist()
    for twist in twist_list:
        output = add_twists(output, twist)
    return output

class Controller():
    """A PID controller for controlling the velocity of a robot."""

    def __init__(
        self,
        p_gain_linear=0.0,
        i_gain_linear=0.0,
        d_gain_linear=0.0,
        p_gain_angular=0.0,
        i_gain_angualr=0.0,
        d_gain_angular=0.0
    ):
        """Init the controller."""
        self.p_gain_linear = p_gain_linear
        self.i_gain_linear = i_gain_linear
        self.d_gain_linear = d_gain_linear
        self.p_gain_angular = p_gain_angular
        self.i_gain_angular = i_gain_angualr
        self.d_gain_angular = d_gain_angular
        self.error_proportional = Twist()
        self.error_integral = Twist()
        self.error_derivative = Twist()
        self.target_twist = Twist()
        self.current_twist = Twist()
        rospy.loginfo("Created a controller with gains:")
        rospy.loginfo(" - Linear P: %s", self.p_gain_linear)
        rospy.loginfo(" - Linear I: %s", self.i_gain_linear)
        rospy.loginfo(" - Linear D: %s", self.d_gain_linear)
        rospy.loginfo(" - Angular P: %s", self.p_gain_angular)
        rospy.loginfo(" - Angualr I: %s", self.i_gain_angular)
        rospy.loginfo(" - Angualr D: %s", self.d_gain_angular)
    
    def update_current_values(self, odom_message):
        """Update the position and velocity tracked by the controller.
        
        We also update the error values to reflect the new information
        """
        new_error = subtract_twists(self.target_twist, odom_message.twist.twist)
        # Note: We must update the derivative before the proportional!
        self.error_derivative = subtract_twists(new_error, self.error_proportional)
        self.error_integral = add_twists(self.error_integral, new_error)
        self.error_proportional = new_error
        
        self.current_twist = odom_message.twist.twist
    
    def update_target(self, twist_message):
        """Update the target output of the system."""
        self.target_twist = twist_message
    
    def compute_output(self):
        """Update the output twist based on the new target twist."""
        p_factor = scalar_multiply_twist(linear_scale=self.p_gain_linear, angular_scale=self.p_gain_angular, twist=self.error_proportional) 
        i_factor = scalar_multiply_twist(linear_scale=self.i_gain_linear, angular_scale=self.i_gain_angular, twist=self.error_integral)
        d_factor = scalar_multiply_twist(linear_scale=self.d_gain_linear, angular_scale=self.d_gain_angular, twist=self.error_derivative)
        return add_n_twists([p_factor, i_factor, d_factor])

    


if __name__ == "__main__":
    rospy.init_node("control_system")
    rate = rospy.Rate(CONTROLS_FREQ_HZ)
    controller = Controller(0.8, 0.05, 0.6, 0.0, 0.0, 0.0)

    input_cmd_stream = rospy.Subscriber(
        "cmd_vel",
        Twist,
        callback=controller.update_target
    )
    input_odom_stream = rospy.Subscriber(
        "odom",
        Odometry,
        callback=controller.update_current_values,
    )

    output_command_stream = rospy.Publisher(
        "cmd_vel_mux/input/teleop",
        Twist,
        queue_size=1)

    

    while not rospy.is_shutdown():
        output_command_stream.publish(controller.compute_output())
        rate.sleep()






















