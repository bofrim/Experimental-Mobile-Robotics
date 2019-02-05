# Standard Python
# from abc import ABC, abstractmethod

# ROS Python
import rospy
from geometry_msgs.msg import Vector3


def cap_mag(value, cap):
    """Return the value with the minimum magnitude."""
    if abs(value) < abs(cap):
        return value
    return cap


class VelocityCalculator(object):
    """Abstract velocity calculator."""

    def __init__(
        self, linear_scale=1, angular_scale=1, target_distance_m=1.1, target_angle_deg=0
    ):
        self.linear_scale = float(linear_scale)
        self.angular_scale = float(angular_scale)
        self.target_distance_m = float(target_distance_m)
        self.target_angle_deg = float(target_angle_deg)

    #    @abstractmethod
    def calculate_angular(self, delta_theta):
        """Calculate an angular velocity to output to the robot base."""

    #    @abstractmethod
    def calculate_linear(self, delta_distance):
        """Calculate a linear velocity to output to the robot base."""


class SimpleVelocityCalculator(VelocityCalculator):
    """The most simple velocity calculator."""

    def calculate_angular(self, delta_theta):
        """Naive approach to try to make delta theta target angle."""
        angular = Vector3()
        if delta_theta > self.target_angle_deg:
            angular.z = self.angular_scale  # * abs(delta_theta)
        else:
            angular.z = -1 * self.angular_scale  # * abs(delta_theta)

        return angular

    def calculate_linear(self, delta_distance):
        """Naive approach to try and make delta distance equal target distance.
        
        Only move forward.
        """
        linear = Vector3()
        print(delta_distance, self.target_distance_m)
        if delta_distance > self.target_distance_m:
            linear.x = self.linear_scale

        return linear


class AttenuatedVelocityCalculator(VelocityCalculator):
    """Attenuate velocity as the taraget nears."""

    def calculate_angular(self, delta_theta, attenuation_threshold_deg=20):
        """Naive approach to try to make delta theta target angle."""
        angular = Vector3()
        if delta_theta > self.target_angle_deg:
            angular.z = self.angular_scale
        else:
            angular.z = -1 * self.angular_scale

        is_in_att_range = (
            abs(self.target_angle_deg - delta_theta) < attenuation_threshold_deg
        )
        rospy.loginfo("target_angle_deg: %s", self.target_angle_deg)
        rospy.loginfo("abs(delta_theta): %s", abs(delta_theta))
        rospy.loginfo("difference: %s", abs(self.target_angle_deg - delta_theta))
        rospy.loginfo("attenuation_thresh: %s", attenuation_threshold_deg)
        rospy.loginfo("Is being attenuated: %s", is_in_att_range)
        if is_in_att_range:
            angular.z *= float(abs(self.target_angle_deg - delta_theta)) / float(
                attenuation_threshold_deg
            )

        return angular

    def calculate_linear(self, delta_distance, attenuation_threshold_m=0.1):
        """Naive approach to try and make delta distance equal target distance.
        
        Only move forward.
        """
        linear = Vector3()
        if delta_distance > self.target_distance_m:
            linear.x = self.linear_scale

        return linear
