# Standard Python
#from abc import ABC, abstractmethod

# ROS Python
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
            angular.z = self.angular_scale
        else:
            angular.z = -1 * self.angular_scale

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

class RampedVelocityCalculator(VelocityCalculator):
    """A Velocity calculator with ramps."""

    DELTA_ANGLE_RAMP_DEG = 7.0
    DELTA_LINEAR_RAMP_M = 0.2


    def calculate_angular(self, delta_theata):
        angular = Vector3()
        angular.z = self.angular_scale * cap_mag((float(delta_theata) - self.target_angle_deg) / RampedVelocityCalculator.DELTA_ANGLE_RAMP_DEG, cap=1)
        
        assert abs(angular.z) <= self.angular_scale, "ang z: " + str(angular.z) + " scale:" + str(self.angular_scale)
        
        print("Delta theta: ", delta_theata)
        print(angular)  
        return angular

    def calculate_linear(self, delta_distance):
        linear = Vector3()
        if delta_distance > self.target_distance_m:
            linear.x = self.linear_scale * min(1, float(abs(delta_distance - self.target_distance_m))/RampedVelocityCalculator.DELTA_LINEAR_RAMP_M)
        
        else:
            #linear.x = -1 * self.linear_scale * min(1, float(abs(delta_distance - self.target_distance_m))/RampedVelocityCalculator.DELTA_LINEAR_RAMP_M)
            linear.x = 0
        
        assert abs(linear.x) < self.linear_scale
        print("Delat disgtance: ", delta_distance)
        print(linear)
        return linear
