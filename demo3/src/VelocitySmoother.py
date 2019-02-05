#!/usr/bin/env python
# Standard Python Imports

# ROS Python
import rospy
from geometry_msgs.msg import Twist

# Constants
SMOOTHER_FREQ_HZ = 10


def cap_mag(value, cap):
    """Return the value with the minimum magnitude."""
    if abs(value) < abs(cap):
        return value
    return cap


class Smoother(object):
    def __init__(
        self, max_lin_accel=0.08, max_ang_accel=0.3, max_lin_vel=2.7, max_ang_vel=5.4
    ):
        self.max_lin_accel = max_lin_accel
        self.max_ang_accel = max_ang_accel
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        self.current_lin_vel = 0.0
        self.target_lin_vel = 0.0
        self.current_ang_vel = 0.0
        self.target_ang_vel = 0.0

    def update_target(self, twist):
        self.target_lin_vel = cap_mag(twist.linear.x, self.max_lin_vel)
        self.target_ang_vel = cap_mag(twist.angular.z, self.max_ang_vel)

    def calc(self):
        linear_update = min(
            self.max_lin_accel, abs(self.current_lin_vel - self.target_lin_vel)
        )

        angular_update = min(
            self.max_ang_accel, abs(self.current_ang_vel - self.target_ang_vel)
        )

        if self.current_lin_vel < self.target_lin_vel:
            self.current_lin_vel += linear_update
        elif self.current_lin_vel > self.target_lin_vel:
            self.current_lin_vel -= linear_update

        if self.current_ang_vel < self.target_ang_vel:
            self.current_ang_vel += angular_update
        elif self.current_ang_vel > self.target_ang_vel:
            self.current_ang_vel -= angular_update

        out_twist = Twist()
        out_twist.linear.x = float(self.current_lin_vel)
        out_twist.angular.z = float(self.current_ang_vel)
        rospy.loginfo("Smooth: Output Twist = %s", str(out_twist))
        rospy.loginfo("Smoother: Update Linear: %s", str(linear_update))
        rospy.loginfo("Smoother: Angular Update: %s", str(angular_update))
        rospy.loginfo("Smoother: target lin: %s", str(self.target_lin_vel))
        rospy.loginfo("Smoother: current lin: %s", str(self.current_lin_vel))
        return out_twist


if __name__ == "__main__":
    rospy.init_node("vel_smoother")
    rate = rospy.Rate(SMOOTHER_FREQ_HZ)
    smoother = Smoother()
    sub = rospy.Subscriber("smooth_vel_in", Twist, callback=smoother.update_target)
    pub = rospy.Publisher("smooth_vel_out", Twist, queue_size=1)

    while not rospy.is_shutdown():
        pub.publish(smoother.calc())
        rate.sleep()
