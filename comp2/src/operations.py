#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from utils import wait_for_odom_angle, angle_ramp


def simple_turn(angle, twist_pub, max_error=3, anglular_scale=1.0):
    """"""
    init_theta = wait_for_odom_angle()
    theta = init_theta
    direction = np.sign(angle)
    target_theta = init_theta + angle
    print("pre target: ", target_theta)
    if target_theta > 180:
        target_theta = target_theta % 360 - 360
    if target_theta < -180:
        target_theta = target_theta % -360 + 360
    print("post target: ", target_theta)

    while abs(target_theta - theta) > max_error:
        print(
            "target, theta, diff > max err",
            int(target_theta),
            int(theta),
            int(abs(target_theta - theta)),
            int(max_error),
        )
        out_twist = Twist()
        out_twist.angular.z = direction * anglular_scale
        twist_pub.publish(out_twist)
        theta = wait_for_odom_angle()


if __name__ == "__main__":
    rospy.init_node("operator")
    twist_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
    while not rospy.is_shutdown():
        simple_turn(90, twist_pub)
        simple_turn(-180, twist_pub)
        simple_turn(180, twist_pub)
        break
