#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from utils import wait_for_odom_angle, angle_ramp
from comp2.msg import Centroid

g_centroid = Centroid()
g_prev_err = 0


def simple_turn(angle, twist_pub, max_error=3, anglular_scale=1.0):
    """"""
    init_theta = wait_for_odom_angle()
    theta = init_theta
    direction = np.sign(angle)
    target_theta = init_theta + angle
    # print("pre target: ", target_theta)
    if target_theta > 180:
        target_theta = target_theta % 360 - 360
    if target_theta < -180:
        target_theta = target_theta % -360 + 360
    # print("post target: ", target_theta)

    while abs(target_theta - theta) > max_error:
        # print(
        #     "target, theta, diff > max err",
        #     int(target_theta),
        #     int(theta),
        #     int(abs(target_theta - theta)),
        #     int(max_error),
        # )
        out_twist = Twist()
        out_twist.angular.z = direction * anglular_scale
        twist_pub.publish(out_twist)
        theta = wait_for_odom_angle()


def centroid_turn_callback(centroid):
    """"""
    global g_centroid
    g_centroid = centroid


def turn_to_line(approx_angle, twist_pub, max_error=10, anglular_scale=1.0):
    global g_centroid
    global g_prev_err
    simple_turn(approx_angle, twist_pub)
    white_sub = rospy.Subscriber(
        "white_line_centroid", Centroid, centroid_turn_callback
    )
    g_prev_err = 0
    while abs(g_centroid.err) > max_error or g_centroid.cx == -1:
        out_twist = Twist()
        delta_err = g_centroid.err - g_prev_err
        if g_centroid.cx == -1:
            out_twist.angular.z = 0.2 * abs(approx_angle) / float(approx_angle)
        else:
            out_twist.angular.z = (-float(g_centroid.err) / 200) + (
                -float(delta_err) / 250
            )
        twist_pub.publish(out_twist)
        g_prev_err = g_centroid.err
    white_sub.unregister()


if __name__ == "__main__":
    rospy.init_node("operator")
    twist_pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
    while not rospy.is_shutdown():
        simple_turn(90, twist_pub)
        simple_turn(-180, twist_pub)
        simple_turn(180, twist_pub)
        break
