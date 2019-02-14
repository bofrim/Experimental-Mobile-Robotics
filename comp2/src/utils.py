#!/usr/bin/env python
import rospy
import numpy
from enum import Enum
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy import ROSException
from ros_numpy import numpify
from tf.transformations import decompose_matrix


def interpolate_map(value, orig_1, orig_2, map_1, map_2):
    orig_range = orig_2 - orig_1
    map_range = map_2 - map_1
    range_frac = float(value - orig_1) / float(orig_range)
    return map_1 + (range_frac * map_range)


def point_to_distance(y_value, y_min, y_max):
    """This is fundamentally flawed, but it will work for this application."""
    return interpolate_map(y_value, y_min, y_max, TOP_DISTANCE, BOTTOM_DISTANCE)


def wait_for_odom_angle(timeout=None):
    odom = rospy.wait_for_message("odom", Odometry, timeout=timeout)
    pose = numpify(odom.pose.pose)
    _, _, angles, _, _ = decompose_matrix(pose)
    theta = angles[2] * 180 / 3.14159
    return theta


def angle_ramp(desired_angle, current_angle, scale=0.3, ramp_denominator=90):
    rotation_direction = 1
    rotation_ramp = 0
    if desired_angle != current_angle:
        rotation_direction = (desired_angle - current_angle) / abs(
            desired_angle - current_angle
        )
        rotation_ramp = max(2, abs(desired_angle - current_angle) / ramp_denominator)

    return rotation_direction * rotation_ramp * scale
