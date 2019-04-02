#!/usr/bin/env python
import rospy
import numpy
from enum import Enum
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy import ROSException
from ros_numpy import numpify
from tf.transformations import decompose_matrix
from kobuki_msgs.msg import Led


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


def display_count(
    count, light_pubs, color_primary=Led.GREEN, color_secondary=Led.BLACK
):
    # light_pubs = []
    # light_pubs.append(rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1))
    # light_pubs.append(rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=1))

    led_on_msg = Led()
    led_on_msg.value = color_primary
    led_off_msg = Led()
    led_off_msg.value = color_secondary

    if count & 0x01:
        light_pubs[1].publish(led_on_msg)
    else:
        light_pubs[1].publish(led_off_msg)

    if count & 0x02:
        light_pubs[0].publish(led_on_msg)
    else:
        light_pubs[0].publish(led_off_msg)

    # light_pubs[0].unregister()
    # light_pubs[1].unregister()


def angle_ramp(desired_angle, current_angle, scale=0.3, ramp_denominator=90):
    rotation_direction = 1
    rotation_ramp = 0
    if desired_angle != current_angle:
        rotation_direction = (desired_angle - current_angle) / abs(
            desired_angle - current_angle
        )
        rotation_ramp = max(2, abs(desired_angle - current_angle) / ramp_denominator)

    return rotation_direction * rotation_ramp * scale
