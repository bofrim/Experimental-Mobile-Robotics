#!/usr/bin/env python
import rospy
import numpy
from enum import Enum
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy import ROSException
from ros_numpy import numpify
from tf.transformations import decompose_matrix
import tf
from kobuki_msgs.msg import Led


def interpolate_map(value, orig_1, orig_2, map_1, map_2):
    orig_range = orig_2 - orig_1
    map_range = map_2 - map_1
    range_frac = float(value - orig_1) / float(orig_range)
    return map_1 + (range_frac * map_range)


def point_to_distance(y_value, y_min, y_max):
    """This is fundamentally flawed, but it will work for this application."""
    return interpolate_map(y_value, y_min, y_max, TOP_DISTANCE, BOTTOM_DISTANCE)


def extract_angle(pose):
    pose = numpify(pose)
    _, _, angles, _, _ = decompose_matrix(pose)
    theta = angles[2] * 180 / 3.14159
    return theta


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


def standardize_theta(angle):
    target_theta = angle
    if angle >= 180:
        target_theta = angle % 360 - 360
    if angle < -180:
        target_theta = angle % -360 + 360
    return target_theta


def broadcast_box_sides(
    br,
    listen,
    relative_frame_name,
    box_frame_prefix="box",
    side_offset_from_middle=0.7,
    middle_offset_from_relative=(0, 0, -0.23),
    relative_rotation=(0, 0, 1, 0),
    global_frame="map"
):
    # Publish a frame to the middle, relative to some other frame
    br.sendTransform(
        middle_offset_from_relative,
        relative_rotation,
        rospy.Time.now(),
        box_frame_prefix + "_middle",
        relative_frame_name,
    )

    try:
        box_trans, box_rot = listen.lookupTransform(
            global_frame, box_frame_prefix + "_middle", rospy.Time(0)
        )

        # Find the sides of the box
        front_trans = map(sum, zip(box_trans, (-side_offset_from_middle, 0, 0.010)))
        back_trans = map(sum, zip(box_trans, (side_offset_from_middle, 0, 0.010)))
        left_trans = map(sum, zip(box_trans, (0, -side_offset_from_middle, 0.010)))
        right_trans = map(sum, zip(box_trans, (0, side_offset_from_middle, 0.010)))

        # Publish frames to the sides of the box relative to odom
        br.sendTransform(
            front_trans, (0, 0, 0, 1), rospy.Time.now(), "box_front", global_frame
        )
        br.sendTransform(back_trans, (0, 0, 1, 0), rospy.Time.now(), "box_back", global_frame)
        br.sendTransform(
            left_trans,
            (0, 0, 0.70710678, 0.70710678),
            rospy.Time.now(),
            "box_right",
            global_frame,
        )
        br.sendTransform(
            right_trans,
            (0, 0, -0.70710678, 0.70710678),
            rospy.Time.now(),
            "box_left",
            global_frame,
        )
    except tf.ExtrapolationException as e:
        print(e)
        pass
