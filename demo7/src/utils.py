import rospy
import numpy as np
from geometry_msgs.msg import Twist
from comp2.msg import Centroid

g_prev_err = 0


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


def simple_turn(angle, twist_pub, max_error=3, anglular_scale=1.0):
    """"""
    init_theta = wait_for_odom_angle()
    theta = init_theta
    direction = np.sign(angle)
    target_theta = init_theta + angle
    if target_theta > 180:
        target_theta = target_theta % 360 - 360
    if target_theta < -180:
        target_theta = target_theta % -360 + 360

    while abs(target_theta - theta) > max_error:
        out_twist = Twist()
        out_twist.angular.z = direction * anglular_scale
        twist_pub.publish(out_twist)
        theta = wait_for_odom_angle()

