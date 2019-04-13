# Python
import numpy as np

# ROS
import rospy
from ros_numpy import numpify
from tf import ExtrapolationException, TransformBroadcaster, TransformListener
from tf.transformations import decompose_matrix, euler_from_quaternion

# ROS Messages
from geometry_msgs.msg import Twist
from comp2.msg import Centroid
from nav_msgs.msg import Odometry
from math import sin, pi, exp


def rad_to_deg(radians):
    return radians * 180 / pi


def wait_for_odom_angle(timeout=None):
    """Wait for an odometry message and extract the yaw angle."""
    odom = rospy.wait_for_message("odom", Odometry, timeout=timeout)
    return extract_angle(odom.pose.pose)


def wait_for_target_deltas(target_position, listener, frame_id="odom"):
    """Find the relative distance between base_link and a target.

    :param target_position: <Point> Target position relative to frame_id coord frame
    :param listener: tf listenter
    """
    # Todo: Use tf listenter
    odom = rospy.wait_for_message("odom", Odometry, timeout=20)
    trans = odom.pose.pose.position
    rot = odom.pose.pose.orientation
    # trans, rot = listener.lookupTransform(frame_id, "base_link", rospy.Time(0))
    theta_robot = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
    delta_x = target_position.x - trans.x
    delta_y = target_position.y - trans.y
    theta_target = rad_to_deg(sin(delta_x / delta_y))
    delta_theta = theta_target - theta_robot
    return delta_x, delta_y, delta_theta


def extract_angle(pose):
    """Extract the yaw from a pose."""
    pose = numpify(pose)
    _, _, angles, _, _ = decompose_matrix(pose)
    theta = rad_to_deg(angles[2])
    return theta


def simple_turn(angle_deg, twist_pub, max_error_deg=3, anglular_scale=1.0):
    """Turn to a specified angle.

    :param angle_deg: <float> the angle in degrees to rotate to
    :param twist_pub: <Publisher> for sending twists to the robot
    :param max_error_deg: <float> Maximum toleragle error in degrees
    :param angular_scale: <float> Maxomum angular velocity to publish
    """
    init_theta = wait_for_odom_angle()
    theta = init_theta
    direction = np.sign(angle_deg)
    target_theta = init_theta + angle_deg
    if target_theta > 180:
        target_theta = target_theta % 360 - 360
    if target_theta < -180:
        target_theta = target_theta % -360 + 360

    while abs(target_theta - theta) > max_error_deg:
        out_twist = Twist()
        out_twist.angular.z = direction * anglular_scale
        twist_pub.publish(out_twist)
        theta = wait_for_odom_angle()


def broadcast_box_sides(
    br,
    listen,
    relative_frame_name,
    box_frame_prefix="box",
    side_from_middle=0.7,
    middle_from_relative=(0, 0, -0.23),
    relative_rotation=(0, 0, 1, 0),
):
    """Publish coordinate frames of the for sides around a box.

    :param br: <Broadcaster> for sending coord frames
    :param listent: <Listener> for getting coord frames
    :param relative_frame_name: <str> name of the frame we measure relative to
    :param box_frame_prefix: <str> prefix of the name of the bradcasted coord frames
    :param side_from_middle: <float> the distance from the middle to side of the box
    :param middle_from_relative: <Tuple[float, float, float]> distance from measured
        frame to middle of the box
    :param relative_rotation: <Tuple[float, float, float, float]> Relative orientation
        from the new coord frames  to the middle of the box
    """
    # Publish a frame to the middle, relative to some other frame
    br.sendTransform(
        middle_from_relative,
        relative_rotation,
        rospy.Time.now(),
        box_frame_prefix + "_middle",
        relative_frame_name,
    )

    # Get the middle of the box relative to the global odom frame
    rospy.sleep(0.1)
    try:
        box_trans, box_rot = listen.lookupTransform(
            "odom", box_frame_prefix + "_middle", rospy.Time(0)
        )

        # Find the sides of the box
        front_trans = map(sum, zip(box_trans, (-side_from_middle, 0, 0.010)))
        back_trans = map(sum, zip(box_trans, (side_from_middle, 0, 0.010)))
        left_trans = map(sum, zip(box_trans, (0, -side_from_middle, 0.010)))
        right_trans = map(sum, zip(box_trans, (0, side_from_middle, 0.010)))

        # Publish frames to the sides of the box relative to odom
        br.sendTransform(
            front_trans, (0, 0, 0, 1), rospy.Time.now(), "box_front", "odom"
        )
        br.sendTransform(back_trans, (0, 0, 1, 0), rospy.Time.now(), "box_back", "odom")
        br.sendTransform(
            left_trans,
            (0, 0, 0.70710678, 0.70710678),
            rospy.Time.now(),
            "box_right",
            "odom",
        )
        br.sendTransform(
            right_trans,
            (0, 0, -0.70710678, 0.70710678),
            rospy.Time.now(),
            "box_left",
            "odom",
        )
        print("done")
    except ExtrapolationException as e:
        pass

