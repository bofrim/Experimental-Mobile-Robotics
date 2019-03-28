import rospy
import numpy as np
from geometry_msgs.msg import Twist
from comp2.msg import Centroid
from ros_numpy import numpify
from nav_msgs.msg import Odometry
from tf.transformations import decompose_matrix
import tf


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


def broadcast_box_sides(
    br,
    listen,
    relative_frame_name,
    box_frame_prefix="box",
    side_offset_from_middle=0.5,
    middle_offset_from_relative=(0, 0, -0.25),
    relative_rotation=(0, 0, 1, 0),
):
    # Publish a frame to the middle, relative to some other frame
    br.sendTransform(
        middle_offset_from_relative,
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
        front_trans = map(sum, zip(box_trans, (-side_offset_from_middle, 0, 0.010)))
        back_trans = map(sum, zip(box_trans, (side_offset_from_middle, 0, 0.010)))
        left_trans = map(sum, zip(box_trans, (0, -side_offset_from_middle, 0.010)))
        right_trans = map(sum, zip(box_trans, (0, side_offset_from_middle, 0.010)))

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


if __name__ == "__main__":
    from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers

    MIDCAM_AR_TOPIC = "ar_pose_marker_mid"
    rospy.init_node("utils")

    listen = tf.TransformListener()
    br = tf.TransformBroadcaster()

    def ar_callback(msg):
        for m in msg.markers:
            broadcast_box_sides(br, listen, "ar_marker_" + str(m.id))

    ar_sub = rospy.Subscriber(MIDCAM_AR_TOPIC, AlvarMarkers, ar_callback, queue_size=1)
    rospy.spin()

