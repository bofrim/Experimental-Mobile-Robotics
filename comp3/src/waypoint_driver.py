#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseWithCovariance,
    Pose,
    Point,
    Quaternion,
)
from ast import literal_eval
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

WAYPOINT_FILE_NAME = "waypoints_active.txt"

if __name__ == "__main__":
    rospy.init_node("Waypoint driver")
    pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)

    # Load waypointfile
    with open(WAYPOINT_FILE_NAME, "r") as waypointfile:
        waypoints = waypointfile.read().splitlines()

    print("\n".join(waypoints))
    print()

    # Convert the waypoints to a list of position, oritinations
    waypoint_objects = []
    for waypoint in waypoints:
        translation_str, orientation_str = tuple(waypoint.split(";"))
        print([float(axis) for axis in translation_str.split(",")], orientation_str)

        waypoint_objects.append(
            (
                Point(*[float(axis) for axis in translation_str.split(",")]),
                Quaternion(*[float(axis) for axis in orientation_str.split(",")]),
            )
        )
    print(waypoints)

    # Send the first on as the initial position
    rospy.loginfo("Sending initial pose...")
    initial_pose_cov_stamp = PoseWithCovarianceStamped()
    initial_pose_cov = PoseWithCovariance()
    initial_pose = Pose()
    initial_pose.position = waypoint_objects[0][0]
    initial_pose.orientation = waypoint_objects[0][1]
    initial_pose_cov.pose = initial_pose
    initial_pose_cov_stamp.pose = initial_pose_cov
    pose_pub.publish(initial_pose_cov_stamp)
    rospy.loginfo(" ...Sent.")

    # Iterate over the rest
    rospy.loginfo("Creating action client..."),
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("...Created.")
    for point, quaternion in waypoint_objects[1:]:
        rospy.loginfo("Nav to goal..."),
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "map"
        goal_pose.target_pose.pose.position = point
        goal_pose.target_pose.pose.orientation = quaternion
        client.send_goal(goal_pose)
        client.wait_for_result()
        rospy.loginfo("...Done.")

