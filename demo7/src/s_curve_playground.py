#!/usr/bin/env python
import actionlib
import rospy
import tf
import smach
import numpy as np

from math import exp
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led, BumperEvent
from nav_msgs.msg import Odometry

from utils import broadcast_box_sides, wait_for_odom_angle, wait_for_target_deltas

g_listen = None


def calc_s_curve(dx, dy, dt, num_points=10):
    """1/(1+EXP(-(X*10-5)))"""

    def calc(x, y_scale):
        return y_scale / (1 + exp(-1 * (x * 10 - 5)))

    normal_xvals = np.linspace(0, 1, num_points)
    yvals = [calc(x, dy) for x in normal_xvals]
    xvals = normal_xvals * dx
    return zip(xvals, yvals)


def calc_ang_vel(dx, dy, dt, scale=0.7):
    if dt < 0:
        return -1 * scale
    else:
        return scale


class SCurve(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["reposition", "complete", "exit"])
        self.rate = rate
        self.pub_node = pub_node
        # TODO: Get actual target
        # self.target_point = get_target_loc().position
        self.listen = tf.TransformListener()

    def execute(self, userdata):

        # Get the deltas to target
        targ_dx, targ_dy, targ_dt = wait_for_target_deltas(
            self.target_point, listener=g_listen
        )
        print(targ_dx, targ_dy)

        # Compute Waypoints
        waypoints = calc_s_curve(targ_dx, targ_dy, targ_dt, 50)

        for wp in waypoints:
            print(wp)

        # Drive
        for (wp_x, wp_y) in waypoints:
            print("Next Waypoint")
            print(wp_x, wp_y)
            # Get the deltas to waypoint
            wp_dx, wp_dy, wp_dt = wait_for_target_deltas(
                Point(wp_x, wp_y, 0), listener=self.listen
            )
            while not rospy.is_shutdown() and wp_dx > 0:
                print(wp_dx, wp_dy, wp_dt)
                # Drive
                self.send_vel(wp_dx, wp_dy, wp_dt)
                # Update the deltas to waypoint
                wp_dx, wp_dy, wp_dt = wait_for_target_deltas(
                    Point(wp_x, wp_y, 0), listener=self.listen
                )
                self.rate.sleep()

        return "complete"

    def send_vel(self, dx, dy, dt, linear_scale=0.3, angular_scale=0.3):
        twist = Twist()
        twist.linear.x = linear_scale
        twist.angular.z = calc_ang_vel(dx, dy, dt, scale=angular_scale)
        self.pub_node.publish(twist)


class Reposition(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["scurve", "exit"])
        self.rate = rate
        self.pub_node = pub_node

    def execute(self, userdata):
        self.box_marker_id = None
        self.box_marker = None
        listen = tf.TransformListener()
        br = tf.TransformBroadcaster()

        ar_sub = rospy.Subscriber(
            "ar_pose_marker_mid", AlvarMarkers, self.ar_callback, queue_size=1
        )

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "box_front"
        goal.target_pose.pose.position = Point(0, 0, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        return "drive_to_start"

    def ar_callback(self, msg):
        if not self.box_marker:
            for marker in msg.markers:
                self.box_marker = marker
                self.box_marker_id = marker.id
                return
        else:
            broadcast_box_sides(
                self.br, self.listen, "ar_marker_" + str(self.box_marker.id)
            )

        return


def main():
    global g_listen
    rospy.init_node("s_curve")

    state_machine = smach.StateMachine(outcomes=["complete", "exit"])
    g_listen = tf.TransformListener()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)

    with state_machine:
        smach.StateMachine.add(
            "SCURVE",
            SCurve(rate, cmd_vel_pub),
            transitions={
                "reposition": "REPOSITION",
                "complete": "complete",
                "exit": "exit",
            },
        )

        smach.StateMachine.add(
            "REPOSITION",
            Reposition(rate, cmd_vel_pub),
            transitions={"scurve": "SCURVE", "exit": "exit"},
        )

    state_machine.execute()


if __name__ == "__main__":
    main()
