#!/usr/bin/env python
import rospy
from demo3.msg import ZoneScan
from geometry_msgs.msg import Twist

from Utils import closest_object_in_range, DISTANCE_LIMIT
from Constants import (
    DISTANCE_LIMIT,
    OUTER_DISTANCE_LIMIT,
    NORMAL_DISTANCE_LIMIT,
    INNER_DISTANCE_LIMIT,
    RIGHT,
    OUTER_RIGHT,
    INNER_RIGHT,
    LEFT,
    OUTER_LEFT,
    INNER_LEFT,
    ZONE_NUM,
    INF_DISTANCE,
)
from ZoneDetector import is_straight_ahead_clear_nimble

CORRECTION_FACTOR = 0.1
CORRECTION_FACTOR_RIGHT = -1 * CORRECTION_FACTOR
CORRECTION_FACTOR_LEFT = CORRECTION_FACTOR
CORRECTION_FACTOR_LINEAR = 0.5 * CORRECTION_FACTOR
INPUT_VEL_TOPIC = "avoid_vel_in"
OUTPUT_VEL_TOPIC = "avoid_vel_out"


class Avoider(object):
    def __init__(self):
        """"""
        self.pub = rospy.Publisher(OUTPUT_VEL_TOPIC, Twist, queue_size=1)
        self.vel_sub = rospy.Subscriber(
            INPUT_VEL_TOPIC, Twist, callback=self.adjust_velocity
        )
        self.zone_sub = rospy.Subscriber(
            "/zone_scan", ZoneScan, callback=self.update_zones
        )
        # self.focus_sub = rospy.Subscriber("focusedObject", Vector, update_focus)
        self.zones = [DISTANCE_LIMIT] * ZONE_NUM

    def adjust_velocity(self, twist):
        """Adjust the velocity of a twist message to attempt to avoid things."""
        print("Adjust")
        if is_straight_ahead_clear_nimble(self.zones):
            if (
                self.zones[OUTER_RIGHT] < OUTER_DISTANCE_LIMIT
                or self.zones[OUTER_RIGHT] >= INF_DISTANCE
            ):
                twist.angular.z += CORRECTION_FACTOR_LEFT
                rospy.loginfo("Correct left")
            elif (
                self.zones[OUTER_LEFT] < OUTER_DISTANCE_LIMIT
                or self.zones[OUTER_LEFT] >= INF_DISTANCE
            ):
                twist.angular.z += CORRECTION_FACTOR_RIGHT
                print("Correct right")

        if max(self.zones) >= INF_DISTANCE:
            # Actually set the linear velocity if anything is inside of the detectavle range
            twist.linear.x = -1 * CORRECTION_FACTOR_LINEAR
            print("BACKUP")
        self.pub.publish(twist)

    def update_zones(self, zone_scan):
        self.zones = zone_scan.distances


if __name__ == "__main__":
    rospy.init_node("avoider")
    avoider = Avoider()
    rospy.spin()
