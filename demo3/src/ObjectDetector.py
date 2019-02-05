#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from demo3.msg import Vector 
from Utils import closest_object_in_range, DISTANCE_LIMIT

# From LaserScan topic
#   msg.angle_min: -0.521567881107
#   msg.angle_max: 0.524276316166
#   msg.angle_increment: 0.00163668883033
#   len(msg.ranges): 640

SCAN_BOUNDARY_DELTA_DEG = 10
DISTANCE_LIMIT = 1000


class object_detector:
    def __init__(self):
        self.laser_scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.laser_scan_callback
        )
        self.distance_pub = rospy.Publisher("/focusedObject", Vector, queue_size=1)
        self.scan_min_angle = None
        self.scan_max_angle = None
        self.scan_angle_increment = None

        self.object_distance = DISTANCE_LIMIT
        self.object_angle = 0.0

    def laser_scan_callback(self, msg):
        if (
            self.scan_min_angle is None
            or self.scan_max_angle is None
            or self.scan_angle_increment is None
        ):
            self.scan_min_angle = math.degrees(msg.angle_min)
            self.scan_max_angle = math.degrees(msg.angle_max)
            self.scan_angle_increment = math.degrees(msg.angle_increment)

        scan_left_boundary = max(
            self.scan_min_angle, self.object_angle - SCAN_BOUNDARY_DELTA_DEG
        )
        scan_right_boundary = min(
            self.scan_max_angle, self.object_angle + SCAN_BOUNDARY_DELTA_DEG
        )

        angle, distance = closest_object_in_range(msg.ranges, scan_left_boundary, scan_right_boundary, self.scan_min_angle, self.scan_angle_increment)
        if distance >= DISTANCE_LIMIT:
            angle, distance = closest_object_in_range(msg.ranges, scan_left_boundary, scan_right_boundary, self.scan_min_angle, self.scan_angle_increment)
 
        self.object_distance = distance
        self.object_angle = angle

        rospy.loginfo("DISTANCE: %s", distance)
        rospy.loginfo("ANGLE: %s", angle)

        distance_msg = Vector()
        distance_msg.angle = self.object_angle
        distance_msg.magnitude = self.object_distance

        self.distance_pub.publish(distance_msg)


if __name__ == "__main__":
    rospy.init_node("object_detector")
    detector_node = object_detector()
    rospy.spin()
