#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from demo3.msg import ZoneScan
from Utils import closest_object_in_range, DISTANCE_LIMIT

ZONE_NUM = 6

class object_detector():
    def __init__(self):
        self.laser_scan_sub = rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        self.zone_pub = rospy.Publisher("/zoneScan", ZoneScan, queue_size=1)
        self.scan_min_angle = None
        self.scan_max_angle = None
        self.scan_angle_increment = None

        self.object_distance = DISTANCE_LIMIT
        self.object_angle = 0.0

    def laser_scan_callback(self, msg):
        if self.scan_min_angle is None or self.scan_max_angle is None or self.scan_angle_increment is None:
            self.scan_min_angle = math.degrees(msg.angle_min)
            self.scan_max_angle = math.degrees(msg.angle_max)
            self.scan_angle_increment = math.degrees(msg.angle_increment)

        zone_span = (self.scan_max_angle - self.scan_min_angle) / ZONE_NUM
        zone_distances = [ self.zone_distance(msg.ranges, self.scan_min_angle + (zone_span * x), zone_span, self.scan_min_angle, self.scan_angle_increment) for x in range(0, ZONE_NUM)]        

        print "DISTANCES: " + str(zone_distances)

        zone_msg = ZoneScan()
        zone_msg.distances = zone_distances
        self.zone_pub.publish(zone_msg)

    def zone_distance(self, ranges, left_zone_bound, zone_span, scan_min_angle, scan_angle_increment):
        right_zone_bound = left_zone_bound + zone_span - self.scan_angle_increment
        _, distance = closest_object_in_range(ranges, left_zone_bound, right_zone_bound, self.scan_min_angle, self.scan_angle_increment)
            
        return distance


if __name__ == "__main__":
    rospy.init_node('object_detector')
    detector_node = object_detector()
    rospy.spin()
