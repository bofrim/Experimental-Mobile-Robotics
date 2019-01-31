#!/usr/bin/env python
import rospy
import smach
import smach_ros

import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from demo3.msg import ZoneScan

DISTANCE_LIMIT = 3.0
FAR_RIGHT = 0
RIGHT = 1
CLOSE_RIGHT = 2
CLOSE_LEFT = 3
LEFT = 4
FAR_LEFT = 5

class Robber():
    def __init__(self):
        rospy.init_node("robber")
        self.rate = rospy.Rate(10)
        self.bumped = False
        self.zone_distances = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.zone_sub = rospy.Subscriber("zone_scan", ZoneScan, self.zone_callback)
        self.bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_callback)

    def zone_callback(self, msg):
        self.zone_distances = msg.distances

    def bumper_callback(self, msg):
        self.bumped = bool(msg.state)

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()
            twist.angular.z = 0.0
            
            #TODO: Handle bump condition
            if self.bumped:
                print "Bumped"

            current_zones = self.zone_distances

            if current_zones[CLOSE_LEFT] <= 1.3 and current_zones[CLOSE_RIGHT] <= 1.3:
                print "IN FRONT"
                direction = (current_zones[CLOSE_LEFT] - current_zones[CLOSE_RIGHT]) / abs(current_zones[CLOSE_LEFT] - current_zones[CLOSE_RIGHT])
                twist.angular.z = direction * 0.9
            
            else:
                if current_zones[FAR_LEFT] <= 0.6 or current_zones[LEFT] <= 1.0 or current_zones[CLOSE_LEFT] <= 1.4:
                    print "TURNING RIGHT"
                    clock_vel = 1.4 - min(current_zones[FAR_LEFT], current_zones[LEFT], current_zones[CLOSE_LEFT])
                    twist.angular.z -= clock_vel

                if current_zones[FAR_RIGHT] <= 0.6 or current_zones[RIGHT] <= 1.0 or current_zones[CLOSE_RIGHT] <= 1.4:
                    print "TURNING LEFT"
                    cclock_vel = 1.4 - min(current_zones[FAR_RIGHT], current_zones[RIGHT], current_zones[CLOSE_RIGHT])
                    twist.angular.z += cclock_vel

            twist.linear.x = float(1.0 - abs(twist.angular.z))

            self.vel_pub.publish(twist)
            self.rate.sleep()

                   
if __name__ == '__main__':
    robber = Robber()
    robber.run()