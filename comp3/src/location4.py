#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from comp2.msg import Centroid
from general_states import Drive

class DriverRamp(Drive):
    def __init__(self, rate, pub_node):
        super(DriverRamp, self).__init__(rate, pub_node, ["exit"])

    def execute(self, userdata):
        self.stop_distance = -1
        white_line_sub = rospy.Subscriber(
            "white_line_ramp_centroid", Centroid, self.image_callback
        )

        while not rospy.is_shutdown():
            if self.path_centroid.cx == -1 or self.path_centroid.cy == -1:
                white_line_sub.unregister()
                return "exit"

            #TODO: Add in condition that robot takes the wrong path and sees a red line

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        white_line_sub.unregister()
        return "exit"