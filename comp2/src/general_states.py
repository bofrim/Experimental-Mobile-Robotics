#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from time import time


class Drive(smach.State):
    def __init__(self, rate, pub_node, outcomes):

        smach.State.__init__(self, outcomes=outcomes)  # ["stop", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node
        self.stop_sub = rospy.Subscriber(
            "red_line_distance", Float32, self.red_line_callback
        )
        self.image_sub = rospy.Subscriber(
            "camera/rgb/image_raw", Image, self.image_callback
        )
        self.stop_distance = 1000
        self.prev_err = 0
        self.twist = Twist()

    def red_line_callback(self, msg):
        self.stop_distance = msg.data

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 170])
        upper_white = numpy.array([255, 10, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = image.shape
        search_top = h * 0.75
        search_bot = search_top + 60
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            curr_err = cx - w / 2
            delta_err = curr_err - self.prev_err

            self.twist.linear.x = 0.4
            self.twist.angular.z = (-float(curr_err) / 100) + (-float(delta_err) / 100)

        cv2.imshow("window", mask)
        cv2.waitKey(3)


class Driver(Drive):
    def __init__(self, rate, pub_node):
        super(Driver, self).__init__(rate, pub_node, ["stop", "exit"])

    def execute(self, userdata):
        while not rospy.is_shutdown():

            if self.stop_distance < 0.3:
                global red_line_num
                red_line_num += 1
                return "stop"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()


class LineStop(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["advance", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        for _ in range(50):
            self.vel_pub.publish(Twist())
            self.rate.sleep()
        return "advance"


class LineAdvancer(Drive):
    def __init__(self, rate, pub_node):
        super(LineAdvancer, self).__init__(rate, pub_node, ["redline", "exit"])

    def execute(self, userdata):
        while not rospy.is_shutdown():

            if self.stop_distance > 100:
                #TODO drive forward certain distance so that rover
                #   is centered on the red line

                return "drive"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()


class RedLine(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "location1", "location2", "location3", "exit"])
        self.rate = rate
        self.vel_pub = pub_node
        self.current_red_line = 0

        # IMPORTANT: These states currently assumme that location2 
        # redline is only scanned once
        self.next_states = {
            1: "drive",
            2: "location1",
            3: "drive",
            4: "location2",
            5: "drive",
            6: "drive",
            7: "location3",
            8: "location3",
            9: "location3",
            10: "exit"
        }

    def execute(self, userdata):
        current_red_line += 1
        return self.next_states[red_line_num]