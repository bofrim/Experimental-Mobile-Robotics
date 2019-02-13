#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
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
        super(Driver, self).__init__(rate, pub_node, ["drive_to_red_line", "exit"])

    def execute(self, userdata):
        while not rospy.is_shutdown():

            #TODO: Tweak this based on red line detection
            if self.stop_distance < 1.0:
                return "drive_to_red_line"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        return "exit"


class DriveToRedLine(Drive):
    def __init__(self, rate, pub_node):
        super(DriveToRedLine, self).__init__(rate, pub_node, ["stop", "exit"])

    def execute(self, userdata):
        while not ropsy.is_shutdown():
            # TODO: Drive to red line centroid
            #       - get twist message from red_line_image_processing
            #       - publish twist message


            if self.stop_distance < 0.3:
                return "stop"
        

class LineStop(smach.State):
    def __init__(self, rate, pub_node, led_pub_nodes):
        smach.State.__init__(self, outcomes=["advance", "exit"])
        self.rate = rate
        self.vel_pub = pub_node
        self.led_pubs = led_pub_nodes

    def execute(self, userdata):
        for _ in range(50):
            self.vel_pub.publish(Twist())
            self.rate.sleep()

        led_msg = Led()
        led_msg.value = Led.RED

        led_pub_nodes[0].publish(led_msg)
        led_pub_nodes[0].publish(led_msg)

        return "advance"


class Advancer(Drive):
    def __init__(self, rate, pub_node):
        super(Advancer, self).__init__(rate, pub_node, ["at_line", "exit"])
        self.old_stop_distance = 100

    def execute(self, userdata):
        while not rospy.is_shutdown():
            red_distance_change = self.old_stop_distance - self.stop_distance
            if red_distance_change < -0.2:
                # TODO: Tweak this so that the rover goes onto the red line
                for _ in range(0,4):
                    twist = Twist()
                    twist.linear.x = 0.3
                    self.vel_pub.publish(twist)

                return "at_line"

            self.old_stop_distance = self.stop_distance
            self.vel_pub.publish(self.twist)
            self.rate.sleep()


class AtLine(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["drive", "turn_left_1", "turn_left_2_start", "turn_left_2_end", "turn_left_3", "exit"])
        self.rate = rate
        self.current_red_line = 0

        # IMPORTANT: These states currently assume that the red line at location2 
        # is counted twice (for both directions)
        self.next_states = {
            1: "drive",
            2: "turn_left_1",
            3: "drive",
            4: "turn_left_2_start",
            5: "turn_left_2_end",
            6: "drive",
            7: "drive",
            7: "turn_left_3",
            8: "turn_left_3",
            9: "turn_left_3",
            10: "exit"
        }

    def execute(self, userdata):
        current_red_line += 1
        return self.next_states[red_line_num]

class TurnRight(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        return "drive"