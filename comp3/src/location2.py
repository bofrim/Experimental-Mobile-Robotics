#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros
from collections import defaultdict

from operations import simple_turn
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led, Sound

from comp2.msg import Centroid
from general_states import Drive

from image_processing import (
    get_red_mask,
    get_red_mask_image_det,
    get_green_mask,
    detect_shape,
    detect_green_shape,
    Shapes,
    study_shapes,
    count_objects,
)
import cv_bridge
import cv2
from utils import display_count

the_shape = Shapes.unknown


def get_the_shape():
    global the_shape
    return the_shape


class TurnLeft2Start(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_objects", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(64, self.vel_pub)
        return "drive_to_objects"


class DriveToObjects(Drive):
    def __init__(self, rate, pub_node):
        super(DriveToObjects, self).__init__(rate, pub_node, ["detect2", "exit"])
        #self.pub_node = pub_node

    def execute(self, userdata):
        global the_shape
        white_line_sub = rospy.Subscriber(
            "white_line_centroid", Centroid, self.image_callback
        )

        while not rospy.is_shutdown():

            if self.path_centroid.cx == -1 or self.path_centroid.cy == -1:
                twist = Twist()
                twist.linear.x = -0.1

                for _ in range(0, 7):
                    self.vel_pub.publish(twist)
                    rospy.sleep(0.2)

                white_line_sub.unregister()
                return "detect2"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        return "exit"


class DriveFromObjects(Drive):
    def __init__(self, rate, pub_node):
        super(DriveFromObjects, self).__init__(rate, pub_node, ["advance", "exit"])

    def execute(self, userdata):
        self.stop_distance = -1
        prev_err = 0

        stop_sub = rospy.Subscriber(
            "red_line_distance", Centroid, self.red_line_callback
        )
        image_sub = rospy.Subscriber(
            "white_line_centroid", Centroid, self.image_callback
        )

        while not rospy.is_shutdown():

            twist_msg = Twist()

            # TODO: Tweak this based on red line detection
            if self.stop_distance > 350:
                stop_sub.unregister()
                image_sub.unregister()

                return "advance"

            curr_err = self.path_centroid.err

            delta_err = curr_err - prev_err

            if self.path_centroid.cx == -1 or self.path_centroid.cy == -1:
                twist_msg.linear.x = 0.075
                twist_msg.angular.z = -0.2
            else:
                twist_msg.linear.x = 0.25
                twist_msg.angular.z = (-float(curr_err) / 150) + (
                    -float(delta_err) / 225
                )

            prev_err = curr_err

            self.vel_pub.publish(twist_msg)
            self.rate.sleep()

        stop_sub.unregister()
        image_sub.unregister()
        return "exit"


class Detect2(smach.State):
    def __init__(self, rate, sound_pub, light_pubs):
        smach.State.__init__(self, outcomes=["turn_180", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.sound_pub = sound_pub
        self.light_pubs = light_pubs

    def execute(self, userdata):
        # Maybe do some corrections
        # Count objects
        global the_shape
        the_shape = study_shapes(
            get_green_mask, min_samples=50, max_samples=150, confidence=0.6
        )

        count_tally = {1: 0, 2: 0, 3: 0}

        for _ in range(60):
            image = rospy.wait_for_message("camera/rgb/image_raw", Image)
            red_mask = get_red_mask_image_det(image)
            green_mask = get_green_mask(image)
            shape_mask = red_mask | green_mask
            count = count_objects(shape_mask, threshold=2000)
            if count in count_tally:
                count_tally[count] += 1
        display_count(max(count_tally, key=count_tally.get), self.light_pubs)
        print(count_tally)

        sound_msg = Sound()
        sound_msg.value = Sound.ON
        for i in range(max(count_tally, key=count_tally.get) - 1):
            self.sound_pub.publish(sound_msg)
            for _ in range(8):
                self.rate.sleep()
        self.sound_pub.publish(sound_msg)

        print("Counted:", max(count_tally, key=count_tally.get))
        print("The shape is: ", the_shape)
        return "turn_180"


class Turn180(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_from_objects", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(180, self.vel_pub)
        return "drive_from_objects"


class TurnLeft2End(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(43, self.vel_pub)
        return "drive"


if __name__ == "__main__":
    rospy.init_node("loc2")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # image = rospy.wait_for_message("camera/rgb/image_raw", Image)
        # red_mask = get_red_mask(image)
        # green_mask = get_green_mask(image)
        # mask = red_mask | green_mask
        # canvas = cv_bridge.CvBridge().imgmsg_to_cv2(image, desired_encoding="bgr8")
        # shapes, moments = detect_shape(mask, canvas)
        # avg_y_center = numpy.mean([int(M["m01"] / (M["m00"] + 1.5)) for M in moments])
        # big_shape_count = 0
        # for m in moments:
        #     # print("moment size:", m["m00"])
        #     if m["m00"] > 2300:
        #         big_shape_count += 1
        # cv2.imshow("window", canvas)
        # cv2.waitKey(3)
        # print("len, avg y", big_shape_count, avg_y_center)

        shape = detect_green_shape()
        green_mask = get_green_mask()
        cv2.imshow("green", green_mask)
        cv2.waitKey(3)
        print(shape.name)
        rate.sleep()
