#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros
from collections import defaultdict

from operations import simple_turn
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led

from comp2.msg import Centroid
from general_states import Drive

from image_processing import (
    get_red_mask,
    get_green_mask,
    detect_shape,
    detect_green_shape,
    Shapes,
)
import cv_bridge
import cv2

the_shape = Shapes.unknown


class TurnLeft2Start(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_objects", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(65, self.vel_pub)
        return "drive_to_objects"


class DriveToObjects(Drive):
    def __init__(self, rate, pub_node, light_pubs):
        super(DriveToObjects, self).__init__(rate, pub_node, ["detect2", "exit"])
        self.light_pubs = light_pubs

    def execute(self, userdata):
        global the_shape
        white_line_sub = rospy.Subscriber(
            "white_line_centroid", Centroid, self.image_callback
        )

        # Drive until we get the cardinal value
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.twist)
            self.rate.sleep()
            image = rospy.wait_for_message("camera/rgb/image_raw", Image)
            red_mask = get_red_mask(image)
            green_mask = get_green_mask(image)
            mask = red_mask | green_mask
            canvas = cv_bridge.CvBridge().imgmsg_to_cv2(image, desired_encoding="bgr8")
            shapes, moments = detect_shape(mask, canvas=canvas)
            big_moments = [m for m in moments if m["m00"] > 3050]

            cv2.imshow("window", canvas)
            cv2.waitKey(3)
            avg_y_center = numpy.mean(
                [int(M["m01"] / (M["m00"] + 1.5)) for M in big_moments]
            )
            print("#shapes, avg y", len(big_moments), avg_y_center)
            if len(big_moments) > 0 and avg_y_center > 160:
                # Try to get a good read on the shape
                shape_count = 0
                prev_shape = Shapes.unknown
                shape_totals = defaultdict(int)
                for _ in range(90):
                    shape = detect_green_shape()
                    if shape == Shapes.unknown:
                        shape_count = 0
                        continue
                    shape_totals[shape] += 1
                    if shape == prev_shape:
                        shape_count += 1
                    else:
                        prev_shape = shape
                        shape_count = 0

                    if shape_count > 8:
                        print("discovered with a sequence")
                        break
                else:
                    print("discovered with accumulation")
                    print(shape_totals)
                    shape = max(shape_totals, key=shape_totals.get)
                    self.rate.sleep()

                    # Possibly add timeout - which sets triangle ;)

                # Maybe center the shapes
                ("SAW IMAGES")
                the_shape = shape
                print("The shape is: ", the_shape.name)
                # Show the count
                led_on_msg = Led()
                led_on_msg.value = Led.ORANGE
                led_off_msg = Led()
                led_off_msg.value = Led.BLACK
                if len(big_moments) & 0x01:
                    self.light_pubs[1].publish(led_on_msg)
                else:
                    self.light_pubs[1].publish(led_off_msg)

                if len(big_moments) & 0x02:
                    self.light_pubs[0].publish(led_on_msg)
                else:
                    self.light_pubs[0].publish(led_off_msg)
                break

        white_line_sub.unregister()
        return "detect2"


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
                twist_msg.angular.z = (-float(curr_err) / 200) + (
                    -float(delta_err) / 225
                )

            prev_err = curr_err

            self.vel_pub.publish(twist_msg)
            self.rate.sleep()

        stop_sub.unregister()
        image_sub.unregister()
        return "exit"


class Detect2(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["turn_180", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate

    def execute(self, userdata):
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
        simple_turn(40, self.vel_pub)
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
