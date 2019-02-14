#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros

from operations import simple_turn
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from comp2.msg import Centroid
from general_states import Drive

from image_processing import get_red_mask, get_green_mask, detect_shape
import cv_bridge
import cv2


class TurnLeft2Start(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_objects", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        simple_turn(70, self.vel_pub)
        return "drive_to_objects"


class DriveToObjects(Drive):
    def __init__(self, rate, pub_node):
        super(DriveToObjects, self).__init__(rate, pub_node, ["detect2", "exit"])

    def execute(self, userdata):
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
            big_moments = [m for m in moments if m["m00"] > 3000]

            cv2.imshow("window", canvas)
            cv2.waitKey(3)
            avg_x_center = numpy.mean(
                [int(M["m10"] / (M["m00"] + 1.5)) for M in big_moments]
            )
            avg_y_center = numpy.mean(
                [int(M["m01"] / (M["m00"] + 1.5)) for M in big_moments]
            )
            print("#shapes, avg y", len(big_moments), avg_y_center)
            if len(big_moments) == 3 and avg_y_center > 110:
                # Maybe center the shapes
                print("SAW IMAGES")
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
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = -0.2
            else:
                twist_msg.linear.x = 0.3
                twist_msg.angular.z = (-float(curr_err) / 225) + (-float(delta_err) / 225)
        
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
        simple_turn(60, self.vel_pub)
        return "drive"


if __name__ == "__main__":
    rospy.init_node("loc2")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        image = rospy.wait_for_message("camera/rgb/image_raw", Image)
        red_mask = get_red_mask(image)
        green_mask = get_green_mask(image)
        mask = red_mask | green_mask
        canvas = cv_bridge.CvBridge().imgmsg_to_cv2(image, desired_encoding="bgr8")
        shapes, moments = detect_shape(mask, canvas)
        avg_y_center = numpy.mean([int(M["m01"] / (M["m00"] + 1.5)) for M in moments])
        big_shape_count = 0
        for m in moments:
            print("moment size:", m["m00"])
            if m["m00"] > 2300:
                big_shape_count += 1
        cv2.imshow("window", canvas)
        cv2.waitKey(3)
        print("len, avg y", big_shape_count, avg_y_center)
        rate.sleep()
