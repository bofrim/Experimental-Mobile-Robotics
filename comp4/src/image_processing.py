#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from enum import Enum

from config_globals import *

def threshold_hsv_360(hsv, h_max, h_min, s_max, s_min, v_max, v_min, denoise=0, fill=0):
    """Taken from:
    
    https://eclass.srv.ualberta.ca/pluginfile.php/4909552/mod_folder/content/0/Lab5/CMPUT%20412%20Lab%205.pdf?forcedownload=1
    """
    lower_color_range_0 = np.array([0, s_min, v_min], dtype=float)
    upper_color_range_0 = np.array([h_max / 2.0, s_max, v_max], dtype=float)
    lower_color_range_360 = np.array([h_min / 2.0, s_min, v_min], dtype=float)
    upper_color_range_360 = np.array([360 / 2.0, s_max, v_max], dtype=float)
    mask0 = cv2.inRange(hsv, lower_color_range_0, upper_color_range_0)
    mask360 = cv2.inRange(hsv, lower_color_range_360, upper_color_range_360)
    mask = mask0 | mask360
    if denoise > 0:
        kernel = np.ones((denoise, denoise), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if fill > 0:
        kernel = np.ones((fill, fill), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def hsv_bound(image, upper_bound, lower_bound, denoise=0, fill=0):
    """Create a mask exposing only regions fall within bounds.
    
    Basically just a wrapper for threshold_hsv_360.
    """
    return threshold_hsv_360(
        image,
        upper_bound[0],
        lower_bound[0],
        upper_bound[1],
        lower_bound[1],
        upper_bound[2],
        lower_bound[2],
        denoise=denoise,
        fill=fill,
    )


def detect_shape(mask, canvas=None, threshold=100, approx_factor=0.03):
    """Detect a shape contained in an image.
    
    Adapted from: https://stackoverflow.com/questions/11424002/how-to-detect-simple-geometric-shapes-using-opencv
    """
    detected_shapes = []
    moments = []
    _, contours, _ = cv2.findContours(mask, 1, 2)
    for cnt in contours:
        if cv2.moments(cnt)["m00"] > threshold:
            approx = cv2.approxPolyDP(
                cnt, approx_factor * cv2.arcLength(cnt, True), True
            )
            if len(approx) == 3:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 255, 0), -1)
                detected_shapes.append(Shapes.triangle)
                moments.append(cv2.moments(cnt))

            elif len(approx) == 4:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 0, 255), -1)
                detected_shapes.append(Shapes.square)
                moments.append(cv2.moments(cnt))
            elif len(approx) == 5:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, 255, -1)
                detected_shapes.append(Shapes.pentagon)
                moments.append(cv2.moments(cnt))
            elif len(approx) >= 6:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 255, 255), -1)
                detected_shapes.append(Shapes.circle)
                moments.append(cv2.moments(cnt))
            else:
                detected_shapes.append(Shapes.unknown)
                moments.append(cv2.moments(cnt))
    if canvas != None:
        cv2.imshow("shape", canvas)
        cv2.waitKey(1)

    return detected_shapes, moments


def study_shapes(
    mask_func,
    min_samples=20,
    max_samples=100,
    confidence=0.5,
    mass_threshold=1000,
    rate=None,
    topic="/camera/rgb/image_raw",
    approx_factor=0.03,
):
    if rate is None:
        rate = rospy.Rate(20)

    sample_count = 1
    square_count = 0
    triangle_count = 0
    circle_count = 0
    print("start study")

    while not rospy.is_shutdown():
        mask = mask_func(topic=topic)
        shapes, _ = detect_shape(
            mask, threshold=mass_threshold, approx_factor=approx_factor
        )
        if len(shapes) != 1:
            # Skip, there were too few or too many shapes
            continue

        if shapes[0] == Shapes.square:
            print(
                "square confidence: " + str(float(square_count) / float(sample_count))
            )
            square_count += 1
        elif shapes[0] == Shapes.triangle:
            print(
                "triangle confidence: "
                + str(float(triangle_count) / float(sample_count))
            )
            triangle_count += 1
        elif shapes[0] == Shapes.circle:
            print(
                "circle confidence: " + str(float(circle_count) / float(sample_count))
            )
            circle_count += 1

        if min_samples <= sample_count < max_samples:
            if float(square_count) / float(sample_count) > confidence:
                return Shapes.square
            elif float(triangle_count) / float(sample_count) > confidence:
                return Shapes.triangle
            elif float(circle_count) / float(sample_count) > confidence:
                return Shapes.circle
        elif max_samples <= sample_count:
            return Shapes.unknown

        print(
            str(sample_count)
            + " / ["
            + str(min_samples)
            + ", "
            + str(max_samples)
            + "]"
        )
        sample_count += 1
        rate.sleep()
    print("stop study")


def lowest_object_coord(mask, threshold=100):
    """Find the coordinates of the lowes object in the mask.
    
    Note: The lowest object corresponds to the largest y coord
    """
    _, contours, _ = cv2.findContours(mask, 1, 2)
    moments = [cv2.moments(c) for c in contours if cv2.moments(c)["m00"] > threshold]
    lowest_y = -1
    lowest_coord = (0, lowest_y)
    for m in moments:
        x, y = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
        if y > lowest_y:
            lowest_y = y
            lowest_coord = (x, y)
    return lowest_coord


def count_objects(mask, threshold=1000, canvas=None):
    """Count the number of distinct objects in the boolean image."""
    _, contours, _ = cv2.findContours(mask, 1, 2)
    moments = [cv2.moments(cont) for cont in contours]
    big_moments = [m for m in moments if m["m00"] > threshold]
    if canvas != None:
        for moment in big_moments:
            cx = int(moment["m10"] / moment["m00"])
            cy = int(moment["m01"] / moment["m00"])
            cv2.circle(canvas, (cx, cy), 20, (0, 0, 255), -1)
    return len(big_moments)


def detect_green_shape(image=None):
    mask = get_green_mask(image)
    shapes, moments = detect_shape(mask)
    if len(shapes) == 1:
        return shapes[0]

    return Shapes.unknown


def right_most_object_coord(mask, threshold=100):
    """Find the coordinates for the right most object in the mask."""
    _, contours, _ = cv2.findContours(mask, 1, 2)
    moments = [cv2.moments(c) for c in contours if cv2.moments(c)["m00"] > threshold]
    right_most_x = -1
    right_most_coord = (right_most_x, 0)
    for m in moments:
        x, y = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
        if x > right_most_x:
            right_most_x = x
            right_most_coord = (x, y)
    return right_most_coord


def get_hsv_image(image=None, topic="camera/rgb/image_raw"):
    if image is None:
        image = rospy.wait_for_message(topic, Image, timeout=0.4)
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def get_red_mask(image=None, topic="camera/rgb/image_raw"):
    return hsv_bound(
        get_hsv_image(image=image, topic=topic), RED_UPPER, RED_LOWER, 3, 6
    )


def get_red_mask_image_det(image=None, topic="camera/rgb/image_raw"):
    return hsv_bound(
        get_hsv_image(image=image, topic=topic), RED_UPPER_IMG, RED_LOWER_IMG, 3, 6
    )


def get_white_mask(image=None, topic="camera/rgb/image_raw"):
    return hsv_bound(
        get_hsv_image(image=image, topic=topic), WHITE_UPPER, WHITE_LOWER, 3, 6
    )


def get_green_mask(image=None, topic="camera/rgb/image_raw"):
    # return hsv_bound(get_hsv_image(image=image), GREEN_UPPER, GREEN_LOWER, 3, 6)
    hsv = get_hsv_image(image=image)
    mask = cv2.inRange(hsv, np.array(GREEN_LOWER_180), np.array(GREEN_UPPER_180))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return mask


def image_testing_callback(msg):
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    red_mask = get_red_mask(image=msg)
    green_mask = get_green_mask(image=msg)
    comb_mask = red_mask | green_mask

    all_shapes, all_moments = detect_shape(mask=comb_mask, canvas=image, threshold=1000)
    # big_moment = max(all_moments, key=lambda m: m["m00"])
    # print(big_moment["m00"])
    # # print(all_shapes, all_moments)

    # green_shapes, green_moments = detect_shape(
    #     mask=green_mask, canvas=image, threshold=1000
    # )
    # if len(green_shapes):
    #     green_shape_moment = max(green_moments, key=lambda m: m["m00"])
    #     green_shape_index = green_moments.index(green_shape_moment)
    #     green_shape = green_shapes[green_shape_index]
    #     print(green_shape, green_shape_moment["m00"])
    # if len(all_shapes):
    #     # print(all_shapes, all_moments)
    #     pass

    cv2.imshow("red", red_mask)
    cv2.imshow("green", green_mask)
    cv2.imshow("comb", comb_mask)
    cv2.waitKey(1)


if __name__ == "__main__":
    bridge = cv_bridge.CvBridge()
    rospy.init_node("img_proc")
    # image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, image_testing_callback)
    image_sub = rospy.Subscriber("usb_cam/image_raw", Image, image_testing_callback)
    while True:
        study_shapes(
            get_red_mask, min_samples=30, topic="usb_cam/image_raw", approx_factor=0.02
        )
    rospy.spin()
