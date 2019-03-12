#!/usr/bin/env python
import cv2
from collections import defaultdict
import numpy as np
import rospy
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import math
from enum import Enum


class Shapes(Enum):
    unknown = -1
    triangle = 3
    square = 4
    pentagon = 5
    circle = 9


H_MAX = 20
H_MIN = 317
S_MAX = 255
S_MIN = 180
V_MAX = 255
V_MIN = 80

RED_UPPER = [20, 255, 255]
RED_LOWER = [317, 80, 80]
GREEN_UPPER = [176, 255, 255]
GREEN_LOWER = [100, 84, 58]
GREEN_UPPER_180 = [75, 255, 255]
GREEN_LOWER_180 = [45, 84, 58]
WHITE_UPPER = [255, 10, 255]
WHITE_LOWER = [0, 0, 185]


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


def crop(image, upper_frac, window_frac):
    """Crop an image to the fraction bounds.

    Fractions are measured from the top of the screen.
    """
    # Undetermined shape size
    shape = image.shape
    h = shape[0]
    w = shape[1]
    #print("h, w", h, w)
    top = h * upper_frac
    bottom = h * (upper_frac + window_frac)
    #print("top, bottom", top, bottom)
    if top > bottom:
        print("WARNING: Cropping everything! Top, bottom:", top, bottom)
    image[0:top, 0:w] = 0
    image[bottom:h, 0:w] = 0
    return image


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


def detect_shape(mask, canvas=None, threshold=100):
    """Detect a shape contained in an image.
    
    Adappted from: https://stackoverflow.com/questions/11424002/how-to-detect-simple-geometric-shapes-using-opencv
    """
    detected_shapes = []
    moments = []
    _, contours, _ = cv2.findContours(mask, 1, 2)
    for cnt in contours:
        if cv2.moments(cnt)["m00"] > threshold:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
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
            elif len(approx) > 9:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 255, 255), -1)
                detected_shapes.append(Shapes.circle)
                moments.append(cv2.moments(cnt))
            else:
                detected_shapes.append(Shapes.unknown)
                moments.append(cv2.moments(cnt))
    return detected_shapes, moments


def detect_green_shape(image=None):
    mask = get_green_mask(image)
    shapes, moments = detect_shape(mask)
    if len(shapes) == 1:
        return shapes[0]

    return Shapes.unknown


# def path_angle_center(mask, upper_crop=0.5, lower_crop=0.95):
#     """Detect the angle of a path.

#     Crop fractions are measured from the top of the screen.
#     """
#     mask = crop(mask, upper_crop, lower_crop)
#     lines = extract_lines(mask)
#     if lines is not None:
#         angles = [
#             np.rad2deg(line[0][1])
#             if np.rad2deg(line[0][1]) < 90
#             else np.rad2deg(line[0][1]) - 180
#             for line in lines
#         ]
#         return np.average(angles)

#     return None


def path_mass_center(mask, top_frac=0.7, window_frac=0.3, mass_threshold=1000):
    """Detect the centroid of a path.
    
    Crop fractions are measured from the top of the screen.
    Return the +/- distance between the centroid and the middle of the screen.
    Return None if there is no centroid.
    """
    h, w = mask.shape
    #print("h, w", h, w)
    search_top = h * top_frac
    search_bot = h * (top_frac + window_frac)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    cv2.imshow("windowasdf_crop", mask)
    cv2.waitKey(3)
    M = cv2.moments(mask)
    if M["m00"] > mass_threshold:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(mask, (cx, cy), 20, (0, 0, 255), -1)
        return cx - w / 2

    return None


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


def bin_filter(image):
    """"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    adapt_type = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
    thresh_type = cv2.THRESH_BINARY_INV
    out = cv2.adaptiveThreshold(blur, 255, adapt_type, thresh_type, 11, 2)
    kernel = np.ones((4, 4), np.uint8)
    out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel)
    cv2.imshow("out", out)
    cv2.waitKey(3)
    return out


def extract_lines(mask, rho=2, theta=np.pi / 180, thresh=400):
    return cv2.HoughLines(mask, rho, theta, thresh)


def get_red_mask(image=None):
    """"""
    if image is None:
        image = rospy.wait_for_message("camera/rgb/image_raw", Image)
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return hsv_bound(hsv, RED_UPPER, RED_LOWER, 3, 6)


def get_white_mask(image=None):
    """"""
    if image is None:
        image = rospy.wait_for_message("camera/rgb/image_raw", Image)
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return hsv_bound(hsv, WHITE_UPPER, WHITE_LOWER, 3, 6)


def get_green_mask(image=None):
    """"""
    if image is None:
        image = rospy.wait_for_message("camera/rgb/image_raw", Image)
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(GREEN_LOWER_180), np.array(GREEN_UPPER_180))
    return mask


def draw_lines(lines, canvas, rgb=(0, 0, 255)):
    if lines is None:
        lines = []
    #print("Len lines=", len(lines))
    for line in lines:
        rho = line[0][0]
        theta = line[0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
        pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
        cv2.line(canvas, pt1, pt2, rgb, 3, cv2.LINE_AA)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(
            canvas,
            str(int(theta * 180.0 / 3.1415)),
            (50, 150),
            font,
            1,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
    cv2.imshow("canvas", canvas)


def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # # Lines
    # white_mask = hsv_bound(hsv, WHITE_UPPER, WHITE_LOWER)
    # kernel = np.ones((2, 2), np.uint8)
    # white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    # path_mask = bin_filter(cv2.bitwise_and(image, image, mask=white_mask))
    # cv2.imshow("masked", cv2.bitwise_and(image, image, mask=white_mask))
    # cv2.waitKey(3)
    # # hsv_white = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # # mask = hsv_bound(hsv_white, WHITE_LOWER, WHITE_UPPER)
    # cv2.imshow("bin", path_mask)
    # cv2.waitKey(3)
    # white_lines = extract_lines(path_mask)
    # red_color = (0, 0, 255)
    # draw_lines(white_lines, image, red_color)
    # avg_ang = path_angle_center(white_mask)
    # cv2.putText(
    #     image,
    #     str(avg_ang),
    #     (50, 50),
    #     cv2.FONT_HERSHEY_SIMPLEX,
    #     1,
    #     (0, 0, 0),
    #     2,
    #     cv2.LINE_AA,
    # )
    # cv2.imshow("angle", image)
    # cv2.waitKey(3)
    # # Shapes
    red_mask = hsv_bound(hsv, RED_UPPER, RED_LOWER, denoise=8)
    shapes, moments = detect_shape(red_mask, image)
    #print(detect_shape(red_mask, image))
    #print([(int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) for M in moments])
    cv2.imshow("mask", red_mask)
    cv2.imshow("image", image)
    cv2.waitKey(3)
    # Lowest Shapes
    red_mask = hsv_bound(
        cv2.cvtColor(image, cv2.COLOR_BGR2HSV), RED_UPPER, RED_LOWER, denoise=3, fill=6
    )
    lowest_object_coord(red_mask)


if __name__ == "__main__":
    bridge = cv_bridge.CvBridge()
    rospy.init_node("img_proc")
    image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, image_callback)
    while not rospy.is_shutdown():
        rospy.spin()
