#!/usr/bin/env python
import cv2
from collections import defaultdict
import numpy as np
import rospy
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import math

H_MAX = 20
H_MIN = 317
S_MAX = 255
S_MIN = 180
V_MAX = 255
V_MIN = 80

RED_UPPER = [20, 255, 255]
RED_LOWER = [317, 180, 80]
GREEN_UPPER = []
GREEN_LOWER = []
WHITE_UPPER = [255, 10, 255]
WHITE_LOWER = [0, 0, 170]


def threshold_hsv_360(
    hsv, h_max=H_MAX, h_min=H_MIN, s_max=S_MAX, s_min=S_MIN, v_max=V_MAX, v_min=V_MIN
):
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
    return mask


def hsv_bound(image, upper_bound, lower_bound):
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
    )


def count_objects(mask):
    """Count the number of distinct objects in the boolean image."""
    contours, _ = cv2.findContours(mask, 1, 2)
    return len(contours)


def detect_shape(mask, canvas):
    """Detect a shape contained in an image.
    
    Adappted from: https://stackoverflow.com/questions/11424002/how-to-detect-simple-geometric-shapes-using-opencv
    """
    _, contours, _ = cv2.findContours(mask, 1, 2)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        print len(approx)
        if len(approx)==5:
            print("pentagon")
            cv2.drawContours(canvas,[cnt],0,255,-1)
        elif len(approx)==3:
            print("triangle")
            cv2.drawContours(canvas,[cnt],0,(0,255,0),-1)
        elif len(approx)==4:
            print("square")
            cv2.drawContours(canvas,[cnt],0,(0,0,255),-1)
        elif len(approx) == 9:
            print("half-circle")
            cv2.drawContours(canvas,[cnt],0,(255,255,0),-1)
        elif len(approx) > 10:
            print("circle")
            cv2.drawContours(canvas,[cnt],0,(0,255,255),-1)
        else:
            print("Maybe circle")


def line_end(mask):
    """Detect if a line is ending."""


def cross_line(path_mask, line_mask):
    """Detect a line crossing the path."""
    combined_mask = path_mask | line_mask
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges,1,np.pi/180,200)
    for rho,theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)


def t_intersection(mask):
    """Detect if there is a T itersection."""


def partial_line_cross(path_mask, line_mask):
    """Detect if a line fully crosses a path."""

def aligned_with_path(path_mask):
    """Check if we are alined with a path."""

def aligned_path_angle(Path_mask):
    """Find the angle of a path that is most aligned with the robot."""

def bin_filter(image):
    """"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    adapt_type = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
    thresh_type = cv2.THRESH_BINARY_INV
    out = cv2.adaptiveThreshold(blur, 255, adapt_type, thresh_type, 11, 2)
    kernel = np.ones((4,4),np.uint8)
    out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel)
    cv2.imshow("out", out)
    cv2.waitKey(3)
    return out


def segment_by_angle_kmeans(lines, k=2, **kwargs):
    """Groups lines based on angle with k-means.

    Uses k-means on the coordinates of the angle on the unit circle 
    to segment `k` angles inside `lines`.

    Adapted from https://stackoverflow.com/questions/46565975/find-intersection-point-of-two-lines-drawn-using-houghlines-opencv?rq=1
    """

    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
    flags = kwargs.get('flags', cv2.KMEANS_RANDOM_CENTERS)
    attempts = kwargs.get('attempts', 10)

    # returns angles in [0, pi] in radians
    angles = np.array([line[0][1] for line in lines])
    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
                    for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)  # transpose to row vec

    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented

def draw_lines(lines, canvas, rgb=(0, 0, 255)):
    if lines is None:
        lines = []
    print("Len lines=", len(lines))
    for line in lines:
            rho = line[0][0]
            theta = line[0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(canvas, pt1, pt2, rgb, 3, cv2.LINE_AA)
    cv2.imshow("canvas", canvas)

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Lines
    white_mask = hsv_bound(hsv, WHITE_UPPER, WHITE_LOWER)
    kernel = np.ones((2,2),np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    path_mask = bin_filter(cv2.bitwise_and(image,image, mask=white_mask))
    cv2.imshow("masked", cv2.bitwise_and(image,image, mask=white_mask))
    cv2.waitKey(3)
    # hsv_white = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # mask = hsv_bound(hsv_white, WHITE_LOWER, WHITE_UPPER)
    rho, theta, thresh = 2, np.pi/180, 400
    cv2.imshow("bin", path_mask)
    cv2.waitKey(3)
    white_lines = cv2.HoughLines(path_mask, rho, theta, thresh)
    # segmented_lines = segment_by_angle_kmeans(lines)
    red_color = (0, 0, 255)
    draw_lines(white_lines, image, red_color)
    # Shapes
    red_mask = hsv_bound(hsv, RED_LOWER, RED_LOWER)
    kernel = np.ones((10,10),np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    detect_shape(red_mask, image)
    cv2.imshow("shapes", red_mask)
    cv2.imshow("lines", image)
    cv2.waitKey(3)

if __name__ == "__main__":
    bridge = cv_bridge.CvBridge()
    rospy.init_node("img_proc")
    image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, image_callback)
    while not rospy.is_shutdown():
        rospy.spin()
