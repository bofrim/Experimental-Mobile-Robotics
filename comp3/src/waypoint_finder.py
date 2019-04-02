#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from time import sleep
import datetime
import os

TEMP_FILE_NAME = "temp_file"
FILE_NAME_PREFIX = "waypoints"
FILE_NAME_SUFFIX = ".txt"


def write_position_to_file():
    with open(TEMP_FILE_NAME, "a") as waypoint_file:
        try:
            trans, rot = g_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
            print(str(trans) + ";" + str(rot) + "\n")
            waypoint_file.write(
                str(trans)[1:-1].replace(" ", "")
                + ";"
                + str(rot)[1:-1].replace(" ", "")
                + "\n"
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            print("tf error: ", e)
    sleep(1)


def joy_callback(msg):
    # Buttons based on Controller "D" Mode
    if msg.buttons[0]:  # X
        write_position_to_file()
    elif msg.buttons[2]:  # B
        rospy.signal_shutdown("User quit")


if __name__ == "__main__":
    rospy.init_node("WaypointFinder")
    g_listener = tf.TransformListener()
    joy_sub = rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)
    rospy.spin()
    filename = (
        FILE_NAME_PREFIX
        + str(datetime.datetime.now()).replace(" ", "_")
        + FILE_NAME_SUFFIX
    )
    print("Writing to " + filename)
    os.rename(TEMP_FILE_NAME, filename)
