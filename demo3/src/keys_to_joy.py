#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

key_mapping = {
    "a": 0,
    "b": 1,
    "x": 2,
    "y": 3,
    "u": 4,
    "i": 5,
    "o": 6,
    "p": 7,
    "j": 8,
    "k": 9,
    "h": 10,
}


def keys_cb(msg, joy_pub):
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return  # unknown-key

    button_index = key_mapping[msg.data[0]]
    joy_message = Joy()
    joy_message.buttons = [0] * len(key_mapping)
    joy_message.buttons[button_index] = 1
    joy_message.axes = [0] * 8

    joy_pub.publish(joy_message)


if __name__ == "__main__":
    rospy.init_node("keys_to_joy")
    joy_pub = rospy.Publisher("joy", Joy, queue_size=1)
    rospy.Subscriber("keys", String, keys_cb, joy_pub)
    rospy.spin()
