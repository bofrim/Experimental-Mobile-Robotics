#!/usr/bin/env python
import general_states
import rospy
import smach
import smach_ros

from location1 import Location1
from location2 import Location2
from location3 import Location3

from general_states import Driver, LineAdvancer, LineStop
from geometry_msgs.msg import Twist
from time import time


def main():

    rospy.init_node("robber_state_machine")

    state_machine = smach.StateMachine(outcomes=["complete", "exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)

    with state_machine:
            smach.StateMachine.add(
                "DRIVE",
                Driver(rate, cmd_vel_pub),
                transitions={"stop": "STOP", "exit": "exit"},
            )

            smach.StateMachine.add(
                "STOP",
                LineStop(rate, cmd_vel_pub),
                transitions={"advance": "ADVANCE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "ADVANCE",
                LineAdvancer(rate, cmd_vel_pub),
                transitions={"redline":"REDLINE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "REDLINE",
                RedLine(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "location1": "DRIVE", 
                             "location2": "DRIVE", "location3":"DRIVE", 
                             "exit":"exit"},
            )

    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()