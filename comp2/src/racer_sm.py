#!/usr/bin/env python
import general_states
import rospy
import smach
import smach_ros

from location1 import TurnLeft1, Detect1
from location2 import TurnLeft2End, DriveToObjects, Detect2, Turn180, TurnLeft2End
from location3 import TurnLeft3, Detect3

from general_states import Driver, DriveToRedLine, LineStop, Advancer, AtLine, TurnRight 
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from time import time


def main():
    rospy.init_node("robber_state_machine")

    state_machine = smach.StateMachine(outcomes=["complete", "exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    light_pubs = []
    light_pubs.append(rospy.Publisher('/mobile_base/commands/led1',Led))
    light_pubs.append(rospy.Publisher('/mobile_base/commands/led2',Led))
    rate = rospy.Rate(10)

    with state_machine:
            smach.StateMachine.add(
                "DRIVE",
                Driver(rate, cmd_vel_pub),
                transitions={"drive_to_red_line": "DRIVE_TO_RED_LINE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "DRIVE_TO_RED_LINE",
                DriveToRedLine(rate, cmd_vel_pub),
                transitions={"stop": "STOP", "exit": "exit"}
            )

            smach.StateMachine.add(
                "STOP",
                LineStop(rate, cmd_vel_pub, light_pubs),
                transitions={"advance": "ADVANCE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "ADVANCE",
                Advancer(rate, cmd_vel_pub),
                transitions={"at_line":"AT_LINE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "AT_LINE",
                AtLine(rate),
                transitions={"drive": "DRIVE", 
                             "turn_left_1": "TURN_LEFT_1", 
                             "turn_left_2_start": "TURN_LEFT_2_START",
                             "turn_left_2_end": "TURN_LEFT_2_END", 
                             "turn_left_3":"TURN_LEFT_3",
                             "exit":"exit"}
            )

            smach.StateMachine.add(
                "TURN_LEFT_1",
                TurnLeft1(rate, cmd_vel_pub),
                transitions={"detect_1":"DETECT_1", "exit": "exit"}
            )

            smach.StateMachine.add(
                "DETECT_1",
                Detect1(rate),
                transitions={"turn_right": "TURN_RIGHT", "exit": "exit"} 
            )

            smach.StateMachine.add(
                "TURN_RIGHT",
                TurnRight(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"}
            )

            smach.StateMachine.add(
                "TURN_LEFT_2_START",
                TurnLeft2Start(rate, cmd_vel_pub),
                transitions={"drive_to_objects":"DRIVE_TO_OBJECTS", "exit": "exit"}
            )

            smach.StateMachine.add(
                "DRIVE_TO_OBJECTS",
                DriveToObjects(rate, cmd_vel_pub),
                transitions={"detect_2": "DETECT_2", "exit": "exit"}
            )

            smach.StateMachine.add(
                "DETECT_2",
                Detect2(rate),
                transitions={"turn_180": "TURN_180", "exit": "exit"} 
            )

            smach.StateMachine.add(
                "TURN_180",
                Turn180(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"}
            )

            smach.StateMachine.add(
                "TURN_LEFT_2_END",
                TurnLeft2End(rate, cmd_vel_pub),
                transitions={"drive":"DRIVE", "exit": "exit"}
            )

            smach.StateMachine.add(
                "TURN_LEFT_3",
                TurnLeft3(rate, cmd_vel_pub),
                transitions={"detect_3":"DETECT_3", "exit": "exit"}
            )

            smach.StateMachine.add(
                "DETECT_3",
                Detect3(rate),
                transitions={"turn_right": "TURN_RIGHT", "exit": "exit"} 
            )

    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()