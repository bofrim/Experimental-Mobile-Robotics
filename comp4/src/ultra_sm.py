#!/usr/bin/env python
import general_states
import rospy
import smach
import smach_ros

from location1 import Detect1
from location2 import DriveToObjects, DriveFromObjects, Detect2
from location3 import Detect3
from location4 import DriverRamp, BoxSurvey, TagScan1, TagScan2, Push, OnRamp, ShapeScan

from general_states import Driver, Advancer, AtLine, Turn
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led, Sound
from time import time


def main():
    rospy.init_node("ultra_state_machine")
    if rospy.has_param("~initial_line"):
        initial_line = rospy.get_param("~initial_line")
    else:
        initial_line = 0

    sound_pub = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=1)

    state_machine = smach.StateMachine(outcomes=["complete", "exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    light_pubs = []
    light_pubs.append(rospy.Publisher("/mobile_base/commands/led1", Led, queue_size=1))
    light_pubs.append(rospy.Publisher("/mobile_base/commands/led2", Led, queue_size=1))

    rate = rospy.Rate(10)

    with state_machine:
        smach.StateMachine.add(
            "DRIVE",
            Driver(rate, cmd_vel_pub),
            transitions={"advance": "ADVANCE", "exit": "exit"},
        )

        smach.StateMachine.add(
            "ADVANCE",
            Advancer(rate, cmd_vel_pub),
            transitions={"at_line": "AT_LINE", "exit": "exit"},
        )

        smach.StateMachine.add(
            "AT_LINE",
            AtLine(rate, light_pubs=light_pubs, initial_line=initial_line),
            transitions={
                "drive": "DRIVE",
                "turn_left_1": "TURN_LEFT_1",
                "turn_left_2_start": "TURN_LEFT_2_START",
                "turn_left_2_end": "TURN_LEFT_2_END",
                "off_ramp": "OFF_RAMP",
                "turn_left_3_1": "TURN_LEFT_3_1",
                "turn_left_3_2": "TURN_LEFT_3_2",
                "turn_left_3_3": "TURN_LEFT_3_3",
                "exit": "exit",
            },
        )

        smach.StateMachine.add(
            "TURN_LEFT_1",
            Turn(cmd_vel_pub, 63, "detect1"),
            transitions={"detect1": "DETECT1"},
        )

        smach.StateMachine.add(
            "DETECT1",
            Detect1(rate, sound_pub, light_pubs),
            transitions={"turn_right": "TURN_RIGHT", "exit": "exit"},
        )

        smach.StateMachine.add(
            "TURN_RIGHT",
            Turn(cmd_vel_pub, -84, "drive"),
            transitions={"drive": "DRIVE"},
        )

        smach.StateMachine.add(
            "TURN_LEFT_2_START",
            Turn(cmd_vel_pub, 64, "drive_to_objects"),
            transitions={"drive_to_objects": "DRIVE_TO_OBJECTS"},
        )

        smach.StateMachine.add(
            "DRIVE_TO_OBJECTS",
            DriveToObjects(rate, cmd_vel_pub),
            transitions={"detect2": "DETECT2", "exit": "exit"},
        )

        smach.StateMachine.add(
            "DETECT2",
            Detect2(rate, sound_pub, light_pubs),
            transitions={"turn_180": "TURN_180", "exit": "exit"},
        )

        smach.StateMachine.add(
            "TURN_180",
            Turn(cmd_vel_pub, 180, "drive_from_objects"),
            transitions={"drive_from_objects": "DRIVE_FROM_OBJECTS"},
        )

        smach.StateMachine.add(
            "DRIVE_FROM_OBJECTS",
            DriveFromObjects(rate, cmd_vel_pub),
            transitions={"advance": "ADVANCE", "exit": "exit"},
        )

        smach.StateMachine.add(
            "TURN_LEFT_2_END",
            Turn(cmd_vel_pub, 43, "drive"),
            transitions={"drive": "DRIVE"},
        )

        smach.StateMachine.add(
            "OFF_RAMP",
            DriverRamp(rate, cmd_vel_pub),
            transitions={"start": "BOX_SURVEY", "exit": "exit"},
        )

        smach.StateMachine.add(
            "BOX_SURVEY",
            BoxSurvey(rate, cmd_vel_pub),
            transitions={"tag_scan_1": "TAG_SCAN_1", "exit": "exit"},
        )

        smach.StateMachine.add(
            "TAG_SCAN_1",
            TagScan1(rate, cmd_vel_pub),
            transitions={"tag_scan_2": "TAG_SCAN_2", "push": "PUSH", "exit": "exit"},
        )

        smach.StateMachine.add(
            "TAG_SCAN_2",
            TagScan2(rate, cmd_vel_pub),
            transitions={"tag_scan_1": "TAG_SCAN_1", "push": "PUSH", "exit": "exit"},
        )

        smach.StateMachine.add(
            "PUSH",
            Push(rate, cmd_vel_pub),
            transitions={"shape_scan": "SHAPE_SCAN", "exit": "exit"},
        )

        smach.StateMachine.add(
            "SHAPE_SCAN",
            ShapeScan(rate, cmd_vel_pub, light_pubs),
            transitions={"on_ramp": "ON_RAMP", "exit": "exit"},
        )

        smach.StateMachine.add("ON_RAMP", OnRamp(rate), transitions={"drive": "DRIVE"})

        smach.StateMachine.add(
            "TURN_LEFT_3_1",
            Turn(cmd_vel_pub, 69, "detect3"),
            transitions={"detect3": "DETECT3"},
        )

        smach.StateMachine.add(
            "TURN_LEFT_3_2",
            Turn(cmd_vel_pub, 92, "detect3"),
            transitions={"detect3": "DETECT3"},
        )

        smach.StateMachine.add(
            "TURN_LEFT_3_3",
            Turn(cmd_vel_pub, 92, "detect3"),
            transitions={"detect3": "DETECT3"},
        )

        smach.StateMachine.add(
            "DETECT3",
            Detect3(rate, cmd_vel_pub, sound_pub, light_pubs),
            transitions={
                "turn_right_3_1": "TURN_RIGHT_3_1",
                "turn_right_3_2": "TURN_RIGHT_3_2",
                "turn_right_3_3": "TURN_RIGHT_3_3",
                "exit": "exit",
            },
        )

        smach.StateMachine.add(
            "TURN_RIGHT_3_1",
            Turn(cmd_vel_pub, -69, "drive"),
            transitions={"drive": "DRIVE"},
        )

        smach.StateMachine.add(
            "TURN_RIGHT_3_2",
            Turn(cmd_vel_pub, -92, "drive"),
            transitions={"drive": "DRIVE"},
        )

        smach.StateMachine.add(
            "TURN_RIGHT_3_3",
            Turn(cmd_vel_pub, -92, "drive"),
            transitions={"drive": "DRIVE"},
        )

    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()
