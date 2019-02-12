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

        zone1_sm = smach.StateMachine(outcomes=["zone2", "exit"])
        with zone1_sm:
            smach.StateMachine.add(
                "ADVANCE",
                LineAdvancer(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "DRIVE",
                Driver(rate, cmd_vel_pub),
                transitions={"location": "LOCATION1", "stop": "STOP", "exit": "exit"},
            )

            smach.StateMachine.add(
                "LOCATION1",
                Location1(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "STOP",
                LineStop(rate, cmd_vel_pub),
                transitions={"zone_complete": "zone2", "exit": "exit"},
            )
        
        smach.StateMachine.add(
            "ZONE1",
            zone1_sm,
            transitions={"zone2": "ZONE2", "exit": "exit"},
        )

        zone2_sm = smach.StateMachine(outcomes=["zone3", "exit"])
        with zone2_sm:
            smach.StateMachine.add(
                "ADVANCE",
                LineAdvancer(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "DRIVE",
                Driver(rate, cmd_vel_pub),
                transitions={"location": "LOCATION2", "stop": "STOP", "exit": "exit"},
            )

            smach.StateMachine.add(
                "LOCATION2",
                Location2(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"}
            )

            smach.StateMachine.add(
                "STOP",
                LineStop(rate, cmd_vel_pub),
                transitions={"zone_complete": "zone3", "exit": "exit"}
            )

        smach.StateMachine.add(
            "ZONE2",
            zone2_sm,
            transitions={"zone3": "ZONE3", "exit": "exit"},
        )

        zone3_sm = smach.StateMachine(outcomes=["zone4", "exit"])
        with zone3_sm:
            smach.StateMachine.add(
                "ADVANCE",
                LineAdvancer(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "DRIVE",
                Driver(rate, cmd_vel_pub),
                transitions={"location": "exit", "stop": "STOP", "exit": "exit"},
            )

            smach.StateMachine.add(
                "STOP",
                LineStop(rate, cmd_vel_pub),
                transitions={"zone_complete": "zone4", "exit": "exit"},
            )

        smach.StateMachine.add(
            "ZONE3",
            zone3_sm,
            transitions={"zone4": "ZONE4", "exit": "exit"},
        )

        zone4_sm = smach.StateMachine(outcomes=["course_complete", "exit"])
        with zone4_sm:
            smach.StateMachine.add(
                "ADVANCE",
                LineAdvancer(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"},
            )

            smach.StateMachine.add(
                "DRIVE",
                Driver(rate, cmd_vel_pub),
                transitions={"location": "LOCATION3", "stop": "STOP", "exit": "exit"},
            )

            smach.StateMachine.add(
                "LOCATION3",
                Location3(rate, cmd_vel_pub),
                transitions={"drive": "DRIVE", "exit": "exit"}
            )

            smach.StateMachine.add(
                "STOP",
                LineStop(rate, cmd_vel_pub),
                transitions={"zone_complete": "course_complete", "exit": "exit"},
            )

        smach.StateMachine.add(
            "ZONE4",
            zone4_sm,
            transitions={"course_complete": "complete", "exit": "exit"},
        )

    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()