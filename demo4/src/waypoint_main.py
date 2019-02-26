#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist

from waypoint_states import Drive, Stop, InitWaypoints


def main():
    rospy.init_node("waypoint_navigation")

    state_machine = smach.StateMachine(outcomes=["complete", "exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)

    with state_machine:
        smach.StateMachine.add (
            "INIT",
            InitWaypoints(rate),
            transitions={"drive": "DRIVE", "exit": "exit"}
        )

        smach.StateMachine.add(
            "DRIVE",
            Drive(rate, cmd_vel_pub),
            transitions={"stop": "STOP", "exit": "exit"},
        )

        smach.StateMachine.add(
            "STOP",
            Stop(rate, cmd_vel_pub),
            transitions={"drive": "DRIVE"},
        )

    state_machine.execute()
    state_introspection_server.stop()

if __name__ == '__main__':
    main()