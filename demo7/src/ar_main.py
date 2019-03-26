#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist

from ar_states import (
    FindTargetLogitech,
    FindTargetAuto,
    DriveToStart,
    Survey,
    ApproachParallel,
    PushParallel,
    ApproachPerpendicular,
    PushPerpendicular,
)


def main():
    rospy.init_node("box_push")

    state_machine = smach.StateMachine(outcomes=["complete", "exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)

    with state_machine:
        smach.StateMachine.add(
            "LOCATE", FindTargetLogitech(rate, cmd_vel_pub), transitions={"drive_to_start", "DRIVE_TO_START"}
        )

        smach.StateMachine.add(
            "DRIVE_TO_START", DriveToStart(rate, cmd_vel_pub), transitions={"survey": "SURVEY"}
        )

        smach.StateMachine.add(
            "SURVEY", Survey(rate, cmd_vel_pub), transitions={"approach_par": "APPROACH_PAR"}
        )

        smach.StateMachine.add(
            "APPROACH_PAR", ApproachParallel(rate, cmd_vel_pub), transitions={"push_par": "PUSH_PAR"}
        )

        smach.StateMachine.add(
            "PUSH_PAR", PushParallel(rate, cmd_vel_pub), transitions={"approach_perp": "APPROACH_PERP"}
        )

        smach.StateMachine.add(
            "APPROACH_PERP", ApproachPerpendicular(rate, cmd_vel_pub), transitions={"push_perp": "PUSH_PERP"}
        )

        smach.StateMachine.add(
            "PUSH_PERP", PushPerpendicular(rate, cmd_vel_pub), transitions={"complete": "complete"}
        )


    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()
