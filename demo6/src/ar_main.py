#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist

from ar_states import Survey, Approach, Stop, Push

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
            "SURVEY",
            Survey(rate, cmd_vel_pub),
            transitions={"approach": "APPROACH"}
        )

        smach.StateMachine.add(
            "APPROACH",
            Approach(rate, cmd_vel_pub),
            transitions={"stop": "STOP"},
        )

        smach.StateMachine.add(
            "STOP",
            Stop(rate, cmd_vel_pub),
            transitions={"push": "PUSH"}
        )

        smach.StateMachine.add(
            "PUSH",
            Push(rate, cmd_vel_pub),
            transitions={"exit": "exit"}
        )

    state_machine.execute()
    state_introspection_server.stop()

if __name__ == '__main__':
    main()
