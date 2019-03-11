#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist

from ar_states import Approach


def main():
    rospy.init_node("ar_approach_test")

    state_machine = smach.StateMachine(outcomes=["stop"])

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)

    with state_machine:
        smach.StateMachine.add(
            "APPROACH",
            Approach(rate, cmd_vel_pub),
            transitions={"stop": "stop"},
        )

    state_machine.execute()
    state_introspection_server.stop()

if __name__ == '__main__':
    main()