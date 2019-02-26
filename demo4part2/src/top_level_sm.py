#!/usr/bin/env python
import rospy
import smach


def main():
    rospy.init_node("toplevel_sm")
    state_machine = smach.StateMachine(outcomes=["exit"])

    with state_machine:
        """
        smach.StateMachine.add(
            "",
            ,
            transitions={"": "", "exit": "exit"},
        )
        """
        state_machine.execute()


if __name__ == "__main__":
    main()
