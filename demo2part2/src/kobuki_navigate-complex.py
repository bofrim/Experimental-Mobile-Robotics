#!/usr/bin/env python
import rospy
import smach
import smach_ros
import threading

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent


class Forward(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["backup", "finished"])
        self.mutex = threading.Lock()

        self.bumper_hit = False
        self.bumper_node = rospy.Subscriber(
            "/mobile_base/events/bumper", BumperEvent, self.bumper_callback
        )
        self.kobuki_movement = movement_pub_node

    def bumper_callback(self, msg):
        self.mutex.acquire()
        if msg.bumper == 1 and msg.state == 1:
            self.bumper_hit = True
        self.mutex.release()

    def __execute__(self):
        while True:
            self.mutex.acquire()
            if self.bumper_hit:
                # TODO: Implement different turning options
                return "turn_clockwise"

            self.mutex.release()

            twist = Twist()
            twist.linear.x = 1

            # Use encoders to determine how far we've traveled

            self.kobuki_movement.publish(twist)


class TurnToHorizontal(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["forward", "left"])
        self.kobuki_movement = movement_pub_node

    def __execute__(self):

        # Change this to use encoders to know when to break
        while True:
            twist = Twist()
            twist.angular.z = 0.1

            self.kobuki_movement.Publish(twist)
            break

        # Somehow determine which state we are moving to
        #  OR - have a state for turning straight -> left
        #     - and have a state for turning right -> straight

        # TODO: Change this
        return "forward"


class TurnToVertical(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["forward", "right"])
        self.kobuki_movement = movement_pub_node

    def __execute__(self):

        # Change this to use encoders to know when to break
        while True:
            twist = Twist()
            twist.angular.z = -0.1

            self.kobuki_movement.Publish(twist)
            break

        # Somehow determine which state we are moving to
        #  OR - have a state for turning straight -> left
        #     - and have a state for turning right -> straight

        # TODO: Change this
        return "forward"


def main():
    rospy.init_node("nav_state_machine")

    state_machine = smach.StateMachine(outcomes=["exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    with state_machine:
        smach.StateMachine.add(
            "FORWARD",
            Forward(cmd_vel_pub),
            transitions={"backup": "BACKUP", "finished": "FINISHED"},
        )

    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()
