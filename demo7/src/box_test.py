#!/usr/bin/env python
import actionlib
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist

from ar_states import (
    DriveToStart,
    Survey,
    ApproachParallel,
    PushParallel,
    ApproachPerpendicular,
    PushPerpendicular,
)

POSITIONS = [
    (Point(0, 0, 0.1), Quaternion(0, 0, 0, 1)),
    (Point(0, 0, -0.7), Quaternion(0, 0, 0, 1)),
    (Point(0, -0.4, 0), Quaternion(0, 0, 0, 1)),
    (Point(0, 0.4, 0), Quaternion(0, 0, 0, 1))
]

class BoxApproach(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["push_par"], input_keys=["box_marker"])
        self.rate = rate
        self.pub_node = pub_node
        self.box_marker = None
        self.box_marker_id = None
        self.box_marker_frame = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.counter = 0 

    def execute(self, userdata):
        self.box_marker = TARGET_BOX_MARKER
        self.box_marker_id = TARGET_BOX_MARKER.id
        self.box_marker_frame = "ar_marker_" + str(self.box_marker_id)

        ar_sub = rospy.Subscriber(
            MIDCAM_AR_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        while not self.box_marker:
            rospy.sleep(0.2)

        print("Approach with planner")
        self.client.send_goal(self.calculate_target())
        self.client.wait_for_result()

        self.counter += 1
        ar_sub.unregister()
        return "push_par"

    def calculate_target(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.box_marker_frame
        goal.target_pose.pose.position = POSITIONS[self.counter][0]
        goal.target_pose.pose.orientation = POSITIONS[self.counter][1]
        return goal

    def ar_callback(self, msg):
        for marker in msg.markers:
            if marker.id == self.box_marker_id:
                self.box_marker = marker


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
            "DRIVE_TO_START", DriveToStart(rate, cmd_vel_pub), transitions={"survey": "SURVEY"}
        )

        smach.StateMachine.add(
            "SURVEY", BoxApproach(rate, cmd_vel_pub), transitions={"drive_to_start": "DRIVE_TO_START"}
        )


    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()
