#!/usr/bin/env python
import actionlib
import rospy
import smach
import smach_ros

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led, BumperEvent
from nav_msgs.msg import Odometry

from ar_states import DriveToStart

POSITIONS = [
    (Point(0, 0, -0.7), Quaternion(0, 0, 0, 1)),
    (Point(0, 0, 0.3), Quaternion(0, 0, 1, 0)),
    (Point(0, -0.5, -0.25), Quaternion(0, 0, 0.70710678, 0.70710678)),
    (Point(0, 0.5, -0.25), Quaternion(0, 0, -0.70710678, 0.70710678))
]

class BoxApproach(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_start", "exit"])
        self.rate = rate
        self.pub_node = pub_node
        self.box_marker = None
        self.box_marker_id = None
        self.box_marker_frame = None
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.counter = 0 

    def execute(self, userdata):
        self.box_marker_id = None
        self.box_marker = None

        ar_sub = rospy.Subscriber(
            "ar_pose_marker_mid", AlvarMarkers, self.ar_callback, queue_size=1
        )

        while not self.box_marker_id and not rospy.is_shutdown():
            rospy.sleep(0.2)

        self.box_marker_frame = "ar_marker_" + str(self.box_marker_id)
        print "Frame " + self.box_marker_frame

        if self.counter >= len(POSITIONS):
            return "exit"
            
        print("Approach with planner")
        self.client.send_goal(self.calculate_target())
        self.client.wait_for_result()

        rospy.sleep(2.0)

        back_twist = Twist()
        back_twist.linear.x = -0.2
        for _ in range(0, 20):
            self.pub_node.publish(back_twist)

        self.counter += 1
        ar_sub.unregister()
        return "drive_to_start"

    def calculate_target(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.box_marker_frame
        goal.target_pose.pose.position = POSITIONS[self.counter][0]
        goal.target_pose.pose.orientation = POSITIONS[self.counter][1]
        return goal

    def ar_callback(self, msg):
        if not self.box_marker:
            for marker in msg.markers:
                self.box_marker = marker
                self.box_marker_id = marker.id
                return
        return


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
            "SURVEY", BoxApproach(rate, cmd_vel_pub), transitions={"drive_to_start": "DRIVE_TO_START", "exit": "exit"}
        )

        smach.StateMachine.add(
            "DRIVE_TO_START", DriveToStart(rate, cmd_vel_pub), transitions={"survey": "SURVEY"}
        )


    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()
