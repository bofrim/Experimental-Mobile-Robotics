#!/usr/bin/env python
# Standard Python
from enum import Enum

# ROS Python
import rospy
import smach
import smach_ros
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import decompose_matrix
from ros_numpy import numpify

# Constants
METER_SCALE = 2
ROBOT_WIDTH_M = 0.35 * METER_SCALE
FINISH_DISTANCE_M = 3 * METER_SCALE
HORIZONTAL_THRESHOLD_M = ROBOT_WIDTH_M * 1.1
ANGULAR_VELOCITY = 0.3
LINEAR_VELOCITY = 0.2
TWIST_PUB_FREQ = 10


# Eww, Globals
g_x = None
g_y = None
g_z = None
g_theta = None
g_lin_vel_mag = None
g_ang_vel_mag = None
g_bumper = None
g_horizontal_preferance = None


def odom_callback(msg):
    """Callback for when odometry data is received."""
    global g_x
    global g_y
    global g_z
    global g_theta
    global g_lin_vel_mag
    global g_ang_vel_mag

    pose = numpify(msg.pose.pose)
    _, _, angles, _, _ = decompose_matrix(pose)

    g_x = msg.pose.pose.position.x
    g_y = msg.pose.pose.position.y
    g_z = msg.pose.pose.position.z
    g_theta = angles[2] * 180 / 3.14159
    lin_vel = msg.twist.twist.linear
    g_lin_vel_mag = (lin_vel.x ** 2 + lin_vel.y ** 2 + lin_vel.z ** 2) ** 0.5
    ang_vel = msg.twist.twist.angular
    g_ang_vel_mag = (ang_vel.x ** 2 + ang_vel.y ** 2 + ang_vel.z ** 2) ** 0.5


def bumper_callback(msg):
    """Callback for when bumper data is received."""
    global g_bumper
    g_bumper = bool(msg.state)


def stall_robot(twist_publisher, error=0.005):
    """Send a few blank messages to the robot to allow it to come to a stop."""
    rate = rospy.Rate(TWIST_PUB_FREQ)
    while g_lin_vel_mag > error and g_ang_vel_mag > error:
        print("Slow down")
        twist_publisher.publish(Twist())
        rate.sleep()

class Direction(Enum):
    North = 0
    East = 90
    South = 180
    West = -90


class Forward(smach.State):
    """In the forward state, the robot will move forward until it hits something.

    If the robot reaches it's goal distance, it succeeds. If it hits something beore
    crossing this finish line, it will backup a little bit before turning.
    """

    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["backup", "finished"])
        self.rate = rospy.Rate(TWIST_PUB_FREQ)
        self.kobuki_movement = movement_pub_node

    def execute(self, userdata):
        global g_bumper
        stall_robot(self.kobuki_movement)
        while not g_bumper:
            twist = Twist()
            twist.linear.x = LINEAR_VELOCITY
            self.kobuki_movement.publish(twist)

            if g_x > FINISH_DISTANCE_M:
                return "finished"

            self.rate.sleep()

        return "backup"


class Backup(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["turn_horizontal"])
        self.rate = rospy.Rate(TWIST_PUB_FREQ)
        self.kobuki_movement = movement_pub_node

    def execute(self, userdata):
        stall_robot(self.kobuki_movement)
        backup_twist = Twist()
        backup_twist.linear.x = -1 * LINEAR_VELOCITY
        num_backup_msgs = 10

        for x in range(num_backup_msgs):
            self.kobuki_movement.publish(backup_twist)
            self.rate.sleep()

        return "turn_horizontal"


class TurnHorizontal(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["move_horizontal"])
        self.kobuki_movement = movement_pub_node

    def execute(self, userdata):
        global g_theta
        global g_horizontal_preferance
        stall_robot(self.kobuki_movement)
        turn_kobuki(g_horizontal_preferance.value, self.kobuki_movement)
        # if -10 < g_theta < 10:
        #     turn_kobuki(90, self.kobuki_movement)
        # elif -80 < g_theta and g_theta < -100:
        #     turn_kobuki(90, self.kobuki_movement)
        # elif 80 < g_theta and g_theta < 100:
        #     turn_kobuki(-90, self.kobuki_movement)

        return "move_horizontal"


class Horizontal(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["turn_forward", "backup"])
        self.rate = rospy.Rate(TWIST_PUB_FREQ)
        self.kobuki_movement = movement_pub_node

    def execute(self, userdata):
        global g_bumper
        global g_y
        global g_horizontal_preferance
        stall_robot(self.kobuki_movement)
        initial_y = g_y
        while abs(g_y - initial_y) < HORIZONTAL_THRESHOLD_M:
            if g_bumper:
                # Ran into something while traversing!
                if g_horizontal_preferance is Direction.East:
                    g_horizontal_preferance = Direction.West
                else:
                    g_horizontal_preferance = Direction.East
                return "backup"

            forward_twist = Twist()
            forward_twist.linear.x = LINEAR_VELOCITY
            self.kobuki_movement.publish(forward_twist)
            self.rate.sleep()

        return "turn_forward"


class TurnForward(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["move_forward"])
        self.kobuki_movement = movement_pub_node

    def execute(self, userdata):
        global g_theta
        stall_robot(self.kobuki_movement)
        turn_kobuki(Direction.North.value, self.kobuki_movement)

        return "move_forward"


class Finished(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["exit"])

    def execute(self, userdata):
        # TODO: Play a nice song / flash light
        return "exit"


def turn_kobuki(desired_angle, kobuki_pub_node, angle_tolerance=2):
    global g_theta
    desired_angle = int(desired_angle)
    rate = rospy.Rate(TWIST_PUB_FREQ)
    lower_bound = desired_angle - angle_tolerance
    upper_bound = desired_angle + angle_tolerance

    while g_theta < lower_bound or g_theta > upper_bound:
        rotation_direction = (desired_angle - g_theta) / abs(desired_angle - g_theta)
        rotation_ramp = max(1.5, abs(desired_angle - g_theta) / 30)

        twist = Twist()
        twist.angular.z = rotation_direction * ANGULAR_VELOCITY * rotation_ramp

        kobuki_pub_node.publish(twist)
        rate.sleep()


def main():
    global g_horizontal_preferance
    g_horizontal_preferance = Direction.East
    rospy.init_node("nav_state_machine")

    state_machine = smach.StateMachine(outcomes=["exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
    bumper_sub = rospy.Subscriber(
        "/mobile_base/events/bumper", BumperEvent, bumper_callback
    )

    with state_machine:
        smach.StateMachine.add(
            "FORWARD",
            Forward(cmd_vel_pub),
            transitions={"backup": "BACKUP", "finished": "FINISHED"},
        )

        smach.StateMachine.add(
            "BACKUP",
            Backup(cmd_vel_pub),
            transitions={"turn_horizontal": "TURN_HORIZONTAL"},
        )

        smach.StateMachine.add(
            "TURN_HORIZONTAL",
            TurnHorizontal(cmd_vel_pub),
            transitions={"move_horizontal": "MOVE_HORIZONTAL"},
        )

        smach.StateMachine.add(
            "MOVE_HORIZONTAL",
            Horizontal(cmd_vel_pub),
            transitions={"backup": "BACKUP", "turn_forward": "TURN_FORWARD"},
        )

        smach.StateMachine.add(
            "TURN_FORWARD",
            TurnForward(cmd_vel_pub),
            transitions={"move_forward": "FORWARD"},
        )

        smach.StateMachine.add("FINISHED", Finished(), transitions={"exit": "exit"})

    state_machine.execute()

    state_introspection_server.stop()


if __name__ == "__main__":
    main()
