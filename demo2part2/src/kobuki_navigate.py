#!/usr/bin/env python
# Standard Python
from enum import Enum
from time import time

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
from kobuki_msgs.msg import Sound

# Constants
METER_SCALE = 1
ROBOT_WIDTH_M = 0.35 * METER_SCALE
FINISH_DISTANCE_M = 3 * METER_SCALE + ROBOT_WIDTH_M
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


def stall_robot(twist_publisher, error=0.005, hold_time_s=0.3):
    """Send a few blank messages to the robot to allow it to come to a stop."""
    rate = rospy.Rate(TWIST_PUB_FREQ)
    while g_lin_vel_mag > error and g_ang_vel_mag > error:
        twist_publisher.publish(Twist())
        rate.sleep()
    start_time = time()
    while time() - start_time < hold_time_s:
        twist_publisher.publish(Twist())
        rate.sleep()


def angle_ramp(desired_angle, current_angle, scale=1, ramp_denominator=90):
    rotation_direction = 1
    rotation_ramp = 0
    if desired_angle != current_angle:
        rotation_direction = (desired_angle - current_angle) / abs(
            desired_angle - current_angle
        )
        rotation_ramp = max(2, abs(desired_angle - current_angle) / ramp_denominator)

    return rotation_direction * rotation_ramp * ANGULAR_VELOCITY * scale


def proportional_twist(target_angle, linear_velocity=LINEAR_VELOCITY):
    """Calculate a tiwst message to target a desired heading."""
    global g_theta
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angle_ramp(target_angle, g_theta, scale=0.2)
    return twist


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
            twist = proportional_twist(Direction.North.value)
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

            twist = proportional_twist(g_horizontal_preferance.value)
            self.kobuki_movement.publish(twist)
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
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=["exit"])
        self.sound_node = rospy.Publisher("commands/sound", Sound, queue_size=1)
        self.rate = rospy.Rate(0.5)
        self.kobuki_movement = movement_pub_node

    def execute(self, userdata):
        sound = Sound()
        sound.value = 3
        turn_kobuki(Direction.South.value, self.kobuki_movement)

        # sound = Sound()
        # sound.value = 3

        # for _ in range(0, 4):
        #    self.sound_node.publish(sound)
        #    self.rate.sleep()

        return "exit"


def turn_kobuki(desired_angle, kobuki_pub_node, angle_tolerance=2, hold_time_s=1):
    global g_theta
    desired_angle = int(desired_angle)
    rate = rospy.Rate(TWIST_PUB_FREQ)
    lower_bound = desired_angle - angle_tolerance
    upper_bound = desired_angle + angle_tolerance

    def send_twist(scale=1):
        twist = Twist()
        twist.angular.z = angle_ramp(desired_angle, g_theta, scale=scale)
        kobuki_pub_node.publish(twist)

    while g_theta < lower_bound or g_theta > upper_bound:
        send_twist()
        rate.sleep()

    stable_time = time()
    while time() - stable_time < hold_time_s:
        send_twist(scale=0.1)
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

        smach.StateMachine.add(
            "FINISHED", Finished(cmd_vel_pub), transitions={"exit": "exit"}
        )

    state_machine.execute()

    state_introspection_server.stop()


if __name__ == "__main__":
    main()
