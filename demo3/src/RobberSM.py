#!/usr/bin/env python
import rospy
import smach
import smach_ros
from time import time

import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from demo3.msg import ZoneScan

DISTANCE_LIMIT = 3.0
OUTER_RIGHT = 0
RIGHT = 1
INNER_RIGHT = 2
INNER_LEFT = 3
LEFT = 4
OUTER_LEFT = 5

INNER_DISTANCE_LIMIT = 1.5
NORMAL_DISTANCE_LIMIT = 1.3
OUTER_DISTANCE_LIMIT = 1.0

global g_zone_distances
global g_bumped

def zone_callback(msg):
    global g_zone_distances
    g_zone_distances = msg.distances


def bumper_callback(msg):
    global g_bumped
    g_bumped = bool(msg.state)


def is_right_clear(zones):
    return (zones[OUTER_RIGHT] >= OUTER_DISTANCE_LIMIT and
            zones[RIGHT] >= NORMAL_DISTANCE_LIMIT and 
            zones[INNER_RIGHT] >= INNER_DISTANCE_LIMIT)


def is_left_clear(zones):
    return (zones[OUTER_LEFT] >= OUTER_DISTANCE_LIMIT and
            zones[LEFT] >= NORMAL_DISTANCE_LIMIT and 
            zones[INNER_LEFT] >= INNER_DISTANCE_LIMIT)


def is_straight_ahead_blocked(zones):
    return (zones[INNER_LEFT] < INNER_DISTANCE_LIMIT and
            zones[INNER_RIGHT] < INNER_DISTANCE_LIMIT)


def is_straight_ahead_clear(zones):
        return (zones[INNER_LEFT] >= INNER_DISTANCE_LIMIT and
            zones[LEFT] >= NORMAL_DISTANCE_LIMIT and
            zones[INNER_RIGHT] >= INNER_DISTANCE_LIMIT and
            zones[RIGHT] >= NORMAL_DISTANCE_LIMIT)

def is_straight_ahead_clear_nimble(zones):
        return (zones[INNER_LEFT] >= INNER_DISTANCE_LIMIT and
            zones[INNER_RIGHT] >= INNER_DISTANCE_LIMIT)


def calculate_veer_angular_velocity(inner_distance, normal_distance, outer_distance):
    if inner_distance < INNER_DISTANCE_LIMIT:
        angular_vel = 0.5

    elif normal_distance < NORMAL_DISTANCE_LIMIT:
        angular_vel = 0.2

    elif outer_distance < OUTER_DISTANCE_LIMIT:
        angular_vel = 0.1

    return angular_vel


class Straight(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=['veer_left','veer_right','stuck','bump','exit'])
        self.vel_pub = pub_node
        self.rate = rate
    
    def execute(self, userdata):
        global g_zone_distances
        global g_bumped
        while not rospy.is_shutdown():
            current_zones = g_zone_distances

            if g_bumped:
                return 'bump'

            elif is_straight_ahead_blocked(current_zones):
                self.vel_pub.publish(Twist())
                return 'stuck'

            elif not is_left_clear(current_zones):
                return 'veer_right'

            elif not is_right_clear(current_zones):
                return 'veer_left'

            twist = Twist()
            twist.linear.x = 0.8
            self.vel_pub.publish(twist)
            self.rate.sleep()

        return 'exit'


class VeerLeft(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=['straight','stuck','bump','exit'])
        self.vel_pub = pub_node
        self.rate = rate
        self.direction = 1

    def execute(self, userdata):
        global g_zone_distances
        global g_bumped
        while not rospy.is_shutdown():
            current_zones = g_zone_distances

            if g_bumped:
                return 'bump'

            elif is_right_clear(current_zones):
                return 'straight'

            elif not is_left_clear(current_zones):
                self.vel_pub.publish(Twist())
                return 'stuck'

            twist = Twist()
            twist.angular.z = calculate_veer_angular_velocity(current_zones[INNER_RIGHT], current_zones[RIGHT], current_zones[OUTER_RIGHT]) * self.direction
            twist.linear.x = 0.5 - abs(twist.angular.z)
            self.vel_pub.publish(twist)
            self.rate.sleep()

        return 'exit'
            

class VeerRight(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=['straight','stuck','bump','exit'])
        self.vel_pub = pub_node
        self.rate = rate
        self.direction = -1

    def execute(self, userdata):
        global g_zone_distances
        global g_bumped
        while not rospy.is_shutdown():
            current_zones = g_zone_distances

            if g_bumped:
                return 'bump'

            elif is_left_clear(current_zones):
                return 'straight'

            elif not is_right_clear(current_zones):
                self.vel_pub.publish(Twist())
                return 'stuck'

            twist = Twist()
            twist.angular.z = calculate_veer_angular_velocity(current_zones[INNER_LEFT], current_zones[LEFT], current_zones[OUTER_LEFT]) * self.direction
            twist.linear.x = 0.5 - abs(twist.angular.z)
            self.vel_pub.publish(twist)
            self.rate.sleep()

        return 'exit'


class Bumped(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=['stuck', 'exit'])
        self.vel_pub = pub_node
        self.rate = rate

    def execute(self, userdata):
        self.vel_pub.publish(Twist())
        if rospy.is_shutdown():
            return 'exit'

        num_backup_msgs = 5
        backup_twist = Twist()
        backup_twist.linear.x = -0.2

        for x in range(num_backup_msgs):
            self.vel_pub.publish(backup_twist)
            self.rate.sleep()

        return 'stuck'


class Stuck(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=['straight', 'exit'])
        self.vel_pub = pub_node
        self.rate = rate

    def execute(self, userdata):
        global g_zone_distances
        global g_bumped

        self.vel_pub.publish(Twist())

        direction = (g_zone_distances[INNER_LEFT] - g_zone_distances[INNER_RIGHT]) / abs(g_zone_distances[INNER_LEFT] - g_zone_distances[INNER_RIGHT])
        
        starting_time = time()
        time_limit = 1
        while not rospy.is_shutdown():
            current_zones = g_zone_distances

            if time() - starting_time > time_limit:
                if is_straight_ahead_clear_nimble(current_zones):
                    return 'straight'

            else: 
                if is_straight_ahead_clear(current_zones):
                    return 'straight'

            twist = Twist()
            twist.angular.z = 0.6 * direction
            self.vel_pub.publish(twist)
            self.rate.sleep()

        return 'exit'

def main():
    global g_zone_distances
    global g_bumped

    g_zone_distances = [3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
    g_bumped = False

    rospy.init_node('robber_state_machine')

    state_machine = smach.StateMachine(outcomes=['exit'])
    state_introspection_server = smach_ros.IntrospectionServer('server_name', state_machine, '/SM_ROOT')
    state_introspection_server.start()

    rospy.Subscriber('zone_scan', ZoneScan, zone_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(10)

    with state_machine:
        smach.StateMachine.add('STRAIGHT', Straight(rate, cmd_vel_pub),
                                transitions={'veer_left': 'VEER_LEFT',
                                            'veer_right': 'VEER_RIGHT',
                                            'stuck': 'STUCK',
                                            'bump': 'BUMP',
                                            'exit': 'exit'})

        smach.StateMachine.add('VEER_LEFT', VeerLeft(rate, cmd_vel_pub),
                                transitions={'straight': 'STRAIGHT',
                                            'stuck': 'STUCK',
                                            'bump': 'BUMP',
                                            'exit': 'exit'})

        smach.StateMachine.add('VEER_RIGHT', VeerRight(rate, cmd_vel_pub),
                                transitions={'straight': 'STRAIGHT',
                                            'stuck': 'STUCK',
                                            'bump': 'BUMP',
                                            'exit': 'exit'})

        smach.StateMachine.add('STUCK', Stuck(rate, cmd_vel_pub),
                                transitions={'straight': 'STRAIGHT',
                                            'exit': 'exit'})

        smach.StateMachine.add('BUMP', Bumped(rate, cmd_vel_pub),
                                transitions={'stuck': 'STUCK',
                                            'exit': 'exit'})

    state_machine.execute()
    state_introspection_server.stop()
                                    
if __name__ == '__main__':
    main()