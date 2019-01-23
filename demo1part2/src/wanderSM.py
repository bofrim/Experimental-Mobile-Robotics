#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

g_stop_distance = None

# TODO: change this global var
g_range_ahead = 1
def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)

class Turning(smach.State):
    def __init__(self, pub_node):
        smach.State.__init__(self, outcomes=['turn_time_limit', 'shutting_down'])
        self.cmd_vel_pub = pub_node

    def execute(self, userdata):
        time_limit = rospy.Time.now() + rospy.Duration(5)

        while not rospy.is_shutdown():
            if rospy.Time.now() > time_limit:
                return 'turn_time_limit'

            twist = Twist()
            twist.angular.z = 1

            self.cmd_vel_pub.publish(twist)

        return 'shutting_down'    


class Straight(smach.State):
    def __init__(self, pub_node):
        smach.State.__init__(self, outcomes=['straight_time_limit', 'barrier', 'shutting_down'])
        self.cmd_vel_pub = pub_node

    def execute(self, userdata):
        time_limit = rospy.Time.now() + rospy.Duration(30)
        while not rospy.is_shutdown():
            if rospy.Time.now() > time_limit:
                return 'straight_time_limit'
            #elif g_range_ahead < stop_distance 
            elif g_range_ahead < g_stop_distance:
                return 'barrier'

            twist = Twist()
            twist.linear.x = 1

            self.cmd_vel_pub.publish(twist)
        
        return 'shutting_down'


def main():
    global g_stop_distance

    rospy.init_node('wander_state_machine')

    if rospy.has_param("~stop_distance"):
        g_stop_distance = rospy.get_param("~stop_distance")

    state_machine = smach.StateMachine(outcomes=['exit'])
    state_introspection_server = smach_ros.IntrospectionServer('server_name', state_machine, '/SM_ROOT')
    state_introspection_server.start()

    rospy.Subscriber('scan', LaserScan, scan_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    with state_machine:
        smach.StateMachine.add('TURNING', Turning(cmd_vel_pub),
                                transitions={'turn_time_limit': 'STRAIGHT',
                                             'shutting_down': 'exit'})

        smach.StateMachine.add('STRAIGHT', Straight(cmd_vel_pub),
                                transitions={'straight_time_limit': 'TURNING',
                                             'barrier': 'TURNING',
                                             'shutting_down': 'exit'})

    state_machine.execute()
    state_introspection_server.stop()
                                    
if __name__ == '__main__':
    main()



