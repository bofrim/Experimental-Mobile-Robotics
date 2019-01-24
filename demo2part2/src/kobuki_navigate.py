#!/usr/bin/env python
import rospy
import smach
import smach_ros
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import decompose_matrix
from ros_numpy import numpify

FINISH_DISTANCE_M = 3
TWIST_PUB_FREQ = 10

g_x = None
g_y = None
g_z = None
g_theta = None

g_bumper = None

def odom_callback(msg):
    global g_x
    global g_y
    global g_z
    global g_theta

    pose = numpify(msg.pose.pose)
    _, _, angles, _, _ = decompose_matrix(pose)
    
    g_x = msg.pose.pose.position.x
    g_y = msg.pose.pose.position.y
    g_z = msg.pose.pose.position.z
    g_theta = angles[2]

def bumper_callback(msg):
    global g_bumper
    g_bumper = bool(msg.state)

class Forward(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['backup', 'finished'])
        self.rate = rospy.Rate(TWIST_PUB_FREQ)
        self.kobuki_movement = movement_pub_node

    def execute(self, userdata):
        while not g_bumper:
            twist = Twist()
            twist.linear.x = 1
            self.kobuki_movement.publish(twist)

            if g_x > FINISH_DISTANCE_M:
                return 'finished'
            
            self.rate.sleep()

        self.kobuki_movement.publish(Twist())
        return 'backup'            


class Backup(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['turn_horizontal'])
        self.kobuki_movement = movement_pub_node

    def execute(self):
        #TODO: backup for a set amount of distance
        backup_twist = Twist()
        backup_twist.linear.x = -1
        num_backup_msgs = 10

        for x in range(num_backup_msgs):
            self.kobuki_movement.publish(backup_twist)
        
        self.kobuki_movement.publish(Twist())
        return 'turn_horizontal'

    
class TurnHorizontal(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['move_horizontal'])
        self.kobuki_movement = movement_pub_node

    def __execute__(self):
        global g_theta

        if -10 < g_theta < 10:
            turn_kobuki(90, self.kobuki_movement, None)
        elif 80 < g_theta and g_theta < 100:
            turn_kobuki(-90, self.kobuki_movement, None)
        
        return 'move_horizontal'


class Horizontal(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['turn_forward','backup'])
        self.mutex = threading.Lock()

        self.bumper_hit = False
        self.bumper_node = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.kobuki_movement = movement_pub_node

    def bumper_callback(self, msg):
        self.mutex.acquire()
        if msg.bumper == 1 and msg.state == 1:
            self.bumper_hit = True
        self.mutex.release()

    def execute(self):
        self.mutex.acquire()
        if self.bumper_hit:
            return 'backup'
        self.mutex.release()

        #TODO: Drive horizontal for set amount of distance

        return 'success'

class TurnForward(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['move_forward'])
        self.kobuki_movement = movement_pub_node

    def __execute__(self):
        global g_theta
        turn_kobuki(0, self.kobuki_movement, None)

        return 'move_forward'

class Finished(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])
        
    def execute(self):
        # TODO: Play a nice song / flash light
        return 'exit'


def turn_kobuki(desired_angle, kobuki_pub_node, pub_rate):
    global g_theta
    lower_bound = desired_angle - 3
    upper_bound = desired_angle + 3
 
    while( g_theta < lower_bound or g_theta > upper_bound):
        rotation_direction = (desired_angle - g_theta) / abs( desired_angle - g_theta)

        twist = Twist()
        twist.angular.z = rotation_direction * 0.2

        kobuki_pub_node.publish(twist)
             

def main():
    rospy.init_node('nav_state_machine')

    state_machine = smach.StateMachine(outcomes=['exit'])
    state_introspection_server = smach_ros.IntrospectionServer('server_name', state_machine, '/SM_ROOT')
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)

    with state_machine:
        smach.StateMachine.add("FORWARD", Forward(cmd_vel_pub),
                                transitions={"backup": "BACKUP",
                                             "finished": "FINISHED"})
        
        smach.StateMachine.add("BACKUP", Backup(cmd_vel_pub),
                                transitions={"turn_horizontal": "TURN_HORIZONTAL"})

        smach.StateMachine.add("TURN_HORIZONTAL", TurnHorizontal(cmd_vel_pub),
                                transitions={"move_horizontal": "MOVE_HORIZONTAL"})

        smach.StateMachine.add("MOVE_HORIZONTAL", Horizontal(cmd_vel_pub),
                                transitions={"backup": "BACKUP",
                                             "turn_forward": "TURN_FORWARD"})

        smach.StateMachine.add("TURN_FORWARD", TurnForward(cmd_vel_pub),
                                transitions={"move_forward": "FORWARD"})

        smach.StateMachine.add("FINISHED", Finished(),
                                transitions={"exit": "exit"})

    state_machine.execute()

    state_introspection_server.stop()

                                    
if __name__ == '__main__':
    main()



