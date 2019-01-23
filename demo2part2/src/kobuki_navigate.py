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


g_x = None
g_y = None
g_z = None
g_theta = None

def odom_callback(self, msg):
    global g_x
    global g_y
    global g_z
    global g_theta

    pose = numpify(msg.pose.pose)
    _, _, angles, _ = decompose_matrix(pose)
    
    g_x = msg.pose.pose.position.x
    g_y = msg.pose.pose.position.y
    g_z = msg.pose.pose.position.z
    g_theta = angles[2]

class Forward(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['backup', 'finished'])
      
        self.mutex = threading.Lock()

        self.bumper_hit = False
        self.bumper_node = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.kobuki_movement = movement_pub_node

    def bumper_callback(self, msg):
        self.mutex.acquire()
        if msg.bumper == 1 and msg.state == 1:
            self.bumper_hit = True
        self.mutex.release()


    def __execute__(self):
        while(True):
            self.mutex.acquire()
            if self.bumper_hit:
                return 'backup'            
            self.mutex.release()

            twist = Twist()
            twist.linear.x = 1

            # TODO: Read and store data from encoders

            ''' TODO:
            if self.position.x == 3:
                return 'finished'
            '''

            self.kobuki_movement.publish(twist)


class Backup(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['turn_horizontal'])
        self.kobuki_movement = movement_pub_node

    def __execute__(self):
        #TODO: backup for a set amount of distance
        return 'turn_horizontal'

    
class TurnHorizontal(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['move_horizontal'])
        self.kobuki_movement = movement_pub_node

    def __execute__(self):
        #TODO: Determine direction
        # Eiter turn 90 degrees or 180 degrees
        return 'move_horizontal'


class Horizontal(smach.State):
    def __init__(self, movement_pub_node):
        smach.State.__init__(self, outcomes=['success','backup'])
        self.mutex = threading.Lock()

        self.bumper_hit = False
        self.bumper_node = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.kobuki_movement = movement_pub_node

    def bumper_callback(self, msg):
        self.mutex.acquire()
        if msg.bumper == 1 and msg.state == 1:
            self.bumper_hit = True
        self.mutex.release()

    def __execute__(self):
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
        # TODO: Determine direction to turn
        # TODO: Turn 90 degrees
        return 'move_forward'

class Finished(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])
        
    def __execute__(self):
        # TODO: Play a nice song / flash light
        return 'exit'


def main():
    rospy.init_node('nav_state_machine')

    state_machine = smach.StateMachine(outcomes=['exit'])
    state_introspection_server = smach_ros.IntrospectionServer('server_name', state_machine, '/SM_ROOT')
    state_introspection_server.start()

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    self.odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

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

        smach.StateMachine.add("FINISHED", TurnForward(cmd_vel_pub),
                                transitions={"exit": "exit"})

    state_machine.execute()
    state_introspection_server.stop()
                                    
if __name__ == '__main__':
    main()



