#!/usr/bin/env python
import rospy
import smach
import smach_ros

import math

DISTANCE_LIMIT = 3

g_distances = [3, 3, 3, 3, 3, 3]
FAR_LEFT = 0
LEFT = 1
CLOSE_LEFT = 2
CLOSE_RIGHT = 3
RIGHT = 4
FAR_RIGHT = 5

class Driving(smach.State):
    def __init__(self, pub_node):
        smach.State.__init__(self, outcomes=['veer_left', 'veer_right', 'stuck', 'bump'])
        self.vel_pub = pub_node

    def execute(self):
        global g_distances
        print "Driving"


class VeerRight(smach.State):
    def __init__(self, pub_node):
        smach.State.__init__(self, outcomes=['driving', 'stuck', 'bump'])
        self.vel_pub = pub_node

    def execute(self):
        global g_distances
        print "Veer Right"


class VeerLeft(smach.State):
    def __init__(self, pub_node):
        smach.State.__init__(self, outcomes=['driving', 'stuck', 'bump'])
        self.vel_pub = pub_node
    
    def execute(self):
        global g_distances
        print "Veer Left"


class Stuck(smach.State):
    def __init__(self, pub_node):
        smach.State.__init__(self, outcomes=['driving', 'bump'])
        self.vel_pub = pub_node

    def execute(self):
        global g_distance
        print "Stuck"

class Bump(smach.State):
    def __init__(self, pub_node):
        smach.State.__init__(self, outcomes=['driving'])
        self.vel_pub = pub_node

    def execute(self):
        global g_distance
        print "Bump"