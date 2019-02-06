#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Drive(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["stop", "exit"])
        self.bridge = cv_bridge.CvBridge()
        self.rate = rate
        self.vel_pub = pub_node
        self.stop_sub = rospy.Subscriber('red_line_distance', Float32, self.red_line_callback)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.stop_distance = -1
        self.prev_err = 0
        self.twist = Twist()

    def red_line_callback(self, msg):
        self.stop_distance = msg.data

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([ 0,  0,  0])
        upper_white = numpy.array([0, 0, 250])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            curr_err = cx - w/2
            delta_err = curr_err - self.prev_err

            self.twist.linear.x = 0.3
            self.twist.angular.z = (-float(curr_err) / 100) + (-float(delta_err)/ 100)


    def execute(self, userdata):
        while not rospy.is_shutdown():

            if 0 < self.stop_distance < 0.3:
                return "stop"

            self.vel_pub.publish(self.twist)
            self.rate.sleep()


class Stop(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive", "exit"])
        self.rate = rate
        self.vel_pub = pub_node

    def execute(self, userdata):
        print "Stop"


def main():
    rospy.init_node("line_follower_sm")

    state_machine = smach.StateMachine(outcomes=["exit"])
    state_introspection_server = smach_ros.IntrospectionServer(
        "server_name", state_machine, "/SM_ROOT"
    )

    state_introspection_server.start()
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(20)

    with state_machine:
        smach.StateMachine.add(
            "DRIVE",
            Drive(rate, cmd_vel_pub),
            transitions={
                "stop": "STOP",
                "exit": "exit",
            },
        )

        smach.StateMachine.add(
            "STOP",
            Stop(rate, cmd_vel_pub),
            transitions={
                "drive": "DRIVE",
                "exit": "exit",
            },
        )

    state_machine.execute()
    state_introspection_server.stop()


if __name__ == "__main__":
    main()