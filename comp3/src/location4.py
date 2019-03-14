#!/usr/bin/env python
import actionlib
import rospy, cv2, cv_bridge, numpy
import smach, smach_ros
import tf

from math import sqrt

from image_processing import detect_shape, get_red_mask
from utils import display_count

from comp2.msg import Centroid
from general_states import Drive

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Twist, Point, Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, Pose
from sensor_msgs.msg import Joy, Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led

MARKER_POSE_TOPIC = "ar_pose_marker"

WAYPOINT_MAP = {
    "off_ramp": (Point(-0.04193958488532269, -0.08882399277556528, 0.010199999999999999), Quaternion(0.0, 0.0, 0.34350948056213637, 0.9391492089992576)),
    "1": (Point(1.4128346917258936, 3.0314988225881017, 0.0102), Quaternion(0.0, 0.0, 0.11511062216559612, 0.9933526788933774)),
    "2": (Point(1.4974838033521558, 2.281408009296837, 0.0102), Quaternion(0.0, 0.0, 0.07185644361174354,0.9974149846034358)),
    "3": (Point(1.6296398016279827, 1.4617992377591547, 0.0102), Quaternion(0.0, 0.0, 0.07214590937638982, 0.997394088492735)),
    "4": (Point(1.7891876967063711, 0.6209652252079119, 0.0102), Quaternion(0.0, 0.0, -0.005679318202515465, 0.9999838725423299)),
    "5": (Point(1.9150393764830873, -0.21134129264055662, 0.010199999999999999), Quaternion(0.0, 0.0, -0.2439218105436791, 0.9697949011729714)),
    "6": (Point(0.20461857551725132, 1.6533389602064918, 0.010200000000000004), Quaternion(0.0,0.0,0.9964777382330111,-0.08385772001445518)),
    "7": (Point(0.4272674336836944, 0.83219598399239, 0.0102), Quaternion(0.0, 0.0, 0.9947478059045937, -0.10235625358519646)),
    "8": (Point(1.0648070049458176, -0.302468245750806, 0.010199999999999999), Quaternion(0.0, 0.0, -0.6178099553656561, 0.7863274502718863)),
    "far": (Point(0.8348034063867399, 2.058496798133918, 0.010199999999999999), Quaternion(0.0, 0.0, 0.4270189450655432, 0.9042426779106981)),
    "on_ramp": (Point(-0.8334473332557941,2.584811945485156, 0.0102), Quaternion(0.0, 0.0, -0.9617331318932912, 0.2739879249505739)),
    "scan": (Point(1.10, 1.4855115054663668, 0.0102), Quaternion(0.0, 0.0, 0.0009838736033419004, 0.9999995159962491)),
}

#    "scan": (Point(0.9380181464288736, 1.4855115054663668, 0.0102), Quaternion(0.0, 0.0, 0.0009838736033419004, 0.9999995159962491)),


class DriverRamp(Drive):
    def __init__(self, rate, pub_node):
        super(DriverRamp, self).__init__(rate, pub_node, ["drive_to_start", "exit"])

    def execute(self, userdata):
        self.stop_distance = -1
        white_line_sub = rospy.Subscriber(
            "white_line_ramp_centroid", Centroid, self.image_callback
        )
        pose_pub = rospy.Publisher(
            "initialpose", PoseWithCovarianceStamped, queue_size=1
        )

        while not rospy.is_shutdown():
            if self.path_centroid.cx == -1 or self.path_centroid.cy == -1:
                rospy.loginfo("Sending initial pose...")
                initial_pose_cov_stamp = PoseWithCovarianceStamped()
                initial_pose_cov = PoseWithCovariance()
                initial_pose = Pose()
                initial_pose.position = WAYPOINT_MAP["off_ramp"][0]
                initial_pose.orientation = WAYPOINT_MAP["off_ramp"][1]
                initial_pose_cov.pose = initial_pose
                initial_pose_cov_stamp.pose = initial_pose_cov
                
                rospy.sleep(0.1)
                pose_pub.publish(initial_pose_cov_stamp)
                rospy.loginfo(" ...Sent.")

                pose_pub.unregister()
                white_line_sub.unregister()
                return "drive_to_start"

            #TODO: Add in condition that robot takes the wrong path and sees a red line

            self.vel_pub.publish(self.twist)
            self.rate.sleep()

        pose_pub.unregister()
        white_line_sub.unregister()
        return "exit"

class DriveToStart(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["ar_survey", "parking_spot", "shape_survey", "on_ramp"])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.start_pose = MoveBaseGoal()
        self.start_pose.target_pose.header.frame_id = 'map'
        self.start_pose.target_pose.pose.position = WAYPOINT_MAP["scan"][0]
        self.start_pose.target_pose.pose.orientation = WAYPOINT_MAP["scan"][1]
        self.current_task = 0
        self.task_list = ["ar_survey", "parking_spot", "on_ramp"]

    def execute(self, userdata):
        rospy.sleep(0.5)
        self.client.send_goal(self.start_pose)
        self.client.wait_for_result()

        return "shape_survey"

        next_state = self.task_list[self.current_task]
        self.current_task += 1
        return next_state


class ArSurvey(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["ar_approach", "exit"],
                                   output_keys=["target_marker"])
        self.pub_node = pub_node
        self.target_marker = None
        self.ar_focus_id = -1
        self.target_repetitions = 0

    def execute(self, userdata):
        self.target_marker = None
        rate = rospy.Rate(10)
        ar_sub = rospy.Subscriber(
            MARKER_POSE_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )
        
        while not rospy.is_shutdown():
            if self.target_marker and self.target_repetitions >= 3 and -0.3 < self.target_marker.pose.pose.position.y and self.target_marker.pose.pose.position.y < 0.3:
                userdata.target_marker = self.target_marker
                ar_sub.unregister()
                return "ar_approach"

            twist_msg = Twist()
            twist_msg.angular.z = 0.3

            self.pub_node.publish(twist_msg)
            rate.sleep()

        ar_sub.unregister()
        return "exit"

    def ar_callback(self, msg):
        if not msg.markers:
            self.ar_focus_id = -1
            self.target_repetitions = 0
            return 

        for marker in msg.markers:
            if marker.id == self.ar_focus_id:
                self.target_repetitions = self.target_repetitions + 1
            else:
                self.ar_focus_id = marker.id
                self.target_repetitions = 0
        
            self.target_marker = marker
            break


class ArApproach(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_start", "exit"],
                                   input_keys=["target_marker"])
        self.pub_node = pub_node
        self.target_marker = None
        self.target_marker_id = None
        self.target_marker_frame = None
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        self.target_marker = userdata.target_marker
        self.target_marker_id = userdata.target_marker.id
        self.target_marker_frame = "ar_marker_" + str(self.target_marker_id)

        ar_sub = rospy.Subscriber(
            MARKER_POSE_TOPIC, AlvarMarkers, self.ar_callback, queue_size=1
        )

        print("Approach with cmd_vel")
        while self.target_marker.pose.pose.position.x > 0.80 and not rospy.is_shutdown():
            direction = self.target_marker.pose.pose.position.y / abs(self.target_marker.pose.pose.position.y)
            msg = Twist()
            msg.angular.z = 0.2 * direction
            msg.linear.x = 0.2
            self.pub_node.publish(msg)
            print("Distance = ", self.target_marker.pose.pose.position.x)

        if rospy.is_shutdown():
            ar_sub.unregister()
            return "exit"

        print("Approach with planner")
        self.client.send_goal(self.calculate_target())
        self.client.wait_for_result()

        display_count(2)

        rospy.sleep(2)
        ar_sub.unregister()
        return "drive_to_start"
    
    def calculate_target(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.target_marker_frame
        goal.target_pose.pose.position = Point(0.0, 0.0, 0.25)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0.84147098, 0.54030231)
        return goal
        
    def ar_callback(self, msg):
        for marker in msg.markers:
            if marker.id == self.target_marker_id:
                self.target_marker = marker


class ParkingSpot(smach.State):
    def __init__(self, rate, parking_spot):
        smach.State.__init__(self, outcomes=["drive_to_start"])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.parking_spot = parking_spot
        self.rate = rate

    def execute(self, userdata):
        if self.parking_spot < 1 or self.parking_spot > 8:
            return "drive_to_start"
        
        pose = MoveBaseGoal()
        pose.target_pose.header.frame_id = 'map'
        pose.target_pose.pose.position = WAYPOINT_MAP[str(self.parking_spot)][0]
        pose.target_pose.pose.orientation = WAYPOINT_MAP[str(self.parking_spot)][1]

        self.client.send_goal(pose)
        self.client.wait_for_result()

        display_count(2, Led.RED)
        rospy.sleep(2)

        return "drive_to_start"


class ShapeApproach(smach.State):
    def __init__(self, rate, pub_node):
        smach.State.__init__(self, outcomes=["drive_to_start", "exit"])
        self.pub_node = pub_node
        self.location_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.shape_type = None
        self.shape_size = 0
        self.shape_centroid = Centroid()
        self.shape_centroid.cx = -1
        self.shape_centroid.cy = -1
        self.shape_centroid.err = 0

    def execute(self, userdata):
        image_sub = rospy.Subscriber(
            "camera/rgb/image_raw", Image, self.image_callback
        )

        rospy.sleep(5)
        print "INIT SIZE: " + str(self.shape_size)
        while self.shape_size < 6474 and not rospy.is_shutdown():
            if self.shape_size == 0:
                print "no red friends :("
                continue

            print "SIZE: " + str(self.shape_size)
            print "ERROR: " + str(self.shape_centroid.err)
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = (-float(self.shape_centroid.err) / 75)
            self.pub_node.publish(twist)
            rospy.sleep(0.1)

        if rospy.is_shutdown():
            image_sub.unregister()
            return "exit"

        print "Close Enough"
        self.client.send_goal(self.calculate_target())
        self.client.wait_for_result()

        display_count(2, Led.ORANGE)

        rospy.sleep(2)
        image_sub.unregister()
        return "drive_to_start"
    
    def calculate_target(self):
        trans, rot = self.location_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        base_x, base_y, base_z = trans

        closest_distance = 1000
        closest_waypoint_pose = None
        closest_waypoint_rot = None

        for waypoint_num in range(1, 9):
            waypoint_pose = WAYPOINT_MAP[str(waypoint_num)][0]
            waypoint_rot = WAYPOINT_MAP[str(waypoint_num)][1]
            eucl_distance = sqrt(pow((waypoint_pose.x - base_x), 2) + pow((waypoint_pose.y - base_y), 2))

            if eucl_distance < closest_distance:
                closest_distance = eucl_distance
                closest_waypoint_pose = waypoint_pose
                closest_waypoint_rot = waypoint_rot
                
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position = closest_waypoint_pose
        goal.target_pose.pose.orientation = closest_waypoint_rot
        return goal

    def image_callback(self, msg):
        mask = get_red_mask(msg)

        height, width = mask.shape
        search_top = height * 0.30
        search_bot = height * 0.65
        mask[0:search_top, 0:width] = 0
        mask[search_bot:height, 0:width] = 0

        shapes, moments = detect_shape(mask, threshold=450)

        cv2.imshow("WOW", mask)
        cv2.waitKey(1)

        shape_centroid = Centroid()
        shape_centroid.cx = -1
        shape_centroid.cy = -1
        shape_centroid.err = 1000

        target_shape = None
        target_shape_size = 0

        for moment, shape in zip(moments, shapes):
            cx, cy = int(moment["m10"] / moment["m00"]), int(moment["m01"] / moment["m00"])
            size = int(moment["m00"])
            cx_error = cx - width / 2

            if abs(cx_error) < abs(shape_centroid.err):
                shape_centroid.cx = cx
                shape_centroid.cy = cy
                shape_centroid.err = cx_error
                target_shape = shape
                target_shape_size = size
                
        self.shape_centroid = shape_centroid
        self.shape_size = target_shape_size
        self.shape_type = target_shape
 

class OnRamp(smach.State):
    def __init__(self, rate):
        smach.State.__init__(self, outcomes=["drive"])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.rate = rate

    def execute(self, userdata):
        pose_prepare = MoveBaseGoal()
        pose_prepare.target_pose.header.frame_id = 'map'
        pose_prepare.target_pose.pose.position = WAYPOINT_MAP["1"][0]
        pose_prepare.target_pose.pose.orientation = WAYPOINT_MAP["1"][1]

        self.client.send_goal(pose_prepare)
        self.client.wait_for_result()

        pose_ramp = MoveBaseGoal()
        pose_ramp.target_pose.header.frame_id = 'map'
        pose_ramp.target_pose.pose.position = WAYPOINT_MAP["on_ramp"][0]
        pose_ramp.target_pose.pose.orientation = WAYPOINT_MAP["on_ramp"][1]

        self.client.send_goal(pose_ramp)
        self.client.wait_for_result()

        return "drive"


if __name__ == "__main__":
    rospy.init_node("test_shape_recog")
    
    state_machine = smach.StateMachine(outcomes=["exit"])
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10)

    with state_machine:
        smach.StateMachine.add(
            "SHAPE_APPROACH",
            ShapeApproach(rate, cmd_vel_pub),
            transitions={"drive_to_start": "exit", "exit": "exit"},
        )

    state_machine.execute()
    state_introspection_server.stop()