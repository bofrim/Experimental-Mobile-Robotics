import actionlib
import rospy
import smach

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist


class DriveState(smach.State):
    """A state that can directly control the robot."""

    def __init__(self, outcomes, rate_hz=10):
        smach.State.__init__(self, outcomes=outcomes)
        self.rate = rospy.Rate(rate_hz)
        self.pub_node = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def execute(self, userdata):
        raise NotImplementedError()


class NavState(smach.State):
    """A state that uses the nav stack to control the robot."""

    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        raise NotImplementedError()

    def move_to_relative_point(self, position, orientation, relative_frame, block=True):
        """Use the move base server to navigate to a point.

        :param positin: <Point> A position relative to the relative_frame coord frame
        :param orientation: <Quaternion> Orientation relative to relative_frame
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = relative_frame
        goal.target_pose.pose.position = position
        goal.target_pose.pose.orientation = orientation
        self.client.send_goal(goal)
        if block:
            self.client.wait_for_result()
