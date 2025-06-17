#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal  # TODO: import control_msgs_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint  # TODO: import trajectory_msgs.msg

# TODO: ACTION_NAME = ???
ACTION_NAME = '/torso_controller/follow_joint_trajectory'

# TODO: JOINT_NAME = ???
JOINT_NAME = 'torso_lift_joint'

TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)

        # TODO: Wait for server
        rospy.loginfo("Waiting for torso action server...")
        self.client.wait_for_server()
        rospy.loginfo("Torso action server ready.")

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT (0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height < self.MIN_HEIGHT or height > self.MAX_HEIGHT:
            rospy.logwarn(f"Requested height {height} is out of bounds. Clipping to limits.")
            height = max(self.MIN_HEIGHT, min(height, self.MAX_HEIGHT))

        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()

        # TODO: Set position of trajectory point
        point.positions = [height]  # Only one joint: torso_lift_joint

        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()

        # TODO: Add joint name to list
        goal.trajectory.joint_names = [JOINT_NAME]

        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)


        rospy.loginfo("Moving Torso")
        # TODO: Send goal
        self.client.send_goal(goal)

        # TODO: Wait for result
        self.client.wait_for_result()
        rospy.loginfo("Done Moving Torso")
