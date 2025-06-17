# TODO: import actionlib
import actionlib
# TODO: import control_msgs.msg
import control_msgs.msg
# TODO: import trajectory_msgs.msg
import trajectory_msgs.msg
from moveit_msgs.msg import MoveGroupAction
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction                                   


import rospy

from .arm_joints import ArmJoints

ARM_ACTION_NAME = '/arm_controller/follow_joint_trajectory'
TRAJECTORY_TIME = 5.0  # seconds

def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'                                      
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'
      

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        self._client = actionlib.SimpleActionClient(
            ARM_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # TODO: Wait for server
        rospy.loginfo('Waiting for arm action server...')
        self._client.wait_for_server()
        rospy.loginfo('Arm action server is ready.')
         # Create MoveGroup action client
        self._move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        rospy.loginfo("Waiting for /move_group action server...")
        self._move_group_client.wait_for_server()
        rospy.loginfo("Connected to /move_group action server.")

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        point.positions = arm_joints.values()
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(TRAJECTORY_TIME)

        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory.joint_names = ArmJoints.names()
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now()

        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
    
    

    def cancel_all_goals(self):
        self._client.cancel_all_goals() # Your action client from Lab 7
        self._move_group_client.cancel_all_goals() # From this lab
    
  
    def move_to_pose(self,
                 pose_stamped,
                 allowed_planning_time=10.0,
                 execution_timeout=15.0,
                 group_name='arm',
                 num_planning_attempts=1,
                 plan_only=False,
                 replan=False,
                 replan_attempts=5,
                 tolerance=0.01,
                 orientation_constraint=None):
        """
        Moves the end-effector to a pose, using motion planning.

        Args:
            pose_stamped: geometry_msgs/PoseStamped. The goal pose for the gripper.
            allowed_planning_time: float. The max planning time in seconds.
            execution_timeout: float. The max duration to wait for execution.
            group_name: str. Either 'arm' or 'arm_with_torso'.
            num_planning_attempts: int. Number of attempts to find a plan.
            plan_only: bool. If True, only plans but does not execute.
            replan: bool. If True, allows replanning if execution fails.
            replan_attempts: int. Max number of replans.
            tolerance: float. Tolerance in meters.
            orientation_constraint: moveit_msgs/OrientationConstraint or None. Optional constraint on the end-effector orientation.

        Returns:
            A string describing the error if any occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        goal_builder.group_name = group_name

        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)

        goal = goal_builder.build()

        self._move_group_client.send_goal(goal)
        finished = self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))

        if not finished:
            self._move_group_client.cancel_goal()
            return "Timed out waiting for result"

        result = self._move_group_client.get_result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return moveit_error_string(result.error_code.val)

        return None

    
    def check_pose(self, 
                   pose_stamped,
                   allowed_planning_time=10.0,
                   group_name='arm',
                   tolerance=0.01):
        """Checks if a pose is reachable without executing."""
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)
