#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction 
import robot_api


def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print('Usage: rosrun applications check_cart_pose.py plan X Y Z')
    print('       rosrun applications check_cart_pose.py ik X Y Z')


def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode."""
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


def compute_ik(pose_stamped, timeout=rospy.Duration(5)):
    """Computes IK for the given pose using the MoveIt service."""
    rospy.wait_for_service('compute_ik')
    compute_ik_srv = rospy.ServiceProxy('compute_ik', GetPositionIK)

    request = GetPositionIKRequest()
    request.ik_request.pose_stamped = pose_stamped
    request.ik_request.group_name = 'arm'
    request.ik_request.timeout = timeout

    try:
        response = compute_ik_srv(request)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return False

    error_str = moveit_error_string(response.error_code.val)
    if error_str != 'SUCCESS':
        return False

    joint_state = response.solution.joint_state
    for name, position in zip(joint_state.name, joint_state.position):
        rospy.loginfo('%s: %.4f' % (name, position))

    return True


def main():
    rospy.init_node('check_cart_pose')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 5:
        print_usage()
        return

    command, x, y, z = argv[1], float(argv[2]), float(argv[3]), float(argv[4])
    arm = robot_api.Arm()

    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.w = 1.0  # Identity orientation

    if command == 'plan':
        error = arm.check_pose(ps, allowed_planning_time=1.0)
        if error is None:
            rospy.loginfo('Found plan!')
        else:
            rospy.loginfo('No plan found.')
        arm.cancel_all_goals()

    elif command == 'ik':
        if compute_ik(ps):
            rospy.loginfo('Found IK!')
        else:
            rospy.loginfo('No IK found.')
    else:
        print_usage()


if __name__ == '__main__':
    main()
