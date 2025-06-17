#!/usr/bin/env python

import actionlib
import math
import rospy

import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg

LOOK_AT_ACTION_NAME = '/head_controller/point_head'
PAN_TILT_ACTION_NAME = '/head_controller/follow_joint_trajectory'
PAN_JOINT = 'head_pan_joint'
TILT_JOINT = 'head_tilt_joint'
PAN_TILT_TIME = 2.5  # seconds


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians
    """
    MIN_PAN = -math.radians(150)
    MAX_PAN = math.radians(150)
    MIN_TILT = -math.radians(50)
    MAX_TILT = math.radians(60)

    def __init__(self):
        self._look_at_client = actionlib.SimpleActionClient(
            LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        self._pan_tilt_client = actionlib.SimpleActionClient(
            PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)

        rospy.loginfo('Waiting for head controllers...')
        self._look_at_client.wait_for_server()
        self._pan_tilt_client.wait_for_server()
        rospy.loginfo('Head controllers are up!')

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space."""
        goal = control_msgs.msg.PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.pointing_axis.x = 1.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 0.0
        goal.pointing_frame = 'head_link'
        goal.min_duration = rospy.Duration(1.0)

        self._look_at_client.send_goal(goal)
        self._look_at_client.wait_for_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles."""
        if not (self.MIN_PAN <= pan <= self.MAX_PAN):
            rospy.logwarn(f'Pan angle {pan} out of bounds.')
            return
        if not (self.MIN_TILT <= tilt <= self.MAX_TILT):
            rospy.logwarn(f'Tilt angle {tilt} out of bounds.')
            return

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now()

        self._pan_tilt_client.send_goal(goal)
        self._pan_tilt_client.wait_for_result()
