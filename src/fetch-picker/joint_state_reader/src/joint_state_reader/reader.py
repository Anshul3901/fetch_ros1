#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class JointStateReader(object):
    """Listens to /joint_states and provides the latest joint angles.

    Usage:
        joint_reader = JointStateReader()
        rospy.sleep(0.1)
        joint_reader.get_joint('shoulder_pan_joint')
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])
    """

    def __init__(self):
        self._joint_states = {}
        rospy.Subscriber('/joint_states', JointState, self._callback)

    def _callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self._joint_states[name] = position

    def get_joint(self, name):
        """Gets the latest joint value.

        Args:
            name: string, the name of the joint whose value we want to read.

        Returns:
            The joint value, or None if we do not have a value yet.
        """
        return self._joint_states.get(name, None)

    def get_joints(self, names):
        """Gets the latest values for a list of joint names.

        Args:
            names: list of strings, the names of the joints whose values we want to read.

        Returns:
            A list of the joint values. Values may be None if we do not have a value for that joint yet.
        """
        return [self._joint_states.get(name, None) for name in names]
