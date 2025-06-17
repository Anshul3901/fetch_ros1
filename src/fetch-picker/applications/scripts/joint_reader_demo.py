#!/usr/bin/env python

import rospy
from joint_state_reader import JointStateReader
import robot_api


def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_reader_demo')
    wait_for_time()

    reader = JointStateReader()
    rospy.sleep(1.0)  # Give the JointStateReader time to receive a message

    # Fetch Only: Get names of arm joints
    names = robot_api.ArmJoints.names()
    # For Kuri: manually specify names like ['joint1', 'joint2', ...] if needed

    arm_vals = reader.get_joints(names)

    for name, val in zip(names, arm_vals):
        print('{}\t{}'.format(name, val if val is not None else 'No Data'))


if __name__ == '__main__':
    main()
