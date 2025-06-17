#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import OrientationConstraint
from moveit_python import PlanningSceneInterface
import robot_api


def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_obstacle_demo')
    wait_for_time()

    # Initialize planning scene
    planning_scene = PlanningSceneInterface("base_link")
    rospy.sleep(1.0)

    # ---- Table obstacle ----
    planning_scene.removeCollisionObject('table')
    table_size_x = 0.5
    table_size_y = 1.0
    table_size_z = 0.03
    table_x = 0.8
    table_y = 0.0
    table_z = 0.6
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                          table_x, table_y, table_z)

    # ---- Divider obstacle (removed for better performance with constraints) ----
    planning_scene.removeCollisionObject('divider')
    # Commenting out addition of divider to avoid planning slowdown with constraints
    size_x = 0.3
    size_y = 0.01
    size_z = 0.4
    x = table_x - (table_size_x / 2) + (size_x / 2)
    y = 0.0
    z = table_z + (table_size_z / 2) + (size_z / 2)
    catkinplanning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)

    rospy.sleep(1.0)

    # Initialize arm
    arm = robot_api.Arm()

    def shutdown():
        arm.cancel_all_goals()
        planning_scene.removeCollisionObject('table')
        planning_scene.removeCollisionObject('divider')
        planning_scene.removeAttachedObject('tray')

    rospy.on_shutdown(shutdown)

    # ---- Pose definitions ----
    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.5
    pose1.pose.position.y = -0.3
    pose1.pose.position.z = 0.75
    pose1.pose.orientation.w = 1.0

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.5
    pose2.pose.position.y = 0.3
    pose2.pose.position.z = 0.75
    pose2.pose.orientation.w = 1.0

    # ---- Common kwargs ----
    kwargs = {
        'allowed_planning_time': 15,
        'execution_timeout': 10,
        'num_planning_attempts': 5,
        'replan': False
    }

    # ---- Move to pose 1 ----
    planning_scene.removeAttachedObject('tray')
    error = arm.move_to_pose(pose1, **kwargs)
    if error is not None:
        rospy.logerr('Pose 1 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 1 succeeded')

        # ---- Simulate object grasp and attach tray ----
        frame_attached_to = 'gripper_link'
        frames_okay_to_collide_with = [
            'gripper_link',
            'l_gripper_finger_link',
            'r_gripper_finger_link'
        ]
        planning_scene.attachBox(
            'tray',
            0.3, 0.07, 0.01,   # size (x, y, z)
            0.05, 0, 0,        # offset from gripper_link
            frame_attached_to,
            frames_okay_to_collide_with
        )
        planning_scene.setColor('tray', 1, 0, 1)  # pink
        planning_scene.sendColors()

    rospy.sleep(1)

    # ---- Move to pose 2 with orientation constraint ----
    oc = OrientationConstraint()
    oc.header.frame_id = 'base_link'
    oc.link_name = 'wrist_roll_link'
    oc.orientation.w = 1.0
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.14
    oc.weight = 1.0

    kwargs['orientation_constraint'] = oc
    error = arm.move_to_pose(pose2, **kwargs)
    if error is not None:
        rospy.logerr('Pose 2 failed: {}'.format(error))
    else:
        rospy.loginfo('Pose 2 succeeded')

    # ---- Clean up ----
    planning_scene.removeAttachedObject('tray')
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('divider')


if __name__ == '__main__':
    main()
