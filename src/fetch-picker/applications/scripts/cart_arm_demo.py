#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from robot_api import Arm  # Your Arm class with move_to_pose() and cancel_all_goals()

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('cart_arm_demo')
    wait_for_time()

    arm = Arm()

    def shutdown():
        rospy.loginfo("Shutting down. Cancelling all arm goals.")
        arm.cancel_all_goals()

    rospy.on_shutdown(shutdown)

    # Define target poses
    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))

    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1

    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2

    gripper_poses = [ps1, ps2]

    i = 0
    rate = rospy.Rate(0.05)  # Max 1 cycle every 20s, but we also use rospy.sleep

    while not rospy.is_shutdown():
        target_pose = gripper_poses[i % 2]
        rospy.loginfo(f"Moving to pose {i % 2 + 1}...")

        error = arm.move_to_pose(target_pose)
        if error is not None:
            rospy.logerr(f"Move to pose {i % 2 + 1} failed: {error}")
        else:
            rospy.loginfo(f"Reached pose {i % 2 + 1}")

        rospy.sleep(1.0)  # Let the arm settle before next move
        i += 1
        rate.sleep()

if __name__ == '__main__':
    main()
