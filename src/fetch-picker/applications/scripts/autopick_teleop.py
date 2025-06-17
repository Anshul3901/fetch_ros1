#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import *
from robot_api import Arm, Gripper
from robot_api.gripper_marker_utils import make_gripper_meshes, update_marker_color
from robot_api.six_dof_controls import make_6dof_controls

def to_matrix(pose):
    """Convert geometry_msgs/Pose to 4x4 transformation matrix."""
    T = tft.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T

def from_matrix(T):
    """Convert 4x4 transformation matrix to geometry_msgs/Pose."""
    q = tft.quaternion_from_matrix(T)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = T[0:3, 3]
    pose.orientation = Quaternion(*q)
    return pose

class AutoPickTeleop:
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._marker_name = "target_object"

    def start(self):
        marker = InteractiveMarker()
        marker.header.frame_id = "base_link"
        marker.name = self._marker_name
        marker.scale = 0.15
        marker.pose.position.x = 0.8
        marker.pose.position.z = 0.75

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.name = "object"
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = cube.scale.y = cube.scale.z = 0.05
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = (0.5, 0.5, 0.5, 1)
        control.markers.append(cube)
        marker.controls.append(control)

        marker.controls.extend(make_6dof_controls())
        self._im_server.insert(marker, self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return

        object_pose = feedback.pose
        T_obj = to_matrix(object_pose)

        # Compute relative transforms
        T_pre = np.eye(4)
        T_pre[0, 3] = -0.1  # 10 cm back

        T_grasp = np.eye(4)  # same as object

        T_lift = np.eye(4)
        T_lift[2, 3] = 0.1  # 10 cm up

        T_pre_grasp = np.dot(T_obj, T_pre)
        T_grasp_pose = T_obj
        T_lift_pose = np.dot(T_obj, T_lift)

        poses = {
            "pregrasp": from_matrix(T_pre_grasp),
            "grasp": from_matrix(T_grasp_pose),
            "lift": from_matrix(T_lift_pose)
        }

        reachability = []
        for name, pose in poses.items():
            ps = PoseStamped()
            ps.header.frame_id = "base_link"
            ps.pose = pose
            result = self._arm.compute_ik(ps)
            reachability.append(result.success)

        # Set cube color based on all reachable
        marker = self._im_server.get(self._marker_name)
        for c in marker.controls:
            for m in c.markers:
                if all(reachability):
                    m.color.r, m.color.g = 0.0, 1.0  # green
                else:
                    m.color.r, m.color.g = 1.0, 0.0  # red
                m.color.b, m.color.a = 0.0, 1.0
        self._im_server.insert(marker, self.handle_feedback)
        self._im_server.applyChanges()

def main():
    rospy.init_node("autopick_teleop")
    im_server = InteractiveMarkerServer("auto_pick_im_server", q_size=2)
    arm = Arm()
    gripper = Gripper()
    auto_pick = AutoPickTeleop(arm, gripper, im_server)
    auto_pick.start()
    rospy.spin()

if __name__ == '__main__':
    main()
