#!/usr/bin/env python3

import rospy
import copy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, MenuEntry
from geometry_msgs.msg import PoseStamped
from robot_api import Arm, Gripper
from robot_api.gripper_marker_utils import make_gripper_meshes, update_marker_color
from robot_api.six_dof_controls import make_6dof_controls

class GripperTeleop:
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()
        self._menu_handler.insert("Go to Pose", callback=self.goto_pose)
        self._menu_handler.insert("Open Gripper", callback=self.open_gripper)
        self._menu_handler.insert("Close Gripper", callback=self.close_gripper)

    def start(self):
        marker = InteractiveMarker()
        marker.name = "gripper_control"
        marker.header.frame_id = "base_link"
        marker.scale = 0.2
        marker.pose.position.z = 1.0  # example default height

        control = InteractiveMarkerControl()
        control.name = "gripper_meshes"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MENU
        meshes = make_gripper_meshes()
        control.markers.extend(meshes)
        marker.controls.append(control)

        marker.controls.extend(make_6dof_controls())

        self._im_server.insert(marker, self.handle_feedback)
        self._menu_handler.apply(self._im_server, marker.name)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose_stamped = PoseStamped()
            pose_stamped.header = feedback.header
            pose_stamped.pose = feedback.pose
            result = self._arm.compute_ik(pose_stamped)
            color = (0.0, 1.0, 0.0, 1.0) if result.success else (1.0, 0.0, 0.0, 1.0)
            marker = self._im_server.get(feedback.marker_name)
            for c in marker.controls:
                update_marker_color(c.markers, color)
            self._im_server.insert(marker, self.handle_feedback)
            self._im_server.applyChanges()

    def goto_pose(self, feedback):
        pose = feedback.pose
        pose_stamped = PoseStamped()
        pose_stamped.header = feedback.header
        pose_stamped.pose = pose
        self._arm.move_to_pose(pose_stamped)

    def open_gripper(self, feedback):
        self._gripper.open()

    def close_gripper(self, feedback):
        self._gripper.close()

def main():
    rospy.init_node('gripper_teleop')
    from robot_api import Arm, Gripper  # assumes robot_api has correct bindings
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    arm = Arm()
    gripper = Gripper()
    teleop = GripperTeleop(arm, gripper, im_server)
    teleop.start()
    rospy.spin()

if __name__ == '__main__':
    main()
