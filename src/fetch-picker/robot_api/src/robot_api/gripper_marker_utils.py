from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import copy

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def make_gripper_meshes(color=(0.5, 0.5, 0.5, 1.0)):
    markers = []

    def mesh_marker(mesh, frame_id='gripper_marker', pose=Pose()):
        m = Marker()
        m.header.frame_id = frame_id
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = mesh
        m.action = Marker.ADD
        m.pose = pose
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    # Main gripper
    markers.append(mesh_marker(GRIPPER_MESH))

    # Left finger (adjust pose)
    left_pose = Pose()
    left_pose.position.y = 0.05
    markers.append(mesh_marker(L_FINGER_MESH, pose=left_pose))

    # Right finger
    right_pose = Pose()
    right_pose.position.y = -0.05
    markers.append(mesh_marker(R_FINGER_MESH, pose=right_pose))

    return markers

def update_marker_color(marker_list, color):
    for m in marker_list:
        m.color.r, m.color.g, m.color.b, m.color.a = color
