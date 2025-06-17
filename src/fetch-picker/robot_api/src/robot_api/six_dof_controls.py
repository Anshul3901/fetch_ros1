from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarkerControl
import copy

def make_6dof_controls():
    controls = []

    for axis in ['x', 'y', 'z']:
        control = InteractiveMarkerControl()
        control.name = f"move_{axis}"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls.append(copy.deepcopy(control))

        control.name = f"rotate_{axis}"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(copy.deepcopy(control))

    return controls
