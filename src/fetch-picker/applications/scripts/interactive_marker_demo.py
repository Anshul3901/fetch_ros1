#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

def handle_viz_input(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(f"{feedback.marker_name} was clicked.")
    else:
        rospy.loginfo("Cannot handle this InteractiveMarker event")

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node("interactive_marker_demo")
    wait_for_time()

    # Create the interactive marker server
    server = InteractiveMarkerServer("simple_marker")

    # Create the interactive marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple Click Control"
    int_marker.pose.position.x = 1.0
    int_marker.pose.orientation.w = 1.0

    # Create a teal cube marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # Create an interactive control and add the marker to it
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    # Register the marker with the server
    server.insert(int_marker, handle_viz_input)
    server.applyChanges()

    rospy.loginfo("Interactive marker ready. Click on it in RViz!")
    rospy.spin()

if __name__ == "__main__":
    main()
