#!/usr/bin/env python

import rospy
import math
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from robot_api import Base
import threading

def make_marker(name, description, position, color):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color = color

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append(marker)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = name
    int_marker.description = description
    int_marker.pose.position = position
    int_marker.pose.orientation.w = 1
    int_marker.controls.append(control)

    return int_marker

def make_color(r, g, b, a=1.0):
    return ColorRGBA(r, g, b, a)

class MarkerDriveInterface(object):
    def __init__(self):
        self._base = Base()
        self._server = InteractiveMarkerServer("base_control")

        # Drive Forward
        self._server.insert(
            make_marker("forward", "Drive Forward", Point(0.7, 0, 0), make_color(0.0, 1.0, 0.0)),
            self.handle_click)

        # Turn Left
        self._server.insert(
            make_marker("left", "Turn Left", Point(0.3, 0.5, 0), make_color(0.0, 0.5, 1.0)),
            self.handle_click)

        # Turn Right
        self._server.insert(
            make_marker("right", "Turn Right", Point(0.3, -0.5, 0), make_color(1.0, 0.5, 0.0)),
            self.handle_click)

        # Stop
        self._server.insert(
            make_marker("stop", "Stop", Point(-0.3, 0, 0), make_color(1.0, 0.0, 0.0)),
            self.handle_click)

        self._server.applyChanges()

    def handle_click(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return

        name = feedback.marker_name
        rospy.loginfo(f"Clicked marker: {name}")

        if name == "forward":
            threading.Thread(target=self._base.go_forward, args=(0.5,)).start()
        elif name == "left":
            threading.Thread(target=self._base.turn, args=(math.radians(30),)).start()
        elif name == "right":
            threading.Thread(target=self._base.turn, args=(math.radians(-30),)).start()
        elif name == "stop":
            self._base.stop()


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('interactive_base_control')
    wait_for_time()
    MarkerDriveInterface()
    rospy.spin()

if __name__ == '__main__':
    main()
