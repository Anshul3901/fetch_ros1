#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import math

class NavPath:
    def __init__(self, marker_publisher):
        self._path = []
        self._last_position = None
        self._marker_pub = marker_publisher
        self._threshold = 0.1  # meters

    def callback(self, msg):
        pos = msg.pose.pose.position
        current = Point(pos.x, pos.y, pos.z)

        if self._last_position is None or self._distance(self._last_position, current) > self._threshold:
            self._path.append(current)
            self._last_position = current
            self.publish_path_marker()

    def _distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

    def publish_path_marker(self):
        marker = Marker()
        marker.header = Header(frame_id='odom')  # or 'base_link' depending on your TF
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.id = 1
        marker.scale.x = 0.02  # line width
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # blue
        marker.points = self._path
        marker.lifetime = rospy.Duration(0)
        self._marker_pub.publish(marker)

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('path_marker')
    wait_for_time()
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.5)
    path = NavPath(marker_pub)
    rospy.Subscriber('odom', Odometry, path.callback)
    rospy.spin()

if __name__ == '__main__':
    main()
