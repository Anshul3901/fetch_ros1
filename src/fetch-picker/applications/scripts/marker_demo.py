#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
        type=Marker.TEXT_VIEW_FACING,
        id=0,
        lifetime=rospy.Duration(1.5),  # Change to rospy.Duration(0) for infinite
        pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
        scale=Vector3(0.2, 0.2, 0.2),  # Increase size
        header=Header(frame_id='base_link'),
        color=ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red and fully opaque
        text=text
    )
    marker_publisher.publish(marker)

def main():
    rospy.init_node('marker_demo')
    wait_for_time()
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)  # Wait for subscribers
    show_text_in_rviz(marker_pub, 'Hello RViz!')
    rospy.sleep(2.0)  # Allow time for RViz to show marker

if __name__ == '__main__':
    main()
