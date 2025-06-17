#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException

def main():
    rospy.init_node('ee_pose_demo')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(1.0)  # 1 Hz

    target_frame = 'base_link'
    source_frame = 'gripper_link'  # You can change this to test others like wrist_roll_link

    rospy.loginfo(f"Starting ee_pose_demo. Listening for transforms from '{target_frame}' to '{source_frame}'.")

    # Wait for TF to be ready
    rospy.sleep(1.0)

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            t = trans.transform.translation
            r = trans.transform.rotation
            rospy.loginfo("Position: (%.4f, %.4f, %.4f) Orientation: (%.4f, %.4f, %.4f, %.4f)",
                          t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"{target_frame} -> {source_frame} transform not available yet: {str(e)}")

        rate.sleep()


if __name__ == '__main__':
    main()
