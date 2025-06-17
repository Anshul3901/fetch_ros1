#! /usr/bin/env python

import rospy
import copy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Base(object):
    def __init__(self):
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self._latest_odom = None

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def wait_for_odom(self):
        while self._latest_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def move(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._pub.publish(twist)

    def stop(self):
        self.move(0, 0)

    def get_position(self, odom):
        return odom.pose.pose.position

    def get_yaw(self, odom):
        orientation_q = odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def euclidean_distance(self, p1, p2):
        return math.sqrt(
            (p1.x - p2.x)**2 + (p1.y - p2.y)**2
        )

    def go_forward(self, distance, speed=0.1):
        self.wait_for_odom()
        start = copy.deepcopy(self._latest_odom)
        start_pos = self.get_position(start)

        rate = rospy.Rate(10)
        direction = -1 if distance < 0 else 1
        distance = abs(distance)

        while not rospy.is_shutdown():
            current_pos = self.get_position(self._latest_odom)
            traveled = self.euclidean_distance(start_pos, current_pos)

            if traveled >= distance:
                break

            self.move(direction * speed, 0)
            rate.sleep()

        self.stop()

    def turn(self, angular_distance, speed=0.5):
        self.wait_for_odom()
        start = copy.deepcopy(self._latest_odom)
        start_yaw = self.get_yaw(start)
        goal_yaw = (start_yaw + angular_distance) % (2 * math.pi)

        direction = -1 if angular_distance < 0 else 1
        angular_distance = abs(angular_distance)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_yaw = self.get_yaw(self._latest_odom)
            diff = (goal_yaw - current_yaw) % (2 * math.pi)

            if direction == 1 and diff < 0.01:
                break
            elif direction == -1 and (2 * math.pi - diff) < 0.01:
                break

            self.move(0, direction * speed)
            rate.sleep()

        self.stop()
