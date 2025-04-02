#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from math import cos, sin, atan2, sqrt
import tf2_ros
from geometry_msgs.msg import TransformStamped

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.left_rpm_sub = self.create_subscription(Float64, 'left_motor_rpm', self.left_rpm_callback, 10)
        self.right_rpm_sub = self.create_subscription(Float64, 'right_motor_rpm', self.right_rpm_callback, 10)

        # Parameters
        self.wheel_base = 0.456  
        self.wheel_radius = 0.075  
        self.time_interval = 0.1  # Time interval (10 Hz)

        self.left_motor_rpm = 0.0
        self.right_motor_rpm = 0.0

        # Odometry message initialization
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Create a timer for updating odometry
        self.timer = self.create_timer(self.time_interval, self.update_odometry)

        # To publish TF from odom to base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def left_rpm_callback(self, msg):
        self.left_motor_rpm = msg.data

    def right_rpm_callback(self, msg):
        self.right_motor_rpm = msg.data

    def update_odometry(self):
        left_linear_vel = self.left_motor_rpm * 2.0 * 3.141592653589793 * self.wheel_radius / 60.0
        right_linear_vel = self.right_motor_rpm * 2.0 * 3.141592653589793 * self.wheel_radius / 60.0

        linear_vel = (left_linear_vel + right_linear_vel) / 2.0
        angular_vel = (right_linear_vel - left_linear_vel) / self.wheel_base

        delta_x = linear_vel * self.time_interval
        delta_theta = angular_vel * self.time_interval

        # Update orientation quaternion
        q = self.odom_msg.pose.pose.orientation
        theta = 2 * atan2(q.z, q.w)
        theta += delta_theta
        new_quat = self.yaw_to_quaternion(theta)
        self.odom_msg.pose.pose.orientation = new_quat

        # Update position
        self.odom_msg.pose.pose.position.x += delta_x * cos(theta)
        self.odom_msg.pose.pose.position.y += delta_x * sin(theta)

        # Update linear and angular velocities
        self.odom_msg.twist.twist.linear.x = linear_vel
        self.odom_msg.twist.twist.angular.z = angular_vel
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the odometry message
        self.odom_pub.publish(self.odom_msg)

        # Publish the TF from odom -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.odom_msg.pose.pose.position.x
        t.transform.translation.y = self.odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quaternion(self, yaw):
        """Convert a yaw angle (in radians) to a quaternion message."""
        quat = Quaternion()
        quat.z = sin(yaw / 2.0)
        quat.w = cos(yaw / 2.0)
        return quat

def main(args=None):
    rclpy.init(args=args)

    try:
        odom_publisher = OdomPublisher()
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

