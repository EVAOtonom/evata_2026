#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class WheelEncoderOdometry(Node):
    def __init__(self):
        super().__init__('wheel_encoder_odometry')

        self.declare_parameter('encoder_topic', '/stm/read_odometer')
        self.declare_parameter('odom_topic', '/teker')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 20.0)

        self.encoder_topic = self.get_parameter('encoder_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        publish_rate = self.get_parameter('publish_rate').value

        self.x = 0.0
        self.last_encoder_cm = None
        self.last_time = self.get_clock().now()
        self.linear_x = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.create_subscription(
            Float32,
            self.encoder_topic,
            self.encoder_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_odometry
        )

        self.get_logger().info(
            f"Wheel encoder odometry started: {self.encoder_topic} -> {self.odom_topic}"
        )

    def encoder_callback(self, msg: Float32):
        current_encoder_cm = msg.data
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if self.last_encoder_cm is None or dt <= 0.0:
            self.last_encoder_cm = current_encoder_cm
            self.last_time = now
            self.linear_x = 0.0
            return

        delta_cm = current_encoder_cm - self.last_encoder_cm
        delta_m = delta_cm / 100.0

        self.linear_x = delta_m / dt
        self.x += delta_m

        self.last_encoder_cm = current_encoder_cm
        self.last_time = now

    def publish_odometry(self):
        now = self.get_clock().now()

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        odom_msg.twist.twist.linear.x = self.linear_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        odom_msg.pose.covariance = [
            0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0
        ]

        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0
        ]

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
