#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class WheelEncoderOdometry(Node):

    def __init__(self):
        super().__init__('wheel_encoder_odometry')

        # Parametreler
        self.declare_parameter(
            'encoder_topic',
            '/stm/read_odometer'
        )

        self.declare_parameter(
            'odom_topic',
            '/teker'
        )

        self.declare_parameter(
            'odom_frame',
            'odom'
        )

        self.declare_parameter(
            'base_frame',
            'base_footprint'
        )

        self.declare_parameter(
            'max_delta_cm',
            200.0
        )

        self.encoder_topic = self.get_parameter(
            'encoder_topic'
        ).value

        self.odom_topic = self.get_parameter(
            'odom_topic'
        ).value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.max_delta_cm = float(
            self.get_parameter('max_delta_cm').value
        )

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            10
        )

        # Subscriber
        self.encoder_sub = self.create_subscription(
            Float32,
            self.encoder_topic,
            self.encoder_callback,
            10
        )

        # Değişkenler
        self.last_encoder_cm = None
        self.x_cm = 0.0

        self.get_logger().info(
            '\nTeker enkoder pose node başlatıldı.\n'
            f'Encoder input  : {self.encoder_topic}\n'
            f'Odometry output: {self.odom_topic}\n'
            'Kullanılan ölçüm: pose.position.x\n'
            'Twist hızları   : 0.0'
        )

    def encoder_callback(self, msg: Float32):
        current_encoder_cm = float(msg.data)
        current_time = self.get_clock().now()

        # İlk mesaj yalnızca referans olarak kaydedilir.
        if self.last_encoder_cm is None:
            self.last_encoder_cm = current_encoder_cm
            self.publish_wheel_odometry(current_time)
            return

        delta_cm = current_encoder_cm - self.last_encoder_cm
        self.last_encoder_cm = current_encoder_cm

        # Enkoder reseti veya hatalı sıçrama.
        if abs(delta_cm) > self.max_delta_cm:
            self.get_logger().warning(
                f'Enkoder sıçraması reddedildi: '
                f'{delta_cm:.2f} cm'
            )
            self.publish_wheel_odometry(current_time)
            return

        # Geçen seneki sistem gibi toplam konum hesaplanır.
        self.x_cm += delta_cm

        self.publish_wheel_odometry(current_time)

    def publish_wheel_odometry(self, stamp):
        odom_msg = Odometry()

        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Santimetre -> metre
        odom_msg.pose.pose.position.x = self.x_cm / 100.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Geçen seneki sistem gibi hız bilgisi yayınlanmaz.
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # EKF yalnızca pose.position.x kullanacak.
        odom_msg.pose.covariance = [
            0.02, 0.0,       0.0,       0.0,       0.0,       0.0,
            0.0,  1000000.0, 0.0,       0.0,       0.0,       0.0,
            0.0,  0.0,       1000000.0, 0.0,       0.0,       0.0,
            0.0,  0.0,       0.0,       1000000.0, 0.0,       0.0,
            0.0,  0.0,       0.0,       0.0,       1000000.0, 0.0,
            0.0,  0.0,       0.0,       0.0,       0.0,       1000000.0
        ]

        # Twist alanları kullanılmayacak.
        odom_msg.twist.covariance = [
            1000000.0, 0.0,       0.0,       0.0,       0.0,       0.0,
            0.0,       1000000.0, 0.0,       0.0,       0.0,       0.0,
            0.0,       0.0,       1000000.0, 0.0,       0.0,       0.0,
            0.0,       0.0,       0.0,       1000000.0, 0.0,       0.0,
            0.0,       0.0,       0.0,       0.0,       1000000.0, 0.0,
            0.0,       0.0,       0.0,       0.0,       0.0,       1000000.0
        ]

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    node = WheelEncoderOdometry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
