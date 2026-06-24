#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, Float32, String
import math
from time import time


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        self.steering_angle_pub = self.create_publisher(Int8, '/stm/steering_angle', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)
        self.reverse_pub = self.create_publisher(Bool, '/stm/reverse_command', 10)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)
        self.obstacle_sub = self.create_subscription(Int8, '/obstacle_detected', self.obstacle_callback, 10)
        self.sign_sub = self.create_subscription(String, '/detected_signs', self.sign_callback, 10)

        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_odom = None
        self.last_odom_time = None

        self.obstacle_detected = False
        self.kirmizi = False

        self.max_motor_power = 100
        self.last_motor_power = 0
        self.last_brake = False

        self.WHEELBASE = 1.75
        self.MAX_LEFT_DEG = 40
        self.MAX_RIGHT_DEG = -43

        self.steering_gain = 1.1
        self.min_steering_speed = 0.20

        self.target_steering_deg = 0.0
        self.filtered_steering_deg = 0.0
        self.last_steering_deg = 0

        self.STEERING_DEADBAND_DEG = 1.0

        # 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('CmdVel Node başlatıldı.')

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def obstacle_callback(self, msg: Int8):
        self.obstacle_detected = (msg.data == 1)
        if self.obstacle_detected:
            self.target_velocity = 0.0
            self.last_motor_power = 0
            self.last_brake = True
            self.target_steering_deg = 0.0

    def odom_callback(self, msg: Float32):
        current_odom = msg.data
        current_time = time()

        if self.last_odom is None or self.last_odom_time is None:
            self.last_odom = current_odom
            self.last_odom_time = current_time
            return

        delta_t = current_time - self.last_odom_time
        if delta_t <= 0:
            return

        distance_m = (current_odom - self.last_odom) / 100.0
        self.current_velocity = distance_m / delta_t

        self.last_odom = current_odom
        self.last_odom_time = current_time

    def sign_callback(self, msg: String):
        if "kirmizi" in msg.data:
            self.kirmizi = True
            self.target_velocity = 0.0
            self.last_motor_power = 0
            self.last_brake = True
            self.target_steering_deg = 0.0
        else:
            self.kirmizi = False

    def calculate_steering_from_cmd(self, linear_x, angular_z):
        if abs(angular_z) < 0.0015:
            return 0.0

        v = max(abs(linear_x), self.min_steering_speed)

        steering_rad = math.atan((self.WHEELBASE * angular_z) / v)
        steering_deg = math.degrees(steering_rad) * self.steering_gain

        # STM yönü ters
        steering_deg *= -1.0

        steering_deg = self.clamp(
            steering_deg,
            self.MAX_RIGHT_DEG,
            self.MAX_LEFT_DEG
        )

        if abs(steering_deg) < self.STEERING_DEADBAND_DEG:
            steering_deg = 0.0

        return steering_deg

    def update_filtered_steering(self):
        error = self.target_steering_deg - self.filtered_steering_deg

        # Daha dengeli: düzde sakin, dönüşte yeterli hızlı
        if abs(error) > 25:
            max_step = 2.2
            alpha = 0.45
        elif abs(error) > 12:
            max_step = 1.6
            alpha = 0.35
        else:
            max_step = 0.8
            alpha = 0.20

        desired = alpha * self.target_steering_deg + (1.0 - alpha) * self.filtered_steering_deg

        delta = desired - self.filtered_steering_deg
        delta = self.clamp(delta, -max_step, max_step)

        self.filtered_steering_deg += delta

        if abs(self.filtered_steering_deg) < self.STEERING_DEADBAND_DEG:
            self.filtered_steering_deg = 0.0

        self.filtered_steering_deg = self.clamp(
            self.filtered_steering_deg,
            self.MAX_RIGHT_DEG,
            self.MAX_LEFT_DEG
        )

        self.last_steering_deg = int(round(self.filtered_steering_deg))

    def cmd_vel_callback(self, msg: Twist):
        if self.obstacle_detected or self.kirmizi:
            self.target_velocity = 0.0
            self.last_motor_power = 0
            self.last_brake = True
            self.target_steering_deg = 0.0
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if linear_x > 0.01:
            self.target_velocity = 0.6
        else:
            self.target_velocity = 0.0
            self.last_motor_power = 0
            self.last_brake = True
            self.target_steering_deg = 0.0
            return

        self.target_steering_deg = self.calculate_steering_from_cmd(
            linear_x,
            angular_z
        )

        speed_error = abs(self.target_velocity - self.current_velocity)
        brake = False

        if self.current_velocity >= self.target_velocity + 1.0:
            motor_power = 0
            brake = True
        else:
            if speed_error > 0.7:
                motor_power = self.max_motor_power
            elif speed_error > 0.5:
                motor_power = int(self.max_motor_power * 0.31)
            elif speed_error > 0.3:
                motor_power = int(self.max_motor_power * 0.30)
            elif speed_error > 0.2:
                motor_power = int(self.max_motor_power * 0.29)
            elif speed_error > 0.1:
                motor_power = int(self.max_motor_power * 0.29)
            else:
                motor_power = 0

            if abs(self.last_steering_deg) >= 25:
                motor_power += 2

        self.last_motor_power = self.clamp(int(motor_power), 0, self.max_motor_power)
        self.last_brake = brake

    def timer_callback(self):
        if self.kirmizi or self.obstacle_detected:
            self.motor_power_pub.publish(Int8(data=0))
            self.brake_pub.publish(Bool(data=True))
            self.steering_angle_pub.publish(Int8(data=0))
            return

        self.update_filtered_steering()

        self.motor_power_pub.publish(Int8(data=self.last_motor_power))
        self.brake_pub.publish(Bool(data=self.last_brake))
        self.steering_angle_pub.publish(Int8(data=self.last_steering_deg))

        self.get_logger().info(
            f'Hedef Direksiyon: {self.target_steering_deg:.1f} | '
            f'Yayın Direksiyon: {self.last_steering_deg} | '
            f'Hız: {self.current_velocity:.2f} | '
            f'Motor: {self.last_motor_power}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()