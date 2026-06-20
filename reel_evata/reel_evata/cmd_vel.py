#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, Float32
import math
from time import time
from std_msgs.msg import String
import json


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

        self.current_velocity = 0.0  # m/s
        self.target_velocity = 0.0   # m/s
        self.last_odom = None
        self.last_odom_time = None

        self.obstacle_detected = False

        self.steering_gain = 1
        self.max_motor_power = 10
        self.max_velocity = 1.5

        # Son değerleri saklayacak değişkenler
        self.last_motor_power = 0
        self.last_brake = False
        self.last_steering_deg = 0

        self.kirmizi = False

        # Timer ile sabit frekansta yayın (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('CmdVel Node başlatıldı.')

    def obstacle_callback(self, msg: Int8):
        self.obstacle_detected = (msg.data == 1)
        if self.obstacle_detected:
            self.get_logger().warn('[ENGEL] Engel algılandı! Araç durdurulacak.')

    def odom_callback(self, msg: Float32):
        current_odom = msg.data
        current_time = time()

        if self.last_odom is None or self.last_odom_time is None:
            self.last_odom = current_odom
            self.last_odom_time = current_time
            return

        delta_s_cm = current_odom - self.last_odom
        delta_t = current_time - self.last_odom_time

        if delta_t <= 0:
            return

        distance_m = delta_s_cm / 100.0
        velocity_mps = distance_m / delta_t

        self.current_velocity = velocity_mps
        self.last_odom = current_odom
        self.last_odom_time = current_time

    def sign_callback(self, msg):
        try:
            detected = (msg.data)
            if "kirmizi" in detected:
                self.get_logger().info("Kırmızı ışık algılandı.")
                self.kirmizi = True
            else:
                self.kirmizi = False
        except Exception as e:
            self.get_logger().error(f"Levha verisi işlenemedi: {e}")
        
    def cmd_vel_callback(self, msg: Twist):
        if self.obstacle_detected:
            self.target_velocity = 0.0
            self.last_motor_power = 0
            self.last_brake = True
            self.last_steering_deg = 0
            return
            
        linear_x = msg.linear.x
        if linear_x > 0:
            self.target_velocity = 0.6 
        else:
            self.target_velocity = 0.0
            self.last_motor_power = 0
            self.last_brake = True
            return
        angular_z = msg.angular.z * 1.25
        is_reverse = self.target_velocity < 0

        WHEELBASE = 1.75
        MAX_LEFT_DEG = 30
        MAX_RIGHT_DEG = -33

        if abs(self.target_velocity) > 0.01:
            steering_rad = math.atan((WHEELBASE * angular_z) / self.target_velocity)
        else:
            steering_rad = 0.0

        angle_deg = math.degrees(steering_rad) * self.steering_gain
        steering_deg = max(MAX_RIGHT_DEG, min(MAX_LEFT_DEG, angle_deg)) * -1
        steering_deg = int(steering_deg)
        self.last_steering_deg = steering_deg

        # === Motor Gücü Hesabı ===
        speed_error = abs(self.target_velocity - self.current_velocity)
        brake = False

        if is_reverse:
            motor_power = 3
            brake = False
        elif self.current_velocity >= self.target_velocity + 1.0:
            motor_power = 0
            brake = True
        else:
            if speed_error > 0.7:
                motor_power = self.max_motor_power
            elif speed_error > 0.5:
                motor_power = int(self.max_motor_power * 0.8)
            elif speed_error > 0.3:
                motor_power = int(self.max_motor_power * 0.6)
            elif speed_error > 0.2:
                motor_power = int(self.max_motor_power * 0.4)
            elif speed_error > 0.1:
                motor_power = int(self.max_motor_power * 0.2)
            else:
                motor_power = 0

            # Keskin direksiyon açıları için +5 güç ver
            if abs(self.last_steering_deg) >= 25:
                motor_power += 2


            # Eğer hedef hız sıfırsa tamamen dur
            if self.target_velocity == 0.0:
                motor_power = 0
                brake = False
        motor_power = max(0, min(self.max_motor_power, int(motor_power)))
        self.last_motor_power = motor_power
        self.last_brake = brake

    def timer_callback(self):
        # Engel varsa her zaman fren uygula
        if self.kirmizi:
            return
        if self.obstacle_detected:
            self.motor_power_pub.publish(Int8(data=0))
            self.brake_pub.publish(Bool(data=True))
            self.steering_angle_pub.publish(Int8(data=0))
            return

        self.motor_power_pub.publish(Int8(data=self.last_motor_power))
        self.brake_pub.publish(Bool(data=self.last_brake))
        self.steering_angle_pub.publish(Int8(data=self.last_steering_deg))

        self.get_logger().info(
            f'[YAYIN] Hedef Hız: {self.target_velocity:.2f} m/s | '
            f'Anlık Hız: {self.current_velocity:.2f} m/s | '
            f'Motor Gücü: {self.last_motor_power} | Fren: {self.last_brake} | '
            f'Direksiyon Açısı: {self.last_steering_deg}'
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