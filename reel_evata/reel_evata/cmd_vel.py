#!/usr/bin/env python3.9
"""
cmd_vel_subscriber
-------------------
nav2'nin /cmd_vel (Twist) mesajlarini dinler, STM'e uc komut yayinlar:
  /stm/motor_power     (Int8)   - PID ile hizi hedefe oturtan motor gucu
  /stm/steering_angle  (Int16)  - direksiyon acisi, derece (-160..160)
  /stm/brake           (Bool)

Kararlilik (tekerleklerin sarsintisiz donmesi) STEERING RATE LIMITER ile
saglaniyor: nav2'den gelen angular.z ne kadar aninda/gurultulu degisirse
degissin, direksiyona giden aci saniyede en fazla `steering_max_rate_dps`
derece degisebilir. Bu; hem nav2 tarafindaki olasi ani sicramalari hem de
STM/aktuator gurultusunu tek bir noktadan, fiziksel olarak anlamli bir
parametreyle (derece/saniye) yumusatir.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int16, Bool, Float32, String
from time import time
import os
import csv
from datetime import datetime


class PID:
    """Basit PID kontrolcu; anti-windup (integral clamp) ve cikis limiti ile."""

    def __init__(self, kp, ki, kd, out_min, out_max, integral_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral_limit = integral_limit if integral_limit is not None else out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return self.out_min

        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # Hizlanmamiz gerekiyorsa (error > 0) taban gucu feedforward olarak ekle;
        # kucuk PID ciktilari olu bolgede (min_motor_power altinda) kaybolmasin.
        base_power = self.out_min if error > 0 else 0.0
        output = base_power + (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(self.out_min, min(self.out_max, output))


class EMAFilter:
    """Ustel hareketli ortalama - sadece odometri hiz tahminini yumusatmak icin."""

    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        self.value = new_value if self.value is None else (
            self.alpha * new_value + (1 - self.alpha) * self.value
        )
        return self.value


class RateLimiter:
    """
    Bir degerin birim zamanda (birim/saniye) ne kadar degisebilecegini sinirlar.
    Hem hiz rampasi (m/s^2) hem de direksiyon aci rampasi (derece/s) icin
    kullaniliyor - tek bir mekanizma, iki yerde: kod tekrari yok.
    """

    def __init__(self, max_rate_up, max_rate_down=None):
        self.max_rate_up = max_rate_up
        self.max_rate_down = max_rate_down if max_rate_down is not None else max_rate_up
        self.value = 0.0

    def update(self, target, dt):
        if dt <= 0:
            return self.value
        delta = target - self.value
        delta = max(-self.max_rate_down * dt, min(self.max_rate_up * dt, delta))
        self.value += delta
        return self.value

    def reset(self, value=0.0):
        self.value = value


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # ==================== HIZ PARAMETRELERI ====================
        self.declare_parameter('max_motor_power', 35)
        self.declare_parameter('min_motor_power', 28)
        self.declare_parameter('max_velocity', 0.8)          # m/s, nav'dan gelen hedefin tavani
        self.declare_parameter('vel_kp', 4.0)
        self.declare_parameter('vel_ki', 1.5)
        self.declare_parameter('vel_kd', 0.8)
        self.declare_parameter('max_accel', 0.45)             # m/s^2
        self.declare_parameter('max_decel', 0.9)               # m/s^2

        # ==================== FREN ====================
        self.declare_parameter('overspeed_brake_margin', 2.5)  # m/s
        self.declare_parameter('stall_velocity_threshold', 0.08)  # bu hizin alti "duruyor" sayilir (m/s)

        # ==================== DIREKSIYON ====================
        self.declare_parameter('steer_max_left', 160)
        self.declare_parameter('steer_max_right', -160)
        self.declare_parameter('angular_z_max', 0.35)
        self.declare_parameter('angular_z_min', -0.35)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('angular_deadband', 0.015)
        self.declare_parameter('steering_max_rate_dps', 400.0)

        # ==================== DONUSTE GUC ARTISI ====================
        self.declare_parameter('curvature_power_boost_enable', True)
        self.declare_parameter('curvature_free_zone', 0.25)
        self.declare_parameter('turn_power_boost_max', 2.0)

        # ==================== KALKIS / STALL ====================
        self.declare_parameter('stall_boost_rate', 2.0)
        self.declare_parameter('absolute_max_motor_power', 35)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_param_update)

        self.steering_angle_pub = self.create_publisher(Int16, '/stm/steering_angle', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)
        self.reverse_pub = self.create_publisher(Bool, '/stm/reverse_command', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)
        self.create_subscription(Int8, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)

        # ---- Durum degiskenleri ----
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.target_steering_deg = 0.0
        self.last_steering_deg = 0
        self.last_motor_power = 0
        self.last_brake = False
        self.angular_z_filtered = 0.0

        self.last_odom = None
        self.last_odom_time = None
        self.obstacle_detected = False
        self.kirmizi = False
        self.stall_timer = 0.0
        self.last_pid_time = None

        self.vel_filter = EMAFilter(0.3)
        self.velocity_ramp = RateLimiter(self.max_accel, self.max_decel)
        self.steering_ramp = RateLimiter(self.steering_max_rate_dps)
        self.velocity_pid = PID(
            kp=self.vel_kp, ki=self.vel_ki, kd=self.vel_kd,
            out_min=float(self.min_motor_power), out_max=float(self.max_motor_power),
            integral_limit=(self.max_motor_power - self.min_motor_power) * 3.0
        )

        # ==================== CSV LOGGING KURULUMU ====================
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(os.getcwd(), f"cmd_vel_log_{timestamp_str}.csv")
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # CSV Baslik satiri (Header)
        self.csv_writer.writerow([
            "Timestamp", "HedefLinX_m_s", "AnlikHiz_m_s", "MotorGucu", 
            "TurnBoost", "StallBoost", "HedefAngZ_rad_s", "HedefAci_deg", 
            "YayinAci_deg", "Fren"
        ])

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info(f'CmdVel Node baslatildi. Veriler kaydediliyor: {self.csv_filename}')

    def _load_params(self):
        gp = lambda name: self.get_parameter(name).value
        self.max_motor_power = gp('max_motor_power')
        self.min_motor_power = gp('min_motor_power')
        self.max_velocity = gp('max_velocity')
        self.vel_kp, self.vel_ki, self.vel_kd = gp('vel_kp'), gp('vel_ki'), gp('vel_kd')
        self.max_accel, self.max_decel = gp('max_accel'), gp('max_decel')

        self.overspeed_brake_margin = gp('overspeed_brake_margin')
        self.stall_velocity_threshold = gp('stall_velocity_threshold')

        self.STEER_MAX_LEFT = gp('steer_max_left')
        self.STEER_MAX_RIGHT = gp('steer_max_right')
        self.max_steer_mag = max(abs(self.STEER_MAX_LEFT), abs(self.STEER_MAX_RIGHT))

        self.angular_z_max, self.angular_z_min = gp('angular_z_max'), gp('angular_z_min')
        self.angular_z_mag = max(abs(self.angular_z_max), abs(self.angular_z_min))
        self.angular_gain = gp('angular_gain')
        self.angular_deadband = gp('angular_deadband')
        self.steering_max_rate_dps = gp('steering_max_rate_dps')

        self.curvature_power_boost_enable = gp('curvature_power_boost_enable')
        self.curvature_free_zone = gp('curvature_free_zone')
        self.turn_power_boost_max = gp('turn_power_boost_max')

        self.stall_boost_rate = gp('stall_boost_rate')
        self.absolute_max_motor_power = gp('absolute_max_motor_power')

    def _on_param_update(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
                self.get_logger().info(f'[PARAM] {p.name} = {p.value}')
            if p.name in ('angular_z_max', 'angular_z_min'):
                self.angular_z_mag = max(abs(self.angular_z_max), abs(self.angular_z_min))
            if p.name == 'steering_max_rate_dps':
                self.steering_ramp.max_rate_up = self.steering_ramp.max_rate_down = p.value
        return SetParametersResult(successful=True)

    def obstacle_callback(self, msg: Int8):
        was_detected = self.obstacle_detected
        self.obstacle_detected = (msg.data == 1)
        if self.obstacle_detected and not was_detected:
            self.get_logger().warn('[ENGEL] Engel algilandi! Arac durdurulacak.')
            self.velocity_pid.reset()

    def odom_callback(self, msg: Float32):
        now = time()
        if self.last_odom is None:
            self.last_odom, self.last_odom_time = msg.data, now
            return
        dt = now - self.last_odom_time
        if dt <= 0:
            return
        velocity_mps = (msg.data - self.last_odom) / 100.0 / dt
        self.current_velocity = self.vel_filter.update(velocity_mps)
        self.last_odom, self.last_odom_time = msg.data, now

    def sign_callback(self, msg):
        try:
            is_red = "kirmizi" in msg.data
            if is_red and not self.kirmizi:
                self.get_logger().info("Kirmizi isik algilandi.")
                self.velocity_pid.reset()
            self.kirmizi = is_red
        except Exception as e:
            self.get_logger().error(f"Levha verisi islenemedi: {e}")

    def cmd_vel_callback(self, msg: Twist):
        if self.obstacle_detected:
            self.target_velocity = 0.0
            return

        if msg.linear.x <= 0.0:
            self.target_velocity = 0.0
            self.target_steering_deg = 0.0
            self.angular_z_filtered = 0.0
            return

        self.target_velocity = min(msg.linear.x, self.max_velocity)

        raw_angular = msg.angular.z
        if abs(raw_angular) < self.angular_deadband:
            raw_angular = 0.0
        raw_angular = max(self.angular_z_min, min(self.angular_z_max, raw_angular * self.angular_gain))
        self.angular_z_filtered = raw_angular

        ratio = raw_angular / self.angular_z_mag
        target_steer = -ratio * self.max_steer_mag
        self.target_steering_deg = max(self.STEER_MAX_RIGHT, min(self.STEER_MAX_LEFT, target_steer))

    def timer_callback(self):
        if self.kirmizi:
            return

        if self.obstacle_detected:
            self.motor_power_pub.publish(Int8(data=0))
            self.brake_pub.publish(Bool(data=True))
            self.steering_angle_pub.publish(Int16(data=0))
            self.velocity_pid.reset()
            self.velocity_ramp.reset(0.0)
            self.steering_ramp.reset(0.0)
            self.stall_timer = 0.0
            return

        now = time()
        dt = 0.02 if self.last_pid_time is None else max(1e-3, now - self.last_pid_time)
        self.last_pid_time = now

        smoothed_steer = self.steering_ramp.update(self.target_steering_deg, dt)
        self.last_steering_deg = int(round(smoothed_steer))

        curvature_fraction = abs(self.last_steering_deg) / self.max_steer_mag
        if self.curvature_power_boost_enable and curvature_fraction > self.curvature_free_zone:
            curvature_boost_fraction = (curvature_fraction - self.curvature_free_zone) / (1.0 - self.curvature_free_zone)
        else:
            curvature_boost_fraction = 0.0

        ramped_target = self.velocity_ramp.update(self.target_velocity, dt)
        turn_boost = 0.0
        stall_boost = 0.0

        is_stop_command = (self.target_velocity <= 0.0)

        if is_stop_command:
            self.last_motor_power = 0
            self.last_brake = self.current_velocity > self.stall_velocity_threshold
            self.velocity_pid.reset()
            self.stall_timer = 0.0
        elif (self.current_velocity - ramped_target) >= self.overspeed_brake_margin:
            self.last_motor_power = 0
            self.last_brake = True
            self.velocity_pid.reset()
            self.stall_timer = 0.0
        else:
            error = ramped_target - self.current_velocity
            base_power = self.velocity_pid.compute(error, dt)
            turn_boost = curvature_boost_fraction * self.turn_power_boost_max

            if self.current_velocity < self.stall_velocity_threshold:
                self.stall_timer += dt
                stall_boost = self.stall_timer * self.stall_boost_rate
            else:
                self.stall_timer = 0.0

            motor_power = min(self.absolute_max_motor_power, base_power + turn_boost + stall_boost)
            self.last_motor_power = int(round(motor_power))
            self.last_brake = False

        self.motor_power_pub.publish(Int8(data=self.last_motor_power))
        self.brake_pub.publish(Bool(data=self.last_brake))
        self.steering_angle_pub.publish(Int16(data=self.last_steering_deg))

        # ---- CSV Dosyasina Yazma ----
        current_time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        fren_durumu = "EVET" if self.last_brake else "HAYIR"
        
        self.csv_writer.writerow([
            current_time_str,
            f"{self.target_velocity:.2f}",
            f"{self.current_velocity:.2f}",
            self.last_motor_power,
            f"{turn_boost:.1f}",
            f"{stall_boost:.1f}",
            f"{self.angular_z_filtered:.3f}",
            f"{self.target_steering_deg:.1f}",
            self.last_steering_deg,
            fren_durumu
        ])
        self.csv_file.flush()  # Verilerin aninda diske islenmesini sagla

        # Terminal ekranina yazdirma
        self.get_logger().info(
            f'HedefLinX: {self.target_velocity:5.2f} | '
            f'AnlikHiz: {self.current_velocity:5.2f} | '
            f'Motor: {self.last_motor_power:3d} (turn={turn_boost:.1f}, stall={stall_boost:.1f}) | '
            f'HedefAngZ: {self.angular_z_filtered:+.3f} | '
            f'HedefAci: {self.target_steering_deg:+6.1f} | '
            f'YayinAci: {self.last_steering_deg:4d} | '
            f'Fren: {fren_durumu}'
        )

    def destroy_node(self):
        # Program kapatildiginda dosyayi guvenli bir sekilde kapat
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info('CSV dosyasi guvenle kapatildi.')
        super().destroy_node()


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