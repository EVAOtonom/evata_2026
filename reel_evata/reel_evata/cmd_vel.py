#!/usr/bin/env python3.9

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int16, Bool, Float32
import math
from time import time
from collections import deque
from std_msgs.msg import String


class PID:
    """Basit PID kontrolcü, anti-windup (integral clamp) ve çıkış limiti ile."""

    def __init__(self, kp, ki, kd, out_min, out_max, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
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

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(self.out_min, min(self.out_max, output))


class EMAFilter:
    """Üstel hareketli ortalama (low-pass) filtre - ani gürültü/sıçramaları yumuşatır."""

    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

    def reset(self, value=None):
        self.value = value


class RateLimiter:
    """
    Bir değerin zamana göre (birim/saniye cinsinden) ne kadar hızlı değişebileceğini sınırlar.
    Hem hız setpoint'ini (ivme limiti) hem direksiyon komutunu yumuşatmak için kullanılır.
    Yükselme ve düşme için ayrı oranlar tanımlanabilir (örn. yavaşlama, hızlanmadan daha hızlı olabilir).
    """

    def __init__(self, max_rate_up, max_rate_down=None):
        self.max_rate_up = max_rate_up
        self.max_rate_down = max_rate_down if max_rate_down is not None else max_rate_up
        self.value = 0.0

    def update(self, target, dt):
        if dt <= 0:
            return self.value
        delta = target - self.value
        max_delta_up = self.max_rate_up * dt
        max_delta_down = self.max_rate_down * dt
        if delta > max_delta_up:
            delta = max_delta_up
        elif delta < -max_delta_down:
            delta = -max_delta_down
        self.value += delta
        return self.value

    def reset(self, value=0.0):
        self.value = value


class CurvaturePredictor:
    """
    Son birkaç direksiyon/eğrilik örneğinin trendine (eğim) bakarak yakın gelecekteki
    eğriliği tahmin eder. Path planner (Hybrid A* / Dijkstra) çıktısından türeyen
    cmd_vel komutları zaman içinde geldiği için, "dönüş keskinleşiyor" trendini
    yakalayıp hız düşürmeyi ÖNCEDEN başlatmamızı sağlar (sadece anlık tepki yerine).
    """

    def __init__(self, window_size=6, lookahead_time=0.35):
        self.buffer = deque(maxlen=window_size)  # (timestamp, curvature_fraction)
        self.lookahead_time = lookahead_time

    def update(self, curvature_fraction, timestamp):
        self.buffer.append((timestamp, curvature_fraction))

        if len(self.buffer) < 2:
            return curvature_fraction

        t0, c0 = self.buffer[0]
        t1, c1 = self.buffer[-1]
        dt = t1 - t0
        if dt <= 0:
            return curvature_fraction

        slope = (c1 - c0) / dt  # eğrilik değişim hızı (1/s)
        predicted = c1 + slope * self.lookahead_time
        predicted = max(0.0, min(1.0, predicted))

        # Trend artıyorsa (dönüş keskinleşiyor) tahmini değeri, azalıyorsa
        # güncel değeri baz al - erken yavaşla ama erken hızlanma (henüz düzelmeden).
        return max(curvature_fraction, predicted)


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # ==================== PARAMETRELER (tune edilebilir) ====================
        # Motor sadece 28-33 aralığında anlamlı çalışıyor (28 altı dönmüyor, 33 üstü gereksiz/riskli)
        self.declare_parameter('max_motor_power', 33)
        self.declare_parameter('min_motor_power', 30)
        self.declare_parameter('max_velocity', 1.5)

        # Hedef hız navigasyondan gelen linear.x'e bağlı: target = linear_x * velocity_scale
        # Genel hızı arttırıp azaltmak için sadece bu çarpanı değiştirmen yeterli.
        self.declare_parameter('velocity_scale', 1.2)

        # Hız PID katsayıları - çıkış aralığı dar (28-33) olduğu için küçük kazançlar yeterli
        self.declare_parameter('vel_kp', 10.0)
        self.declare_parameter('vel_ki', 6.0)
        self.declare_parameter('vel_kd', 0.5)

        # Hız setpoint rampası (ivme limiti, m/s^2) - "gaza aniden basma" hissini ortadan kaldırır.
        # max_decel, max_accel'den büyük tutuldu: dönüşe girerken hızlı yavaşlayabilsin,
        # ama hızlanma her zaman kontrollü/yumuşak olsun.
        self.declare_parameter('max_accel', 0.45)   # m/s^2
        self.declare_parameter('max_decel', 0.9)    # m/s^2

        # Direksiyon limitleri
        self.declare_parameter('steer_max_left', 160)
        self.declare_parameter('steer_max_right', -160)
        self.declare_parameter('wheelbase', 1.75)
        self.declare_parameter('max_left_deg', 30.0)
        self.declare_parameter('max_right_deg', -33.0)

        # Filtre parametreleri
        self.declare_parameter('velocity_filter_alpha', 0.3)   # odometri hız filtresi
        self.declare_parameter('angular_filter_alpha', 0.35)   # gelen angular_z filtresi
        self.declare_parameter('steering_filter_alpha', 0.35)  # çıkış direksiyon filtresi
        self.declare_parameter('steering_rate_max', 400.0)     # direksiyon birimi/saniye (yumuşak dönüş)

        # === Eğrilik-bazlı hız sınırlama (curvature-based speed limiting) ===
        # Dönüş ne kadar keskinse (direksiyon açısı max limite ne kadar yakınsa),
        # hedef hız o kadar düşürülür - "tekeri kırıp gaza basma" davranışını engeller.
        self.declare_parameter('curvature_speed_limit_enable', True)
        self.declare_parameter('curvature_min_speed_fraction', 0.35)  # tam kilitte izin verilen hız oranı
        self.declare_parameter('curvature_free_zone', 0.15)           # bu oranın altında hiç yavaşlama yok

        # Öngörü (prediction) parametreleri
        self.declare_parameter('curvature_lookahead_time', 0.35)  # saniye, ne kadar ileriye bakılsın
        self.declare_parameter('curvature_window_size', 6)         # kaç örnek üzerinden trend hesaplansın

        self._load_params()

        # Direksiyon Int16 (-160..160 aralığı Int8'e sığmıyor)
        self.steering_angle_pub = self.create_publisher(Int16, '/stm/steering_angle', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)
        #self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)
        self.reverse_pub = self.create_publisher(Bool, '/stm/reverse_command', 10)

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)
        self.obstacle_sub = self.create_subscription(Int8, '/obstacle_detected', self.obstacle_callback, 10)
        self.sign_sub = self.create_subscription(String, '/detected_signs', self.sign_callback, 10)

        self.current_velocity_raw = 0.0
        self.current_velocity = 0.0       # filtrelenmiş anlık hız
        self.nav_target_velocity = 0.0    # navigasyondan gelen ham istek (ölçeklenmiş, eğrilik sınırı UYGULANMADAN)
        self.target_velocity = 0.0        # eğrilik-sınırlı, ivme rampasından geçmiş NİHAİ hedef

        self.last_odom = None
        self.last_odom_time = None

        self.obstacle_detected = False
        self.kirmizi = False

        self.last_motor_power = 0
        self.last_brake = False
        self.last_steering_deg = 0        # STM'ye gönderilecek final (-160..160)

        self.angular_z_filtered = 0.0
        self.last_cmd_time = None

        # Filtre / kontrolcü nesneleri
        self.vel_filter = EMAFilter(self.velocity_filter_alpha)
        self.angular_filter = EMAFilter(self.angular_filter_alpha)
        self.steering_filter = EMAFilter(self.steering_filter_alpha)
        self.steering_rate_limiter = RateLimiter(self.steering_rate_max, self.steering_rate_max)
        self.velocity_ramp = RateLimiter(self.max_accel, self.max_decel)
        self.curvature_predictor = CurvaturePredictor(
            window_size=self.curvature_window_size,
            lookahead_time=self.curvature_lookahead_time
        )

        # PID çıkışı doğrudan [min_motor_power, max_motor_power] = [28, 33] aralığında üretiliyor.
        self.velocity_pid = PID(
            kp=self.vel_kp, ki=self.vel_ki, kd=self.vel_kd,
            out_min=float(self.min_motor_power),
            out_max=float(self.max_motor_power),
            integral_limit=(self.max_motor_power - self.min_motor_power) * 3.0
        )
        self.last_pid_time = None

        # Timer ile sabit frekansta yayın (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(
            'CmdVel Node başlatıldı (eğrilik-bazlı hız sınırlama + öngörü + ivme rampası + PID).'
        )

    def _load_params(self):
        gp = lambda name: self.get_parameter(name).value
        self.max_motor_power = gp('max_motor_power')
        self.min_motor_power = gp('min_motor_power')
        self.max_velocity = gp('max_velocity')
        self.velocity_scale = gp('velocity_scale')

        self.vel_kp = gp('vel_kp')
        self.vel_ki = gp('vel_ki')
        self.vel_kd = gp('vel_kd')

        self.max_accel = gp('max_accel')
        self.max_decel = gp('max_decel')

        self.STEER_MAX_LEFT = gp('steer_max_left')
        self.STEER_MAX_RIGHT = gp('steer_max_right')
        self.WHEELBASE = gp('wheelbase')
        self.MAX_LEFT_DEG = gp('max_left_deg')
        self.MAX_RIGHT_DEG = gp('max_right_deg')

        # Fiziksel dereceyi -160..160 aralığına ölçekleyen gain (160/31.5≈5.08)
        avg_deg = (abs(self.MAX_LEFT_DEG) + abs(self.MAX_RIGHT_DEG)) / 2.0
        self.steering_gain = self.STEER_MAX_LEFT / avg_deg

        self.velocity_filter_alpha = gp('velocity_filter_alpha')
        self.angular_filter_alpha = gp('angular_filter_alpha')
        self.steering_filter_alpha = gp('steering_filter_alpha')
        self.steering_rate_max = gp('steering_rate_max')

        self.curvature_speed_limit_enable = gp('curvature_speed_limit_enable')
        self.curvature_min_speed_fraction = gp('curvature_min_speed_fraction')
        self.curvature_free_zone = gp('curvature_free_zone')
        self.curvature_lookahead_time = gp('curvature_lookahead_time')
        self.curvature_window_size = gp('curvature_window_size')

    # ==================== Callback'ler ====================

    def obstacle_callback(self, msg: Int8):
        was_detected = self.obstacle_detected
        self.obstacle_detected = (msg.data == 1)
        if self.obstacle_detected and not was_detected:
            self.get_logger().warn('[ENGEL] Engel algılandı! Araç durdurulacak.')
            self.velocity_pid.reset()

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

        self.current_velocity_raw = velocity_mps
        # Gürültülü odometri türevini yumuşat
        self.current_velocity = self.vel_filter.update(velocity_mps)

        self.last_odom = current_odom
        self.last_odom_time = current_time

    def sign_callback(self, msg):
        try:
            detected = msg.data
            if "kirmizi" in detected:
                if not self.kirmizi:
                    self.get_logger().info("Kırmızı ışık algılandı.")
                    self.velocity_pid.reset()
                self.kirmizi = True
            else:
                self.kirmizi = False
        except Exception as e:
            self.get_logger().error(f"Levha verisi işlenemedi: {e}")

    def cmd_vel_callback(self, msg: Twist):
        now = time()
        dt = 0.05 if self.last_cmd_time is None else max(1e-3, now - self.last_cmd_time)
        self.last_cmd_time = now

        if self.obstacle_detected:
            self.nav_target_velocity = 0.0
            return

        linear_x = msg.linear.x
        if linear_x <= 0:
            self.nav_target_velocity = 0.0
            return

        # Navigasyonun istediği ham hız, sadece çarpanla ölçeklenip tavana kırpılıyor.
        # ros2 param set /cmd_vel_subscriber velocity_scale 1.3   -> genel hızı arttır
        self.nav_target_velocity = min(linear_x * self.velocity_scale, self.max_velocity)

        # Gelen angular_z'yi filtrele - direksiyon titremesinin ana kaynağı budur
        raw_angular = msg.angular.z * 1.25
        self.angular_z_filtered = self.angular_filter.update(raw_angular)

        # === Direksiyon açısı hesabı (bicycle model) ===
        # Bölen olarak ANLIK (filtrelenmiş) hız kullanılıyor: araç henüz yavaşken
        # aynı açısal hız için daha büyük direksiyon açısı üretilir, hızlandıkça
        # açı doğal olarak küçülüp hıza "ayak uydurur". Bölme patlamasına karşı taban var.
        effective_velocity = max(self.current_velocity, 0.2)
        steering_rad = math.atan((self.WHEELBASE * self.angular_z_filtered) / effective_velocity)

        angle_val = math.degrees(steering_rad) * self.steering_gain
        steering_val = max(self.STEER_MAX_RIGHT, min(self.STEER_MAX_LEFT, angle_val)) * -1

        # Çıkışı EMA ile filtrele
        steering_val = self.steering_filter.update(steering_val)

        # Zaman-bazlı rate-limit: direksiyon saniyede en fazla steering_rate_max kadar değişsin
        self.last_steering_deg = int(round(self.steering_rate_limiter.update(steering_val, dt)))

        # === Eğrilik-bazlı hız sınırlama + öngörü ===
        if self.curvature_speed_limit_enable:
            max_steer_mag = max(abs(self.STEER_MAX_LEFT), abs(self.STEER_MAX_RIGHT))
            curvature_fraction = min(1.0, abs(self.last_steering_deg) / max_steer_mag)

            # Son örneklerin trendine bakarak yakın gelecekteki eğriliği tahmin et
            predicted_fraction = self.curvature_predictor.update(curvature_fraction, now)

            # curvature_free_zone altında hiç kısıtlama yok (düz yol / hafif kavis)
            if predicted_fraction <= self.curvature_free_zone:
                effective_curv = 0.0
            else:
                # free_zone ile 1.0 arasını yeniden 0..1'e ölçekle
                effective_curv = (predicted_fraction - self.curvature_free_zone) / (
                    1.0 - self.curvature_free_zone
                )
                effective_curv = max(0.0, min(1.0, effective_curv))

            # Düz yolda %100 hız, tam kilitte curvature_min_speed_fraction kadar hız
            speed_cap_fraction = 1.0 - effective_curv * (1.0 - self.curvature_min_speed_fraction)
        else:
            speed_cap_fraction = 1.0

        # Bu, ivme rampasının hedefleyeceği NİHAİ (eğrilik-sınırlı) hız
        self.target_velocity = self.nav_target_velocity * speed_cap_fraction

    # ==================== Ana kontrol döngüsü (50 Hz) ====================

    def timer_callback(self):
        if self.kirmizi:
            return

        if self.obstacle_detected:
            self.motor_power_pub.publish(Int8(data=0))
            #self.brake_pub.publish(Bool(data=True))
            self.steering_angle_pub.publish(Int16(data=0))
            self.velocity_pid.reset()
            self.velocity_ramp.reset(0.0)
            self.steering_rate_limiter.reset(0.0)
            return

        now = time()
        dt = 0.02 if self.last_pid_time is None else max(1e-3, now - self.last_pid_time)
        self.last_pid_time = now

        # Hedef hızı (eğrilik-sınırlı) ivme limitiyle yumuşat - ani "gaza basma/kesme" olmasın
        ramped_target = self.velocity_ramp.update(self.target_velocity, dt)

        if ramped_target <= 0.02:
            self.last_motor_power = 0
            self.last_brake = True
            self.velocity_pid.reset()
        else:
            error = ramped_target - self.current_velocity

            if self.current_velocity >= ramped_target + 1.0:
                # Beklenmedik şekilde çok hızlıyız: motoru kes, PID'i sıfırla
                self.last_motor_power = 0
                self.last_brake = True
                self.velocity_pid.reset()
            else:
                motor_power = self.velocity_pid.compute(error, dt)
                self.last_motor_power = int(round(motor_power))
                self.last_brake = False

        self.motor_power_pub.publish(Int8(data=self.last_motor_power))
        #self.brake_pub.publish(Bool(data=self.last_brake))
        self.steering_angle_pub.publish(Int16(data=self.last_steering_deg))

        self.get_logger().info(
            f'[YAYIN] Nav Hedef: {self.nav_target_velocity:.2f} | '
            f'Eğrilik-sınırlı Hedef: {self.target_velocity:.2f} | '
            f'Rampalı Hedef: {ramped_target:.2f} | '
            f'Anlık Hız: {self.current_velocity:.2f} m/s | '
            f'Motor: {self.last_motor_power} | Fren: {self.last_brake} | '
            f'Direksiyon: {self.last_steering_deg}'
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