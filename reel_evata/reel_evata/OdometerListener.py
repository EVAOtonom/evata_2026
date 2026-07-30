#!/usr/bin/env python3

import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry


class WheelEncoderOdometry(Node):
    """
    Kümülatif santimetre cinsinden gelen teker enkoderini filtreler
    ve EKF'nin kullanabileceği ileri hız ölçümüne dönüştürür.

    Varsayım:
        /stm/read_odometer mesajı kümülatif mesafedir.

    EKF'de kullanılacak alan:
        twist.twist.linear.x

    EKF'de kullanılmayacak alan:
        pose.pose.position.x
    """

    def __init__(self):
        super().__init__('wheel_encoder_odometry')

        # ========================================================
        # TOPIC VE FRAME PARAMETRELERİ
        # ========================================================

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

        # ========================================================
        # FİZİKSEL SINIR PARAMETRELERİ
        # ========================================================

        # Aracın fiziksel olarak ulaşabileceği maksimum hız.
        # İlk test değeri. Gerçek maksimum hızınıza göre ayarlanmalı.
        self.declare_parameter(
            'max_speed_mps',
            6.0
        )

        # İki geçerli ölçüm arasında izin verilen maksimum ivme.
        self.declare_parameter(
            'max_acceleration_mps2',
            6.0
        )

        # Dinamik maksimum mesafe kontrolüne eklenecek tolerans.
        self.declare_parameter(
            'delta_margin_m',
            0.10
        )

        # Araç geri gidebiliyorsa true kalmalı.
        self.declare_parameter(
            'allow_reverse',
            True
        )

        # İzin verilen en uzun örnek aralığı.
        # Bundan uzun ara olursa hız hesabı sıfırlanır.
        self.declare_parameter(
            'max_sample_gap_sec',
            1.0
        )

        # Çok küçük zaman farklarında bölme hatasını önler.
        self.declare_parameter(
            'min_sample_period_sec',
            0.001
        )

        # ========================================================
        # ENKODER RESET PARAMETRELERİ
        # ========================================================

        # Mevcut değer sıfıra yakınsa ve önceki değer yeterince
        # büyükse enkoder reseti olduğu kabul edilir.
        self.declare_parameter(
            'reset_near_zero_cm',
            20.0
        )

        self.declare_parameter(
            'reset_previous_min_cm',
            500.0
        )

        # ========================================================
        # FİLTRE PARAMETRELERİ
        # ========================================================

        # 3 örneklik median filtre tek örneklik sıçramaları temizler.
        # Teker verisi yaklaşık 3 Hz olduğu için 5 kullanmak
        # gereğinden fazla gecikme oluşturabilir.
        self.declare_parameter(
            'median_window_size',
            3
        )

        # Low-pass filtresi:
        #
        # filtered = alpha * yeni + (1-alpha) * eski
        #
        # 1.0 -> filtre yok
        # küçük değer -> daha fazla yumuşatma, daha fazla gecikme
        self.declare_parameter(
            'low_pass_alpha',
            0.60
        )

        # Çok küçük hızları sıfıra çeker.
        self.declare_parameter(
            'zero_speed_deadband_mps',
            0.03
        )

        # EKF'ye verilecek vx varyansı.
        # 0.04 = 0.2 m/s standart sapma.
        self.declare_parameter(
            'velocity_variance',
            0.04
        )

        # Kümülatif mesafeyi pose.x içinde yalnızca izleme amacıyla
        # yayınlamak için kullanılabilir.
        #
        # EKF yine pose.x alanını kullanmamalıdır.
        self.declare_parameter(
            'publish_integrated_distance_in_pose',
            False
        )

        # ========================================================
        # PARAMETRELERİ OKU
        # ========================================================

        self.encoder_topic = str(
            self.get_parameter('encoder_topic').value
        )

        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )

        self.odom_frame = str(
            self.get_parameter('odom_frame').value
        )

        self.base_frame = str(
            self.get_parameter('base_frame').value
        )

        self.max_speed_mps = float(
            self.get_parameter('max_speed_mps').value
        )

        self.max_acceleration_mps2 = float(
            self.get_parameter(
                'max_acceleration_mps2'
            ).value
        )

        self.delta_margin_m = float(
            self.get_parameter('delta_margin_m').value
        )

        self.allow_reverse = bool(
            self.get_parameter('allow_reverse').value
        )

        self.max_sample_gap_sec = float(
            self.get_parameter(
                'max_sample_gap_sec'
            ).value
        )

        self.min_sample_period_sec = float(
            self.get_parameter(
                'min_sample_period_sec'
            ).value
        )

        self.reset_near_zero_cm = float(
            self.get_parameter(
                'reset_near_zero_cm'
            ).value
        )

        self.reset_previous_min_cm = float(
            self.get_parameter(
                'reset_previous_min_cm'
            ).value
        )

        median_window_size = int(
            self.get_parameter(
                'median_window_size'
            ).value
        )

        # Median pencere boyutu en az 1 ve tek sayı olmalı.
        median_window_size = max(1, median_window_size)

        if median_window_size % 2 == 0:
            median_window_size += 1

        self.median_window_size = median_window_size

        self.low_pass_alpha = float(
            self.get_parameter('low_pass_alpha').value
        )

        self.low_pass_alpha = min(
            1.0,
            max(0.0, self.low_pass_alpha)
        )

        self.zero_speed_deadband_mps = float(
            self.get_parameter(
                'zero_speed_deadband_mps'
            ).value
        )

        self.velocity_variance = float(
            self.get_parameter(
                'velocity_variance'
            ).value
        )

        self.publish_integrated_distance_in_pose = bool(
            self.get_parameter(
                'publish_integrated_distance_in_pose'
            ).value
        )

        # ========================================================
        # PUBLISHER'LAR
        # ========================================================

        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_topic,
            20
        )

        # Filtre davranışını incelemek için yardımcı topic'ler.
        self.raw_speed_pub = self.create_publisher(
            Float32,
            '/wheel/raw_speed',
            10
        )

        self.filtered_speed_pub = self.create_publisher(
            Float32,
            '/wheel/filtered_speed',
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            '/wheel/integrated_distance',
            10
        )

        self.filter_valid_pub = self.create_publisher(
            Bool,
            '/wheel/filter_valid',
            10
        )

        # ========================================================
        # SUBSCRIBER
        # ========================================================

        self.encoder_sub = self.create_subscription(
            Float32,
            self.encoder_topic,
            self.encoder_callback,
            20
        )

        # ========================================================
        # DURUM DEĞİŞKENLERİ
        # ========================================================

        # Son kabul edilen enkoder değeri.
        self.last_encoder_cm = None

        # Son kabul edilen mesajın zamanı.
        self.last_time = None

        # Son kabul edilen filtresiz hız.
        self.last_raw_speed_mps = None

        # Son filtrelenmiş hız.
        self.filtered_speed_mps = 0.0

        # İzleme amacıyla entegre edilen toplam mesafe.
        self.integrated_distance_m = 0.0

        # Median filtre penceresi.
        self.speed_window = deque(
            maxlen=self.median_window_size
        )

        self.get_logger().info(
            '\n'
            'Teker enkoder filtresi başlatıldı.\n'
            f'Encoder input       : {self.encoder_topic}\n'
            f'Odometry output     : {self.odom_topic}\n'
            'EKF ölçümü           : twist.linear.x\n'
            'EKF pose ölçümü      : kullanılmayacak\n'
            f'Maksimum hız        : {self.max_speed_mps:.2f} m/s\n'
            f'Maksimum ivme       : '
            f'{self.max_acceleration_mps2:.2f} m/s^2\n'
            f'Median pencere      : {self.median_window_size}\n'
            f'Low-pass alpha      : {self.low_pass_alpha:.2f}'
        )

    # ============================================================
    # ENKODER CALLBACK
    # ============================================================

    def encoder_callback(self, msg: Float32):
        current_encoder_cm = float(msg.data)
        current_time = self.get_clock().now()

        # NaN veya Inf kontrolü.
        if not math.isfinite(current_encoder_cm):
            self.reject_measurement(
                'Enkoder değeri NaN veya Inf'
            )
            return

        # İlk mesaj yalnızca referans olarak kaydedilir.
        if self.last_encoder_cm is None:
            self.last_encoder_cm = current_encoder_cm
            self.last_time = current_time

            self.filtered_speed_mps = 0.0
            self.last_raw_speed_mps = 0.0

            self.publish_debug_topics(
                raw_speed_mps=0.0,
                valid=False
            )

            self.publish_wheel_odometry(
                stamp=current_time,
                speed_mps=0.0
            )

            self.get_logger().info(
                f'İlk enkoder referansı alındı: '
                f'{current_encoder_cm:.2f} cm'
            )
            return

        dt = (
            current_time.nanoseconds -
            self.last_time.nanoseconds
        ) * 1e-9

        # Çok küçük zaman farkı.
        if dt < self.min_sample_period_sec:
            self.reject_measurement(
                f'Örnek zamanı çok küçük: {dt:.6f} s'
            )
            return

        # Çok uzun veri kesintisi.
        if dt > self.max_sample_gap_sec:
            self.get_logger().warning(
                f'Uzun enkoder veri aralığı: {dt:.3f} s. '
                'Hız filtresi yeniden başlatılıyor.'
            )

            self.reset_filter_reference(
                current_encoder_cm,
                current_time
            )
            return

        delta_cm = (
            current_encoder_cm -
            self.last_encoder_cm
        )

        delta_m = delta_cm / 100.0

        # Zamana bağlı fiziksel maksimum mesafe.
        max_allowed_delta_m = (
            self.max_speed_mps * dt +
            self.delta_margin_m
        )

        # ========================================================
        # ENKODER RESET VE SIÇRAMA KONTROLÜ
        # ========================================================

        if abs(delta_m) > max_allowed_delta_m:

            encoder_reset_detected = (
                current_encoder_cm <=
                self.reset_near_zero_cm
                and
                self.last_encoder_cm >=
                self.reset_previous_min_cm
            )

            if encoder_reset_detected:
                self.get_logger().warning(
                    'Enkoder reseti tespit edildi. '
                    f'Önceki={self.last_encoder_cm:.2f} cm, '
                    f'Yeni={current_encoder_cm:.2f} cm'
                )

                self.reset_filter_reference(
                    current_encoder_cm,
                    current_time
                )
                return

            # Geçici bozuk örnekte referansı değiştirmiyoruz.
            # Böylece sonraki doğru mesaj eski geçerli örnekle
            # karşılaştırılabilir.
            self.reject_measurement(
                'Fiziksel olmayan enkoder sıçraması: '
                f'delta={delta_m:.3f} m, '
                f'dt={dt:.3f} s, '
                f'izin={max_allowed_delta_m:.3f} m'
            )
            return

        raw_speed_mps = delta_m / dt

        # Geri sürüşe izin verilmiyorsa negatif hız reddedilir.
        if not self.allow_reverse and raw_speed_mps < 0.0:
            self.reject_measurement(
                'Negatif teker hızı reddedildi: '
                f'{raw_speed_mps:.3f} m/s'
            )
            return

        # ========================================================
        # İVME KONTROLÜ
        # ========================================================

        if self.last_raw_speed_mps is not None:
            acceleration_mps2 = (
                raw_speed_mps -
                self.last_raw_speed_mps
            ) / dt

            if (
                abs(acceleration_mps2) >
                self.max_acceleration_mps2
            ):
                self.reject_measurement(
                    'Fiziksel olmayan teker ivmesi: '
                    f'{acceleration_mps2:.3f} m/s^2, '
                    f'hız={raw_speed_mps:.3f} m/s'
                )
                return

        # Bu noktadan sonra ölçüm kabul edildi.
        self.last_encoder_cm = current_encoder_cm
        self.last_time = current_time
        self.last_raw_speed_mps = raw_speed_mps

        # ========================================================
        # MEDIAN FİLTRE
        # ========================================================

        self.speed_window.append(raw_speed_mps)

        median_speed_mps = float(
            statistics.median(self.speed_window)
        )

        # ========================================================
        # LOW-PASS FİLTRE
        # ========================================================

        self.filtered_speed_mps = (
            self.low_pass_alpha * median_speed_mps +
            (1.0 - self.low_pass_alpha) *
            self.filtered_speed_mps
        )

        # Düşük hız deadband.
        if (
            abs(self.filtered_speed_mps) <
            self.zero_speed_deadband_mps
        ):
            self.filtered_speed_mps = 0.0

        # Toplam mesafede filtrelenmemiş fakat kabul edilmiş
        # fiziksel delta kullanılır.
        self.integrated_distance_m += delta_m

        self.publish_wheel_odometry(
            stamp=current_time,
            speed_mps=self.filtered_speed_mps
        )

        self.publish_debug_topics(
            raw_speed_mps=raw_speed_mps,
            valid=True
        )

    # ============================================================
    # FİLTRE RESET
    # ============================================================

    def reset_filter_reference(
        self,
        encoder_cm,
        stamp
    ):
        self.last_encoder_cm = encoder_cm
        self.last_time = stamp
        self.last_raw_speed_mps = None

        self.filtered_speed_mps = 0.0
        self.speed_window.clear()

        self.publish_debug_topics(
            raw_speed_mps=0.0,
            valid=False
        )

    # ============================================================
    # REDDEDİLEN ÖLÇÜM
    # ============================================================

    def reject_measurement(self, reason):
        self.get_logger().warning(
            f'Teker ölçümü reddedildi: {reason}'
        )

        # Reddedilen ölçüm /teker olarak yayınlanmaz.
        # Böylece EKF geçersiz veriyi kullanamaz.
        self.publish_debug_topics(
            raw_speed_mps=0.0,
            valid=False
        )

    # ============================================================
    # DEBUG TOPIC'LERİ
    # ============================================================

    def publish_debug_topics(
        self,
        raw_speed_mps,
        valid
    ):
        raw_msg = Float32()
        raw_msg.data = float(raw_speed_mps)
        self.raw_speed_pub.publish(raw_msg)

        filtered_msg = Float32()
        filtered_msg.data = float(
            self.filtered_speed_mps
        )
        self.filtered_speed_pub.publish(
            filtered_msg
        )

        distance_msg = Float32()
        distance_msg.data = float(
            self.integrated_distance_m
        )
        self.distance_pub.publish(
            distance_msg
        )

        valid_msg = Bool()
        valid_msg.data = bool(valid)
        self.filter_valid_pub.publish(valid_msg)

    # ============================================================
    # ODOMETRY YAYINI
    # ============================================================

    def publish_wheel_odometry(
        self,
        stamp,
        speed_mps
    ):
        odom_msg = Odometry()

        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # --------------------------------------------------------
        # POSE
        # --------------------------------------------------------
        #
        # Teker enkoderi global X-Y pose ölçmez.
        # Bu alanlar EKF'de kullanılmayacaktır.

        if self.publish_integrated_distance_in_pose:
            odom_msg.pose.pose.position.x = (
                self.integrated_distance_m
            )
        else:
            odom_msg.pose.pose.position.x = 0.0

        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        # Pose alanlarının tamamı güvenilmez.
        large_variance = 1000000.0

        odom_msg.pose.covariance = [
            large_variance, 0.0,            0.0,            0.0,            0.0,            0.0,
            0.0,            large_variance, 0.0,            0.0,            0.0,            0.0,
            0.0,            0.0,            large_variance, 0.0,            0.0,            0.0,
            0.0,            0.0,            0.0,            large_variance, 0.0,            0.0,
            0.0,            0.0,            0.0,            0.0,            large_variance, 0.0,
            0.0,            0.0,            0.0,            0.0,            0.0,            large_variance
        ]

        # --------------------------------------------------------
        # TWIST
        # --------------------------------------------------------

        # EKF'nin kullanacağı ölçüm.
        odom_msg.twist.twist.linear.x = float(
            speed_mps
        )

        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Sadece vx güvenilir.
        odom_msg.twist.covariance = [
            self.velocity_variance, 0.0,            0.0,            0.0,            0.0,            0.0,
            0.0,                    large_variance, 0.0,            0.0,            0.0,            0.0,
            0.0,                    0.0,            large_variance, 0.0,            0.0,            0.0,
            0.0,                    0.0,            0.0,            large_variance, 0.0,            0.0,
            0.0,                    0.0,            0.0,            0.0,            large_variance, 0.0,
            0.0,                    0.0,            0.0,            0.0,            0.0,            large_variance
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
