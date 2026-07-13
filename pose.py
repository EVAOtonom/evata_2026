#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped,
)

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformBroadcaster,
    TransformException,
)


class InitialPoseMapToOdom(Node):
    """
    RViz veya başka bir node tarafından yayınlanan /initialpose mesajından
    map -> odom dönüşümünü hesaplar ve TF olarak yayınlar.

    Kullanılan dönüşümler:

        T_map_base:
            /initialpose mesajından gelir.

        T_odom_base:
            TF ağacından alınır.

        T_map_odom:
            T_map_base * inverse(T_odom_base)

    Bu node 2D araç kullanımı için x, y ve yaw üzerinden hesaplama yapar.
    """

    def __init__(self):
        super().__init__('initialpose_map_to_odom')

        # ============================================================
        # PARAMETRELER
        # ============================================================

        self.declare_parameter(
            'initialpose_topic',
            '/initialpose'
        )

        self.declare_parameter(
            'map_frame',
            'map'
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
            'tf_publish_rate',
            30.0
        )

        # Initialpose geldiğinde TF bulunamazsa kaç saniye sonra tekrar denensin?
        self.declare_parameter(
            'retry_period',
            0.1
        )

        # TF araması yapılırken en güncel dönüşüm mü kullanılsın?
        self.declare_parameter(
            'use_latest_tf',
            True
        )

        # Z ekseni sıfırlansın mı?
        self.declare_parameter(
            'zero_z',
            True
        )

        # ============================================================
        # PARAMETRELERİ OKU
        # ============================================================

        self.initialpose_topic = self.get_parameter(
            'initialpose_topic'
        ).value

        self.map_frame = self.get_parameter(
            'map_frame'
        ).value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.tf_publish_rate = float(
            self.get_parameter(
                'tf_publish_rate'
            ).value
        )

        self.retry_period = float(
            self.get_parameter(
                'retry_period'
            ).value
        )

        self.use_latest_tf = bool(
            self.get_parameter(
                'use_latest_tf'
            ).value
        )

        self.zero_z = bool(
            self.get_parameter(
                'zero_z'
            ).value
        )

        # ============================================================
        # DURUM DEĞİŞKENLERİ
        # ============================================================

        self.has_map_to_odom = False

        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_z = 0.0
        self.map_to_odom_yaw = 0.0

        # TF henüz hazır değilse initialpose burada bekletilir.
        self.pending_initialpose = None

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # map -> odom sürekli yayınlanır.
        self.tf_publish_timer = self.create_timer(
            1.0 / self.tf_publish_rate,
            self.publish_map_to_odom
        )

        # TF ilk anda bulunamazsa tekrar denemek için timer.
        self.retry_timer = self.create_timer(
            self.retry_period,
            self.retry_pending_initialpose
        )

        # ============================================================
        # SUBSCRIBER
        # ============================================================

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10
        )

        self.get_logger().info(
            'Initialpose map -> odom publisher started.'
        )

        self.get_logger().info(
            f'Initialpose topic: {self.initialpose_topic}'
        )

        self.get_logger().info(
            f'TF chain: {self.map_frame} -> '
            f'{self.odom_frame} -> {self.base_frame}'
        )

    # ================================================================
    # INITIALPOSE CALLBACK
    # ================================================================

    def initialpose_callback(
        self,
        msg: PoseWithCovarianceStamped
    ):
        """
        Gelen /initialpose mesajını saklar ve map -> odom hesabını dener.
        """

        if not self.is_valid_pose(msg):
            self.get_logger().error(
                'Initialpose contains invalid, NaN or infinite values.'
            )
            return

        source_frame = msg.header.frame_id

        if source_frame == '':
            source_frame = self.map_frame

        if source_frame != self.map_frame:
            self.get_logger().error(
                f'Initialpose frame must be "{self.map_frame}", '
                f'but received "{source_frame}".'
            )
            return

        self.pending_initialpose = msg

        self.get_logger().info(
            'Initialpose received. Calculating map -> odom...'
        )

        self.calculate_map_to_odom(msg)

    # ================================================================
    # RETRY
    # ================================================================

    def retry_pending_initialpose(self):
        """
        Initialpose geldiği anda odom -> base TF bulunamazsa,
        bekleyen initialpose ile tekrar hesaplama yapar.
        """

        if self.pending_initialpose is None:
            return

        self.calculate_map_to_odom(
            self.pending_initialpose
        )

    # ================================================================
    # MAP -> ODOM HESABI
    # ================================================================

    def calculate_map_to_odom(
        self,
        initialpose_msg: PoseWithCovarianceStamped
    ):
        """
        T_map_odom = T_map_base * inverse(T_odom_base)
        """

        try:
            if self.use_latest_tf:
                lookup_time = Time()
            else:
                lookup_time = Time.from_msg(
                    initialpose_msg.header.stamp
                )

            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                lookup_time
            )

        except TransformException as ex:
            self.get_logger().warn(
                f'Waiting for TF '
                f'{self.odom_frame} -> {self.base_frame}: {ex}',
                throttle_duration_sec=2.0
            )
            return

        # ------------------------------------------------------------
        # T_map_base
        # /initialpose içinden alınır.
        # ------------------------------------------------------------

        map_base_x = (
            initialpose_msg.pose.pose.position.x
        )

        map_base_y = (
            initialpose_msg.pose.pose.position.y
        )

        map_base_z = (
            initialpose_msg.pose.pose.position.z
        )

        map_base_yaw = self.yaw_from_quaternion(
            initialpose_msg.pose.pose.orientation
        )

        # ------------------------------------------------------------
        # T_odom_base
        # TF ağacından alınır.
        # ------------------------------------------------------------

        odom_base_x = (
            odom_to_base.transform.translation.x
        )

        odom_base_y = (
            odom_to_base.transform.translation.y
        )

        odom_base_z = (
            odom_to_base.transform.translation.z
        )

        odom_base_yaw = self.yaw_from_quaternion(
            odom_to_base.transform.rotation
        )

        # ------------------------------------------------------------
        # Yön hesabı
        #
        # yaw(map -> odom)
        #     =
        # yaw(map -> base) - yaw(odom -> base)
        # ------------------------------------------------------------

        map_odom_yaw = self.normalize_angle(
            map_base_yaw - odom_base_yaw
        )

        cos_yaw = math.cos(map_odom_yaw)
        sin_yaw = math.sin(map_odom_yaw)

        # ------------------------------------------------------------
        # Konum hesabı
        #
        # t_map_base =
        #     t_map_odom + R_map_odom * t_odom_base
        #
        # Dolayısıyla:
        #
        # t_map_odom =
        #     t_map_base - R_map_odom * t_odom_base
        # ------------------------------------------------------------

        rotated_odom_base_x = (
            cos_yaw * odom_base_x
            - sin_yaw * odom_base_y
        )

        rotated_odom_base_y = (
            sin_yaw * odom_base_x
            + cos_yaw * odom_base_y
        )

        map_odom_x = (
            map_base_x - rotated_odom_base_x
        )

        map_odom_y = (
            map_base_y - rotated_odom_base_y
        )

        if self.zero_z:
            map_odom_z = 0.0
        else:
            map_odom_z = (
                map_base_z - odom_base_z
            )

        # ------------------------------------------------------------
        # Sonuçları kaydet
        # ------------------------------------------------------------

        self.map_to_odom_x = map_odom_x
        self.map_to_odom_y = map_odom_y
        self.map_to_odom_z = map_odom_z
        self.map_to_odom_yaw = map_odom_yaw

        self.has_map_to_odom = True

        # Hesap başarıyla tamamlandı.
        self.pending_initialpose = None

        # İlk TF'yi hemen yayınla.
        self.publish_map_to_odom()

        self.get_logger().warn(
            '\n'
            'map -> odom updated from /initialpose\n'
            f'Initial map -> base:\n'
            f'  x: {map_base_x:.3f}\n'
            f'  y: {map_base_y:.3f}\n'
            f'  yaw: {math.degrees(map_base_yaw):.2f} deg\n'
            f'Current odom -> base:\n'
            f'  x: {odom_base_x:.3f}\n'
            f'  y: {odom_base_y:.3f}\n'
            f'  yaw: {math.degrees(odom_base_yaw):.2f} deg\n'
            f'Calculated map -> odom:\n'
            f'  x: {map_odom_x:.3f}\n'
            f'  y: {map_odom_y:.3f}\n'
            f'  yaw: {math.degrees(map_odom_yaw):.2f} deg'
        )

    # ================================================================
    # TF PUBLISHER
    # ================================================================

    def publish_map_to_odom(self):
        """
        Hesaplanan map -> odom dönüşümünü sürekli yayınlar.
        """

        if not self.has_map_to_odom:
            return

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

        transform.header.frame_id = (
            self.map_frame
        )

        transform.child_frame_id = (
            self.odom_frame
        )

        transform.transform.translation.x = (
            self.map_to_odom_x
        )

        transform.transform.translation.y = (
            self.map_to_odom_y
        )

        transform.transform.translation.z = (
            self.map_to_odom_z
        )

        qx, qy, qz, qw = self.quaternion_from_yaw(
            self.map_to_odom_yaw
        )

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )

    # ================================================================
    # YARDIMCI FONKSİYONLAR
    # ================================================================

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        """
        Quaternion içinden yaw açısını çıkarır.
        """

        sin_yaw = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        return math.atan2(
            sin_yaw,
            cos_yaw
        )

    @staticmethod
    def quaternion_from_yaw(yaw: float):
        """
        Yaw açısından quaternion üretir.
        """

        half_yaw = yaw * 0.5

        return (
            0.0,
            0.0,
            math.sin(half_yaw),
            math.cos(half_yaw)
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Açıyı -pi ile +pi arasına getirir.
        """

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    @staticmethod
    def is_valid_pose(
        msg: PoseWithCovarianceStamped
    ) -> bool:
        """
        Gelen pose değerlerinin geçerli olup olmadığını kontrol eder.
        """

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        values = [
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]

        if not all(
            math.isfinite(value)
            for value in values
        ):
            return False

        quaternion_norm = math.sqrt(
            orientation.x ** 2
            + orientation.y ** 2
            + orientation.z ** 2
            + orientation.w ** 2
        )

        if quaternion_norm < 1e-6:
            return False

        return True


def main(args=None):
    rclpy.init(args=args)

    node = InitialPoseMapToOdom()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

