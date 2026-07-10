#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TransformStamped,
)
from std_msgs.msg import Header, Bool

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
    TransformBroadcaster,
)

import sensor_msgs_py.point_cloud2 as pc2


class MapAlignedGpsCircle(Node):
    def __init__(self):
        super().__init__('map_aligned_gps_circle')

        # =========================
        # PARAMETERS
        # =========================
        self.declare_parameter('gps_topic', '/odometry/gps')
        self.declare_parameter('pcl_pose_topic', '/pcl_pose')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('reset_manual_topic', '/reset_manual_initialpose')

        self.declare_parameter('output_topic', '/vehicle_boundary_cloud')
        self.declare_parameter('accepted_pcl_pose_topic', '/pcl_pose_accepted')
        self.declare_parameter('status_topic', '/pcl_pose_in_gps_area')

        self.declare_parameter('pcl_pose_cloud_topic', '/pcl_pose_vehicle_cloud')
        self.declare_parameter(
            'accepted_pcl_pose_cloud_topic',
            '/pcl_pose_accepted_vehicle_cloud'
        )

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # 4 m çap istiyorsan 2.0 yap.
        # 4 m yarıçap istiyorsan 4.0 bırak.
        self.declare_parameter('base_radius', 4.0)

        # GPS covariance'tan radius büyütme katsayısı.
        # 1.0 = yaklaşık 1 sigma
        # 2.0 = daha güvenli alan
        self.declare_parameter('sigma_multiplier', 2.0)

        self.declare_parameter('max_radius', 30.0)
        self.declare_parameter('num_points', 120)

        # Araç çizimi için
        self.declare_parameter('vehicle_length', 2.1)
        self.declare_parameter('vehicle_width', 0.95)
        self.declare_parameter('heading_arrow_length', 1.2)
        self.declare_parameter('vehicle_cloud_density', 20)

        # Bu node map -> odom yayınlasın mı?
        self.declare_parameter('enable_map_odom_tf', True)

        # map -> odom yayın frekansı
        self.declare_parameter('tf_publish_rate', 30.0)

        # İlk gerçek relokalizasyon gelmeden önce odom'u map gibi kabul et.
        self.declare_parameter('assume_odom_as_map_before_relocalization', True)

        # =========================
        # READ PARAMETERS
        # =========================
        self.gps_topic = self.get_parameter('gps_topic').value
        self.pcl_pose_topic = self.get_parameter('pcl_pose_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.reset_manual_topic = self.get_parameter('reset_manual_topic').value

        self.output_topic = self.get_parameter('output_topic').value
        self.accepted_pcl_pose_topic = self.get_parameter(
            'accepted_pcl_pose_topic'
        ).value
        self.status_topic = self.get_parameter('status_topic').value

        self.pcl_pose_cloud_topic = self.get_parameter(
            'pcl_pose_cloud_topic'
        ).value
        self.accepted_pcl_pose_cloud_topic = self.get_parameter(
            'accepted_pcl_pose_cloud_topic'
        ).value

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.base_radius = float(self.get_parameter('base_radius').value)
        self.sigma_multiplier = float(
            self.get_parameter('sigma_multiplier').value
        )
        self.max_radius = float(self.get_parameter('max_radius').value)
        self.num_points = int(self.get_parameter('num_points').value)

        self.vehicle_length = float(self.get_parameter('vehicle_length').value)
        self.vehicle_width = float(self.get_parameter('vehicle_width').value)
        self.heading_arrow_length = float(
            self.get_parameter('heading_arrow_length').value
        )
        self.vehicle_cloud_density = int(
            self.get_parameter('vehicle_cloud_density').value
        )

        self.enable_map_odom_tf = bool(
            self.get_parameter('enable_map_odom_tf').value
        )

        self.assume_odom_as_map_before_relocalization = bool(
            self.get_parameter(
                'assume_odom_as_map_before_relocalization'
            ).value
        )

        tf_publish_rate = float(self.get_parameter('tf_publish_rate').value)

        # =========================
        # STATE
        # =========================
        self.manual_pose_active = False

        self.center_x = None
        self.center_y = None
        self.center_z = 0.0
        self.current_radius = self.base_radius

        self.latest_gps_x = None
        self.latest_gps_y = None
        self.latest_gps_z = None

        # Başlangıçta identity map -> odom yayınla.
        # Kabul edilmiş /pcl_pose gelince gerçek değere güncellenecek.
        self.has_map_to_odom = True
        self.has_relocalized_pose = False

        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_z = 0.0
        self.map_to_odom_yaw = 0.0

        # =========================
        # TF
        # =========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_timer = self.create_timer(
            1.0 / tf_publish_rate,
            self.publish_map_to_odom_timer
        )

        # =========================
        # SUBSCRIBERS
        # =========================
        self.gps_sub = self.create_subscription(
            Odometry,
            self.gps_topic,
            self.gps_callback,
            10
        )

        # ÖNEMLİ:
        # lidar_localization /pcl_pose'u PoseWithCovarianceStamped basıyor.
        self.pcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pcl_pose_topic,
            self.pcl_pose_cov_callback,
            10
        )

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10
        )

        self.reset_manual_sub = self.create_subscription(
            Bool,
            self.reset_manual_topic,
            self.reset_manual_callback,
            10
        )

        # =========================
        # PUBLISHERS
        # =========================
        self.circle_pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10
        )

        self.accepted_pcl_pose_pub = self.create_publisher(
            PoseStamped,
            self.accepted_pcl_pose_topic,
            10
        )

        self.status_pub = self.create_publisher(
            Bool,
            self.status_topic,
            10
        )

        self.pcl_pose_cloud_pub = self.create_publisher(
            PointCloud2,
            self.pcl_pose_cloud_topic,
            10
        )

        self.accepted_pcl_pose_cloud_pub = self.create_publisher(
            PointCloud2,
            self.accepted_pcl_pose_cloud_topic,
            10
        )

        self.get_logger().info('Map aligned GPS relocalization filter started.')
        self.get_logger().info(f'GPS topic: {self.gps_topic}')
        self.get_logger().info(
            f'PCL pose topic: {self.pcl_pose_topic} '
            f'(PoseWithCovarianceStamped)'
        )
        self.get_logger().info(f'Circle cloud topic: {self.output_topic}')
        self.get_logger().info(f'Raw PCL vehicle cloud: {self.pcl_pose_cloud_topic}')
        self.get_logger().info(
            f'Accepted PCL vehicle cloud: {self.accepted_pcl_pose_cloud_topic}'
        )
        self.get_logger().info(f'Accepted PCL pose topic: {self.accepted_pcl_pose_topic}')
        self.get_logger().info(f'Status topic: {self.status_topic}')
        self.get_logger().info(f'Map frame: {self.map_frame}')
        self.get_logger().info(f'Odom frame: {self.odom_frame}')
        self.get_logger().info(f'Base frame: {self.base_frame}')
        self.get_logger().info(f'Base radius: {self.base_radius:.2f} m')
        self.get_logger().info(f'Sigma multiplier: {self.sigma_multiplier:.2f}')
        self.get_logger().info(f'Max radius: {self.max_radius:.2f} m')
        self.get_logger().info(f'enable_map_odom_tf: {self.enable_map_odom_tf}')

    # =========================
    # CALLBACKS
    # =========================

    def gps_callback(self, msg: Odometry):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        pose_map = self.transform_pose_to_map(pose_stamped)
        if pose_map is None:
            return

        self.latest_gps_x = pose_map.pose.position.x
        self.latest_gps_y = pose_map.pose.position.y
        self.latest_gps_z = pose_map.pose.position.z

        if self.manual_pose_active:
            return

        self.center_x = self.latest_gps_x
        self.center_y = self.latest_gps_y
        self.center_z = self.latest_gps_z

        self.current_radius = self.compute_radius_from_covariance(
            msg.pose.covariance
        )

        self.publish_circle(
            self.center_x,
            self.center_y,
            self.center_z,
            self.current_radius
        )

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        pose_map = self.transform_pose_to_map(pose_stamped)
        if pose_map is None:
            return

        self.manual_pose_active = True

        self.center_x = pose_map.pose.position.x
        self.center_y = pose_map.pose.position.y
        self.center_z = pose_map.pose.position.z

        self.current_radius = self.compute_radius_from_covariance(
            msg.pose.covariance
        )

        self.get_logger().warn(
            f'Manual initial pose received. '
            f'Circle center switched to MANUAL: '
            f'x={self.center_x:.3f}, y={self.center_y:.3f}, '
            f'radius={self.current_radius:.3f}'
        )

        self.publish_circle(
            self.center_x,
            self.center_y,
            self.center_z,
            self.current_radius
        )

    def reset_manual_callback(self, msg: Bool):
        if not msg.data:
            return

        self.manual_pose_active = False

        self.get_logger().warn(
            'Manual initial pose reset. Circle center switched back to GPS.'
        )

        if self.latest_gps_x is not None:
            self.center_x = self.latest_gps_x
            self.center_y = self.latest_gps_y
            self.center_z = self.latest_gps_z

            self.publish_circle(
                self.center_x,
                self.center_y,
                self.center_z,
                self.current_radius
            )

    def pcl_pose_cov_callback(self, msg: PoseWithCovarianceStamped):
        """
        lidar_localization /pcl_pose'u PoseWithCovarianceStamped yayınlıyor.
        Biz içeride PoseStamped'a çevirip aynı filtre fonksiyonunu kullanıyoruz.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        self.handle_pcl_pose(pose_stamped)

    def handle_pcl_pose(self, msg: PoseStamped):
        if self.center_x is None or self.center_y is None:
            self.publish_status(False)
            self.get_logger().warn(
                'PCL pose received, but GPS/manual circle center is not ready yet.'
            )
            return

        pcl_pose_map = self.transform_pose_to_map(msg)
        if pcl_pose_map is None:
            self.publish_status(False)
            return

        pcl_x = pcl_pose_map.pose.position.x
        pcl_y = pcl_pose_map.pose.position.y

        # Ham /pcl_pose'un "araç burada" dediği yeri RViz'de göster
        self.publish_vehicle_cloud(
            pcl_pose_map,
            self.pcl_pose_cloud_pub
        )

        dx = pcl_x - self.center_x
        dy = pcl_y - self.center_y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= self.current_radius:
            self.accepted_pcl_pose_pub.publish(pcl_pose_map)
            self.publish_status(True)

            # Kabul edilen pose'u ayrı göster
            self.publish_vehicle_cloud(
                pcl_pose_map,
                self.accepted_pcl_pose_cloud_pub
            )

            # Kabul edilen pose'a göre map -> odom güncelle
            if self.enable_map_odom_tf:
                self.update_map_to_odom_from_accepted_pose(pcl_pose_map)

            self.get_logger().info(
                f'PCL pose ACCEPTED. '
                f'distance={distance:.2f} m <= radius={self.current_radius:.2f} m'
            )
        else:
            self.publish_status(False)

            self.get_logger().warn(
                f'PCL pose REJECTED. '
                f'distance={distance:.2f} m > radius={self.current_radius:.2f} m'
            )

    # =========================
    # MAP -> ODOM TF
    # =========================

    def update_map_to_odom_from_accepted_pose(self, map_base_pose: PoseStamped):
        """
        Kabul edilmiş pose:
            T_map_base

        Güncel odom:
            T_odom_base

        Hesap:
            T_map_odom = T_map_base * inverse(T_odom_base)
        """

        try:
            odom_base_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Cannot update map->odom. Missing '
                f'{self.odom_frame}->{self.base_frame}: {ex}'
            )
            return

        # T_map_base
        map_base_x = map_base_pose.pose.position.x
        map_base_y = map_base_pose.pose.position.y
        map_base_yaw = self.yaw_from_quaternion(map_base_pose.pose.orientation)

        # T_odom_base
        odom_base_x = odom_base_tf.transform.translation.x
        odom_base_y = odom_base_tf.transform.translation.y
        odom_base_yaw = self.yaw_from_quaternion(odom_base_tf.transform.rotation)

        # yaw(map->odom) = yaw(map->base) - yaw(odom->base)
        map_odom_yaw = self.normalize_angle(map_base_yaw - odom_base_yaw)

        c = math.cos(map_odom_yaw)
        s = math.sin(map_odom_yaw)

        # t_map_odom = t_map_base - R_map_odom * t_odom_base
        map_odom_x = map_base_x - (c * odom_base_x - s * odom_base_y)
        map_odom_y = map_base_y - (s * odom_base_x + c * odom_base_y)

        self.map_to_odom_x = map_odom_x
        self.map_to_odom_y = map_odom_y
        self.map_to_odom_z = 0.0
        self.map_to_odom_yaw = map_odom_yaw

        self.has_map_to_odom = True
        self.has_relocalized_pose = True

        self.get_logger().warn(
            f'Updated map->odom from accepted PCL pose: '
            f'x={map_odom_x:.3f}, '
            f'y={map_odom_y:.3f}, '
            f'yaw={map_odom_yaw:.3f}'
        )

    def publish_map_to_odom_timer(self):
        if not self.enable_map_odom_tf:
            return

        if not self.has_map_to_odom:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = self.map_to_odom_x
        t.transform.translation.y = self.map_to_odom_y
        t.transform.translation.z = self.map_to_odom_z

        qx, qy, qz, qw = self.quaternion_from_yaw(self.map_to_odom_yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    # =========================
    # CORE FUNCTIONS
    # =========================

    def compute_radius_from_covariance(self, covariance):
        """
        covariance[0] = x variance
        covariance[7] = y variance

        sigma = sqrt(max(var_x, var_y))
        radius = max(base_radius, sigma_multiplier * sigma)
        """

        try:
            var_x = float(covariance[0])
            var_y = float(covariance[7])
        except Exception:
            return self.base_radius

        if var_x < 0.0 or var_y < 0.0:
            return self.base_radius

        sigma = math.sqrt(max(var_x, var_y))

        radius = max(
            self.base_radius,
            self.sigma_multiplier * sigma
        )

        radius = min(radius, self.max_radius)

        return radius

    def publish_circle(
        self,
        center_x: float,
        center_y: float,
        center_z: float,
        radius: float
    ):
        points = []

        for i in range(self.num_points):
            theta = 2.0 * math.pi * (i / self.num_points)

            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            z = center_z

            points.append([x, y, z])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        pc_msg = pc2.create_cloud_xyz32(header, points)
        self.circle_pub.publish(pc_msg)

    def publish_vehicle_cloud(self, pose_stamped: PoseStamped, publisher):
        """
        /pcl_pose noktasında küçük araç şekli yayınlar.
        Dikdörtgen + ön yön oku şeklinde PointCloud2 üretir.
        """

        x0 = pose_stamped.pose.position.x
        y0 = pose_stamped.pose.position.y
        z0 = pose_stamped.pose.position.z

        yaw = self.yaw_from_quaternion(pose_stamped.pose.orientation)

        half_l = self.vehicle_length * 0.5
        half_w = self.vehicle_width * 0.5

        rear_left = [-half_l, half_w]
        front_left = [half_l, half_w]
        front_right = [half_l, -half_w]
        rear_right = [-half_l, -half_w]

        points_2d = []

        # Araç dikdörtgeni
        self.add_line_points(points_2d, rear_left, front_left, self.vehicle_cloud_density)
        self.add_line_points(points_2d, front_left, front_right, self.vehicle_cloud_density)
        self.add_line_points(points_2d, front_right, rear_right, self.vehicle_cloud_density)
        self.add_line_points(points_2d, rear_right, rear_left, self.vehicle_cloud_density)

        # Ön yön oku
        front_center = [half_l, 0.0]
        arrow_tip = [half_l + self.heading_arrow_length, 0.0]
        self.add_line_points(points_2d, front_center, arrow_tip, self.vehicle_cloud_density)

        # Ok ucu
        arrow_left = [half_l + self.heading_arrow_length - 0.35, 0.20]
        arrow_right = [half_l + self.heading_arrow_length - 0.35, -0.20]
        self.add_line_points(points_2d, arrow_tip, arrow_left, 8)
        self.add_line_points(points_2d, arrow_tip, arrow_right, 8)

        # Merkez artısı
        center_left = [0.0, -0.25]
        center_right = [0.0, 0.25]
        center_back = [-0.25, 0.0]
        center_front = [0.25, 0.0]

        self.add_line_points(points_2d, center_left, center_right, 8)
        self.add_line_points(points_2d, center_back, center_front, 8)

        c = math.cos(yaw)
        s = math.sin(yaw)

        points = []

        for lx, ly in points_2d:
            wx = x0 + c * lx - s * ly
            wy = y0 + s * lx + c * ly
            wz = z0

            points.append([wx, wy, wz])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        pc_msg = pc2.create_cloud_xyz32(header, points)
        publisher.publish(pc_msg)

    def add_line_points(self, points_list, p1, p2, num_points):
        x1, y1 = p1
        x2, y2 = p2

        if num_points <= 1:
            points_list.append([x1, y1])
            points_list.append([x2, y2])
            return

        for i in range(num_points):
            t = i / float(num_points - 1)
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            points_list.append([x, y])

    def publish_status(self, accepted: bool):
        msg = Bool()
        msg.data = accepted
        self.status_pub.publish(msg)

    # =========================
    # TF HELPERS
    # =========================

    def transform_pose_to_map(self, pose_stamped: PoseStamped):
        source_frame = pose_stamped.header.frame_id

        if source_frame == '':
            source_frame = self.map_frame
            pose_stamped.header.frame_id = self.map_frame

        if source_frame == self.map_frame:
            return pose_stamped

        # İlk gerçek relokalizasyon yapılmadan önce odom frame'i map gibi kabul et.
        # Bu sayede /odometry/gps merkezini kullanarak ilk filtreleme yapılabilir.
        if (
            source_frame == self.odom_frame
            and not self.has_relocalized_pose
            and self.assume_odom_as_map_before_relocalization
        ):
            out = PoseStamped()
            out.header.stamp = pose_stamped.header.stamp
            out.header.frame_id = self.map_frame
            out.pose = pose_stamped.pose
            return out

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform pose from {source_frame} '
                f'to {self.map_frame}: {ex}'
            )
            return None

        return self.apply_transform(pose_stamped, transform)

    def apply_transform(self, pose_stamped: PoseStamped, transform):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        tq = transform.transform.rotation
        pq = pose_stamped.pose.orientation

        px = pose_stamped.pose.position.x
        py = pose_stamped.pose.position.y
        pz = pose_stamped.pose.position.z

        rx, ry, rz = self.rotate_vector_by_quaternion(
            px, py, pz,
            tq.x, tq.y, tq.z, tq.w
        )

        out.pose.position.x = rx + tx
        out.pose.position.y = ry + ty
        out.pose.position.z = rz + tz

        qx, qy, qz, qw = self.quaternion_multiply(
            tq.x, tq.y, tq.z, tq.w,
            pq.x, pq.y, pq.z, pq.w
        )

        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw

        return out

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quaternion_from_yaw(self, yaw):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return 0.0, 0.0, qz, qw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_multiply(self, x1, y1, z1, w1, x2, y2, z2, w2):
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return x, y, z, w

    def rotate_vector_by_quaternion(self, vx, vy, vz, qx, qy, qz, qw):
        ix, iy, iz, iw = self.quaternion_multiply(
            qx, qy, qz, qw,
            vx, vy, vz, 0.0
        )

        cx, cy, cz, cw = -qx, -qy, -qz, qw

        rx, ry, rz, rw = self.quaternion_multiply(
            ix, iy, iz, iw,
            cx, cy, cz, cw
        )

        return rx, ry, rz


def main(args=None):
    rclpy.init(args=args)

    node = MapAlignedGpsCircle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()