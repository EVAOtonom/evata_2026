#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
import sensor_msgs_py.point_cloud2 as pc2


class InitialPoseLocalRelocalizer(Node):
    """GPS kullanmadan, /initialpose çevresinde periyodik lidar relokalizasyonu."""

    def __init__(self):
        super().__init__('initialpose_local_relocalizer')

        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('pcl_pose_topic', '/pcl_pose')
        self.declare_parameter('accepted_pose_topic', '/pcl_pose_accepted')
        self.declare_parameter('status_topic', '/pcl_pose_in_initial_area')
        self.declare_parameter('search_area_cloud_topic', '/initialpose_search_area')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # 20.0 => 20 m x 20 m kare alan. 20 m² istersen 4.4721 yap.
        self.declare_parameter('search_area_size', 10.0)
        self.declare_parameter('control_period_sec', 10.0)
        self.declare_parameter('tf_publish_rate', 30.0)
        self.declare_parameter('check_yaw_difference', True)
        self.declare_parameter('max_yaw_difference_deg', 35.0)
        self.declare_parameter('publish_identity_before_first_match', False)

        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.pcl_pose_topic = self.get_parameter('pcl_pose_topic').value
        self.accepted_pose_topic = self.get_parameter('accepted_pose_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.search_area_cloud_topic = self.get_parameter('search_area_cloud_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.search_area_size = float(self.get_parameter('search_area_size').value)
        self.control_period_sec = float(self.get_parameter('control_period_sec').value)
        self.tf_publish_rate = float(self.get_parameter('tf_publish_rate').value)
        self.check_yaw_difference = bool(self.get_parameter('check_yaw_difference').value)
        self.max_yaw_difference = math.radians(
            float(self.get_parameter('max_yaw_difference_deg').value)
        )
        self.publish_identity_before_first_match = bool(
            self.get_parameter('publish_identity_before_first_match').value
        )

        if self.search_area_size <= 0.0:
            raise ValueError('search_area_size must be greater than zero')
        if self.control_period_sec <= 0.0:
            raise ValueError('control_period_sec must be greater than zero')
        if self.tf_publish_rate <= 0.0:
            raise ValueError('tf_publish_rate must be greater than zero')

        self.initial_pose: Optional[PoseStamped] = None
        self.latest_pcl_pose: Optional[PoseStamped] = None

        self.has_map_to_odom = self.publish_identity_before_first_match
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_yaw = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10,
        )
        self.pcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pcl_pose_topic,
            self.pcl_pose_callback,
            10,
        )

        self.accepted_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.accepted_pose_topic,
            10,
        )
        self.status_pub = self.create_publisher(Bool, self.status_topic, 10)
        self.search_area_pub = self.create_publisher(
            PointCloud2,
            self.search_area_cloud_topic,
            10,
        )

        self.control_timer = self.create_timer(
            self.control_period_sec,
            self.control_timer_callback,
        )
        self.tf_timer = self.create_timer(
            1.0 / self.tf_publish_rate,
            self.publish_map_to_odom,
        )

        self.get_logger().info('GPS-free initialpose relocalizer started')
        self.get_logger().info(
            f'Search area: {self.search_area_size:.1f} x {self.search_area_size:.1f} m'
        )
        self.get_logger().info(
            f'PCL pose will be evaluated every {self.control_period_sec:.1f} seconds'
        )

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        pose_map = self.transform_pose_to_map(pose)
        if pose_map is None:
            return

        self.initial_pose = pose_map
        self.latest_pcl_pose = None
        self.publish_search_area()

        yaw = self.yaw_from_quaternion(pose_map.pose.orientation)
        self.get_logger().warn(
            f'/initialpose accepted: x={pose_map.pose.position.x:.3f}, '
            f'y={pose_map.pose.position.y:.3f}, yaw={math.degrees(yaw):.2f} deg'
        )

    def pcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        pose_map = self.transform_pose_to_map(pose)
        if pose_map is not None:
            # Sürekli gelen PCL sonuçlarından sadece en günceli saklanır.
            self.latest_pcl_pose = pose_map

    def control_timer_callback(self):
        if self.initial_pose is None:
            self.publish_status(False)
            self.get_logger().warn('Periodic check skipped: /initialpose is missing')
            return

        if self.latest_pcl_pose is None:
            self.publish_status(False)
            self.get_logger().warn('Periodic check skipped: /pcl_pose is missing')
            return

        center_x = self.initial_pose.pose.position.x
        center_y = self.initial_pose.pose.position.y
        pcl_x = self.latest_pcl_pose.pose.position.x
        pcl_y = self.latest_pcl_pose.pose.position.y

        half_size = self.search_area_size * 0.5
        dx = pcl_x - center_x
        dy = pcl_y - center_y

        inside_square = abs(dx) <= half_size and abs(dy) <= half_size

        initial_yaw = self.yaw_from_quaternion(self.initial_pose.pose.orientation)
        pcl_yaw = self.yaw_from_quaternion(self.latest_pcl_pose.pose.orientation)
        yaw_error = abs(self.normalize_angle(pcl_yaw - initial_yaw))
        yaw_ok = (
            not self.check_yaw_difference
            or yaw_error <= self.max_yaw_difference
        )

        accepted = inside_square and yaw_ok
        self.publish_status(accepted)

        if not accepted:
            self.get_logger().warn(
                f'PCL pose rejected: dx={dx:.2f} m, dy={dy:.2f} m, '
                f'yaw_error={math.degrees(yaw_error):.2f} deg'
            )
            return

        if not self.update_map_to_odom(self.latest_pcl_pose):
            return

        accepted_msg = PoseWithCovarianceStamped()
        accepted_msg.header.stamp = self.get_clock().now().to_msg()
        accepted_msg.header.frame_id = self.map_frame
        accepted_msg.pose.pose = self.latest_pcl_pose.pose
        accepted_msg.pose.covariance[0] = 0.25
        accepted_msg.pose.covariance[7] = 0.25
        accepted_msg.pose.covariance[35] = 0.06853891909122467
        self.accepted_pose_pub.publish(accepted_msg)

        self.get_logger().warn(
            f'PCL pose accepted; map->odom updated. '
            f'dx={dx:.2f} m, dy={dy:.2f} m, '
            f'yaw_error={math.degrees(yaw_error):.2f} deg'
        )

    def update_map_to_odom(self, map_base_pose: PoseStamped) -> bool:
        try:
            odom_base_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Cannot update map->odom; missing '
                f'{self.odom_frame}->{self.base_frame}: {ex}'
            )
            return False

        map_base_x = map_base_pose.pose.position.x
        map_base_y = map_base_pose.pose.position.y
        map_base_yaw = self.yaw_from_quaternion(map_base_pose.pose.orientation)

        odom_base_x = odom_base_tf.transform.translation.x
        odom_base_y = odom_base_tf.transform.translation.y
        odom_base_yaw = self.yaw_from_quaternion(odom_base_tf.transform.rotation)

        map_odom_yaw = self.normalize_angle(map_base_yaw - odom_base_yaw)
        c = math.cos(map_odom_yaw)
        s = math.sin(map_odom_yaw)

        self.map_to_odom_x = map_base_x - (c * odom_base_x - s * odom_base_y)
        self.map_to_odom_y = map_base_y - (s * odom_base_x + c * odom_base_y)
        self.map_to_odom_yaw = map_odom_yaw
        self.has_map_to_odom = True

        self.publish_map_to_odom()
        return True

    def publish_map_to_odom(self):
        # TF'nin kendisi sürekli yayınlanmalıdır. Yalnızca değeri 10 saniyede
        # bir kontrol sonucunda güncellenir. Aksi hâlde TF extrapolation oluşur.
        if not self.has_map_to_odom:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = self.map_to_odom_x
        transform.transform.translation.y = self.map_to_odom_y
        transform.transform.translation.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(self.map_to_odom_yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

    def publish_search_area(self):
        if self.initial_pose is None:
            return

        center_x = self.initial_pose.pose.position.x
        center_y = self.initial_pose.pose.position.y
        center_z = self.initial_pose.pose.position.z
        half = self.search_area_size * 0.5
        corners = [(-half, -half), (half, -half), (half, half), (-half, half)]

        points = []
        samples_per_edge = max(20, int(self.search_area_size * 5))

        for edge in range(4):
            x1, y1 = corners[edge]
            x2, y2 = corners[(edge + 1) % 4]
            for i in range(samples_per_edge):
                t = i / float(samples_per_edge - 1)
                points.append([
                    center_x + x1 + t * (x2 - x1),
                    center_y + y1 + t * (y2 - y1),
                    center_z,
                ])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        self.search_area_pub.publish(pc2.create_cloud_xyz32(header, points))

    def publish_status(self, accepted: bool):
        msg = Bool()
        msg.data = accepted
        self.status_pub.publish(msg)

    def transform_pose_to_map(self, pose: PoseStamped) -> Optional[PoseStamped]:
        source_frame = pose.header.frame_id or self.map_frame
        pose.header.frame_id = source_frame

        if source_frame == self.map_frame:
            return pose

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {source_frame} to {self.map_frame}: {ex}'
            )
            return None

        return self.apply_transform(pose, transform)

    def apply_transform(self, pose: PoseStamped, transform) -> PoseStamped:
        output = PoseStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.map_frame

        tq = transform.transform.rotation
        pq = pose.pose.orientation
        px, py, pz = self.rotate_vector_by_quaternion(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            tq.x,
            tq.y,
            tq.z,
            tq.w,
        )

        output.pose.position.x = px + transform.transform.translation.x
        output.pose.position.y = py + transform.transform.translation.y
        output.pose.position.z = pz + transform.transform.translation.z

        qx, qy, qz, qw = self.quaternion_multiply(
            tq.x, tq.y, tq.z, tq.w,
            pq.x, pq.y, pq.z, pq.w,
        )
        output.pose.orientation.x = qx
        output.pose.orientation.y = qy
        output.pose.orientation.z = qz
        output.pose.orientation.w = qw
        return output

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def quaternion_from_yaw(yaw):
        return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def quaternion_multiply(x1, y1, z1, w1, x2, y2, z2, w2):
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    def rotate_vector_by_quaternion(self, vx, vy, vz, qx, qy, qz, qw):
        ix, iy, iz, iw = self.quaternion_multiply(
            qx, qy, qz, qw, vx, vy, vz, 0.0
        )
        rx, ry, rz, _ = self.quaternion_multiply(
            ix, iy, iz, iw, -qx, -qy, -qz, qw
        )
        return rx, ry, rz


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseLocalRelocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()