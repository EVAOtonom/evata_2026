#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from visualization_msgs.msg import Marker, MarkerArray

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from nav2_msgs.srv import ClearEntireCostmap


def generate_wall_points(
    cx: float,
    cy: float,
    yaw: float,
    length: float,
    width: float,
    z_min: float = 0.0,
    z_max: float = 1.0,
    xy_res: float = 0.10,
    z_res: float = 0.20,
) -> list:
    """
    Duvarın odom-frame'indeki nokta bulutunu döndürür.
    """
    points = []

    x_local = -length / 2.0
    while x_local <= length / 2.0:

        y_local = -width / 2.0
        while y_local <= width / 2.0:

            # Yerel → odom dönüşümü
            wx = cx + x_local * math.cos(yaw) - y_local * math.sin(yaw)
            wy = cy + x_local * math.sin(yaw) + y_local * math.cos(yaw)

            z = z_min
            while z <= z_max:
                points.append([wx, wy, z])
                z += z_res

            y_local += xy_res
        x_local += xy_res

    return points


class SignDynamicObstacle(Node):

    def __init__(self):
        super().__init__("levha_dinamik_engel")

        # Levha bu mesafeden yakınsa tetikle
        self.sign_trigger_distance = 10.0
        self.barriers = self.load_barriers()

        self.wall_width   =  0.1
        self.wall_z_min   =  0.0
        self.wall_z_max   =  2.0

        # hatası olsa bile yol kenarlarında boşluk kalmasın
        self.wall_edge_margin = 0.5

        # Duvar kaç saniye sonra silinsin
        self.wall_duration = 20.0

        # Her levha tipi için ayrı bayrak
        self.walls: dict = {}
        # walls["sol"]      = {"cx","cy","yaw","timer"}
        # walls["soladonulmez"] = {...}

        # ── TF ────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriber ────────────────────────
        self.sign_sub = self.create_subscription(
            String,
            "/detected_signs",
            self.sign_callback,
            10,
        )

        # ── Publisher ─────────────────────────
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/sign_wall_markers",
            10,
        )

        # Local costmap kaynağı
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            "/sign_obstacles",
            10,
        )

        # Global costmap kaynağı (rolling-window temizlemesini aşmak için)
        self.obstacle_global_pub = self.create_publisher(
            PointCloud2,
            "/sign_obstacles_global",
            10,
        )

        # Costmap temizleme servisleri
        self.clear_local_cli = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.clear_global_cli = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )

        # Aktif duvarları 5 Hz'de yayınla
        self.publish_timer = self.create_timer(0.2, self.publish_all_walls)

        self.get_logger().info("Levha Dinamik Engel Node Baslatildi")

    def load_barriers(self):
        try:
            with open("/home/dilara/ros2_ws/src/evata_sim/evata_sim/barriers.json", "r") as f:
                data = json.load(f)

            self.get_logger().info(
                f"{len(data['junctions'])} kavsak yuklendi"
            )

            return data["junctions"]

        except Exception as e:
            self.get_logger().error(
                f"Barrier dosyasi okunamadi: {e}"
            )
            return []

    # ══════════════════════════════════════════
    # LEVHA CALLBACK
    # ══════════════════════════════════════════

    def sign_callback(self, msg: String):
        try:
            data = json.loads(msg.data)

        except Exception as e:
            self.get_logger().warn(
                f"JSON parse hatasi: {e}"
            )
            return

        junction = self.find_current_junction()

        if junction is None:
            return

#LEVHA TETİKLEME
        sign_map = {
            "sol": ["front_barrier", "right_barrier"],
            "soladonulmez": ["left_barrier"],
            "sag": ["front_barrier", "left_barrier"],
        }

        for sign_key, barrier_names in sign_map.items():

            if sign_key not in data:
                continue

            wall_keys = [
                f"{junction['id']}_{sign_key}_{bname}" for bname in barrier_names
            ]

            if all(k in self.walls for k in wall_keys):
                continue

            other_active = any(
                k.startswith(f"{junction['id']}_") and k not in wall_keys
                for k in self.walls
            )
            if other_active:
                self.get_logger().warn(
                    f"Junction {junction['id']} icin baska bir barikat zaten aktif, [{sign_key}] atlandi."
                )
                continue

            distance = float(data[sign_key])

            if distance > self.sign_trigger_distance:
                continue

            for wall_key, barrier_name in zip(wall_keys, barrier_names):

                if wall_key in self.walls:
                    continue

                barrier = junction.get(barrier_name)

                if barrier is None:
                    self.get_logger().warn(
                        f"Junction {junction['id']} icin {barrier_name} tanimli degil, "
                        f"[{sign_key}] icin bu duvar atlandi."
                    )
                    continue

                self.get_logger().info(
                    f"[{sign_key}] bulundu -> {barrier_name} aktif"
                )

                self.create_wall(
                    wall_key,
                    barrier,
                )

    def find_current_junction(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time(),
            )

        except Exception as e:
            self.get_logger().warn(
                f"TF lookup hatasi: {e}"
            )
            return None

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        for junction in self.barriers:

            area = junction["trigger_area"]

            if (
                area["xmin"] <= robot_x <= area["xmax"]
                and
                area["ymin"] <= robot_y <= area["ymax"]
            ):
                self.get_logger().info(
                    f"Kavsak bulundu: {junction['id']} "
                    f"robot=({robot_x:.2f},{robot_y:.2f})"
                )
                return junction

        return None

    # ══════════════════════════════════════════
    # MAP -> ODOM DÖNÜŞÜMÜ
    # ══════════════════════════════════════════

    def transform_map_point_to_odom(self, x: float, y: float):
        """
        barriers.json'daki koordinatlar map frame'inde sabittir.
        Bu fonksiyon onları anlık map->odom TF'ine göre odom frame'ine
        çevirir, böylece robotun odom orijini nerede başlarsa başlasın
        duvar haritadaki gerçek yerinde kalır.
        """
        try:
            map_to_odom = self.tf_buffer.lookup_transform(
                "odom",
                "map",
                rclpy.time.Time(),
            )

        except Exception as e:
            self.get_logger().warn(
                f"map->odom TF alinamadi: {e}"
            )
            return None

        tx = map_to_odom.transform.translation.x
        ty = map_to_odom.transform.translation.y

        q = map_to_odom.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        ox = tx + x * math.cos(yaw) - y * math.sin(yaw)
        oy = ty + x * math.sin(yaw) + y * math.cos(yaw)

        return ox, oy

    # ══════════════════════════════════════════
    # DUVAR OLUŞTUR
    # ══════════════════════════════════════════

    def create_wall(self, sign_key: str, barrier: dict):

        p1 = self.transform_map_point_to_odom(barrier["x1"], barrier["y1"])
        p2 = self.transform_map_point_to_odom(barrier["x2"], barrier["y2"])

        if p1 is None or p2 is None:
            self.get_logger().warn(
                f"[{sign_key}] map->odom TF hazir degil, duvar olusturulamadi."
            )
            return

        x1, y1 = p1
        x2, y2 = p2

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        dx = x2 - x1
        dy = y2 - y1

        length = math.hypot(dx, dy)
        length += 2.0 * self.wall_edge_margin

        yaw = math.atan2(dy, dx)

        def on_wall_timeout(key=sign_key):
            self.remove_wall(key)

        timer = self.create_timer(
            self.wall_duration,
            on_wall_timeout
        )

        self.walls[sign_key] = {
            "cx": cx,
            "cy": cy,
            "yaw": yaw,
            "length": length,
            "timer": timer,
        }

        self.get_logger().info(
            f"BARIYER [{sign_key}] "
            f"merkez=({cx:.2f},{cy:.2f}) "
            f"uzunluk={length:.2f}m"
        )
        self.get_logger().warn(
            f"Barrier ciziliyor: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f})"
        )

    # ══════════════════════════════════════════
    # AKTİF DUVARLARI YAYINLA
    # ══════════════════════════════════════════

    def publish_all_walls(self):
        if not self.walls:
            return

        all_points = []
        marker_array = MarkerArray()

        for idx, (sign_key, wall) in enumerate(self.walls.items()):
            # Marker
            marker_array.markers.append(
                self._build_marker(idx, wall, sign_key)
            )

            # Nokta bulutu
            pts = generate_wall_points(
                wall["cx"],
                wall["cy"],
                wall["yaw"],
                wall["length"],
                self.wall_width,
                self.wall_z_min,
                self.wall_z_max,
            )
            all_points.extend(pts)

        # Marker yayınla
        self.marker_pub.publish(marker_array)

        if not all_points:
            return

        header = Header()
        header.frame_id = "odom"
        header.stamp    = self.get_clock().now().to_msg()

        cloud = point_cloud2.create_cloud_xyz32(header, all_points)

        # Hem local hem global costmap'e gönder
        self.obstacle_pub.publish(cloud)
        self.obstacle_global_pub.publish(cloud)

    # ══════════════════════════════════════════
    # MARKER OLUŞTUR
    # ══════════════════════════════════════════

    def _build_marker(self, idx: int, wall: dict, label: str) -> Marker:
        wall_q = quaternion_from_euler(0.0, 0.0, wall["yaw"])

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns              = "sign_obstacles"
        marker.id              = idx

        marker.type   = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = wall["cx"]
        marker.pose.position.y = wall["cy"]
        marker.pose.position.z = self.wall_z_max / 2.0  # zemin ortalı: 0→3m arası

        marker.pose.orientation.x = wall_q[0]
        marker.pose.orientation.y = wall_q[1]
        marker.pose.orientation.z = wall_q[2]
        marker.pose.orientation.w = wall_q[3]

        marker.scale.x = wall["length"]
        marker.scale.y = self.wall_width
        marker.scale.z = self.wall_z_max - self.wall_z_min

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.85

        # Lifetime sonsuz (0) — kendimiz siliyoruz
        marker.lifetime.sec     = 0
        marker.lifetime.nanosec = 0

        return marker

    # ══════════════════════════════════════════
    # DUVAR SİL
    # ══════════════════════════════════════════

    def remove_wall(self, sign_key: str):
        if sign_key not in self.walls:
            return

        wall = self.walls.pop(sign_key)

        # Timer'ı iptal et
        try:
            wall["timer"].cancel()
        except Exception:
            pass

        # ama marker'ı açıkça DELETE ile kaldır
        marker_array = MarkerArray()
        del_all = Marker()
        del_all.header.frame_id = "odom"
        del_all.header.stamp    = self.get_clock().now().to_msg()
        del_all.ns              = "sign_obstacles"
        del_all.action          = Marker.DELETEALL
        marker_array.markers.append(del_all)
        self.marker_pub.publish(marker_array)

        # Costmap'i temizle: bos nokta bulutu gonder
        # Birden fazla kez gonder ki costmap kesinlikle temizlesin.
        header = Header()
        header.frame_id = "odom"
        header.stamp    = self.get_clock().now().to_msg()
        empty_cloud = point_cloud2.create_cloud_xyz32(header, [])

        for _ in range(5):
            self.obstacle_pub.publish(empty_cloud)
            self.obstacle_global_pub.publish(empty_cloud)

        # Costmap'i servis ile zorla temizle (raytrace ile temizlenmeyen kalıntı hücreler için)
        self._call_clear_costmaps()

        self.get_logger().info(f"DUVAR SILINDI [{sign_key}] — costmap temizlendi")

    def _call_clear_costmaps(self):
        req = ClearEntireCostmap.Request()

        if self.clear_local_cli.service_is_ready():
            self.clear_local_cli.call_async(req)
        else:
            self.get_logger().warn("local_costmap clear servisi hazir degil")

        if self.clear_global_cli.service_is_ready():
            self.clear_global_cli.call_async(req)
        else:
            self.get_logger().warn("global_costmap clear servisi hazir degil")

    def destroy_node(self):
        for wall in self.walls.values():
            try:
                wall["timer"].cancel()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SignDynamicObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()