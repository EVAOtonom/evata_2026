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
    Duvar yaw açısına paralel, length × width × height boyutlarında.
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
        self.sign_trigger_distance = 5.0

        #önüne duvar: forward=15m, lateral=0
        self.front_wall_forward = 15.0
        self.front_wall_lateral = 0.0
        
        #sag duvar
        self.right_wall_forward = 7.0
        self.right_wall_lateral = -4.0

        #soluna duvar: forward=7m, lateral=+4m (sol)
        self.left_wall_forward  = 7.0
        self.left_wall_lateral  = 4.0

        # Duvar 15m × 1m × 3m
        self.wall_length  = 15.0
        self.wall_width   =  1.0
        self.wall_z_min   =  0.0
        self.wall_z_max   =  3.0  

        # Duvar kaç saniye sonra silinsin
        self.wall_duration = 40.0

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

        # Aktif duvarları 5 Hz'de yayınla
        self.publish_timer = self.create_timer(0.2, self.publish_all_walls)

        self.get_logger().info("Levha Dinamik Engel Node Baslatildi")

    # ══════════════════════════════════════════
    # LEVHA CALLBACK
    # ══════════════════════════════════════════

    def sign_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"JSON parse hatasi: {e}")
            return

        sign_configs = {
            "sol": {
                "forward": self.front_wall_forward,
                "lateral": self.front_wall_lateral,
            },
            "soladonulmez": {
                "forward": self.left_wall_forward,
                "lateral": self.left_wall_lateral,
            },
            "sag": {
                "forward": self.front_wall_forward,
                "lateral": self.front_wall_lateral,
            },
            "sagadonulmez": {
                "forward": self.right_wall_forward,
                "lateral": self.right_wall_lateral,
            },
            "girisiyok": {
                "forward": self.front_wall_forward,
                "lateral": self.front_wall_lateral,
            },                                    
        }

        for sign_key, cfg in sign_configs.items():

            if sign_key not in data:
                continue

            # Bu levha için duvar zaten oluşturulduysa atla
            if sign_key in self.walls:
                continue

            distance = float(data[sign_key])

            if distance > self.sign_trigger_distance:
                continue

            self.get_logger().info(
                f"[{sign_key}] levhasi algilandi, mesafe={distance:.2f}m → duvar olusturuluyor"
            )
            self.create_wall(sign_key, cfg["forward"], cfg["lateral"])
            
            if sign_key == "sol" and "soladon_sag" not in self.walls:
                self.get_logger().info("+Sag duvar da olusturuluyor")
                self.create_wall(
                    "soladon_sag",
                    self.right_wall_forward,
                    self.right_wall_lateral,
                )
            if sign_key == "sag" and "sagadon_sol" not in self.walls:
                self.get_logger().info("+Sol duvar da olusturuluyor")
                self.create_wall(
                    "sagadon_sol",
                    self.left_wall_forward,
                    self.left_wall_lateral,
                )                
            


    # ══════════════════════════════════════════
    # DUVAR OLUŞTUR
    # ══════════════════════════════════════════

    def create_wall(self, sign_key: str, forward_offset: float, lateral_offset: float):

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom",
                "base_footprint",
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup hatasi: {e}")
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q  = transform.transform.rotation

        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # Araç koordinat sisteminde offset → odom frame'e çevir
        # ROS konvansiyonu: +x = ileri, +y = SOL
        # lateral_offset pozitif → sol, negatif → sağ
        cx = (
            tx
            + forward_offset * math.cos(yaw)
            - lateral_offset * math.sin(yaw)
        )
        cy = (
            ty
            + forward_offset * math.sin(yaw)
            + lateral_offset * math.cos(yaw)
        )

        # soladon     → duvar ARAÇA DIK (önü keser, düz gitmeyi engeller)
        # soladonulmez → duvar ARAÇA PARALEL (sol seridi keser)
        if sign_key == "sag" or sign_key == "sol":
            wall_yaw = yaw + math.pi / 2.0          
        else:
            wall_yaw = yaw


        # Durumu kaydet — bu koordinatlar artık SABIT, araçla hareket etmez
        def on_wall_timeout(key=sign_key):
            self.remove_wall(key)
 
        timer = self.create_timer(self.wall_duration, on_wall_timeout)


        self.walls[sign_key] = {
            "cx":    cx,
            "cy":    cy,
            "yaw":   wall_yaw,
            "timer": timer,
        }

        self.get_logger().info(
            f"DUVAR KILITLEDI [{sign_key}] | "
            f"odom=({cx:.2f}, {cy:.2f}) | "
            f"wall_yaw={math.degrees(wall_yaw):.1f}° | "
            f"{self.wall_duration:.0f}s sonra silinecek"
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
                self.wall_length,
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

        marker.scale.x = self.wall_length
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
 
        self.get_logger().info(f"DUVAR SILINDI [{sign_key}] — costmap temizlendi")



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
