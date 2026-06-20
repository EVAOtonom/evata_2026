#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8, Bool
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
from sklearn.cluster import DBSCAN
import os

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector_with_audio')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.pointcloud_callback,
            10
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self.obstacle_pub = self.create_publisher(Int8, '/obstacle_detected', 10)
        self.brake_pub = self.create_publisher(Bool, '/stm/brake', 10)
        self.get_logger().info("Sesli ve metin uyarılı Obstacle Detector başlatıldı.")

        self.last_warning_zone = None

    def pointcloud_callback(self, msg):
        raw_points = []

        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            distance = math.sqrt(x**2 + y**2)
            angle = math.atan2(y, x) * 180 / math.pi

            # Gürültü filtreleme: ön, belirli mesafe ve yükseklik aralığı
            if -20 <= angle <= 20 and 1.5 <= distance <= 7.0 and -0.2 < z < 1.5:
                raw_points.append([x, y, z, angle, distance])

        if not raw_points:
            self.marker_pub.publish(MarkerArray())
            self.last_warning_zone = None
            # Engel yok, 0 değeri gönder
            self.obstacle_pub.publish(Int8(data=0))
            return

        points_np = np.array(raw_points)

        # Daha sıkı kümeleme: eps düşürüldü, min_samples artırıldı
        clustering = DBSCAN(eps=0.4, min_samples=6).fit(points_np[:, :2])
        labels = clustering.labels_

        unique_labels = set(labels)
        marker_array = MarkerArray()
        detected_zones = set()

        for cluster_id in unique_labels:
            if cluster_id == -1:
                continue  # Gürültü

            cluster_points = points_np[labels == cluster_id]

            # Küme boyutu filtreleme (çok küçükse geç)
            cluster_width = np.ptp(cluster_points[:, 0])
            cluster_depth = np.ptp(cluster_points[:, 1])
            if cluster_width < 0.3 and cluster_depth < 0.3:
                continue

            x_mean = np.mean(cluster_points[:, 0])
            y_mean = np.mean(cluster_points[:, 1])
            z_mean = np.mean(cluster_points[:, 2])
            angle_mean = np.mean(cluster_points[:, 3])
            distance_mean = np.mean(cluster_points[:, 4])

            if -30 <= angle_mean < -10:
                detected_zones.add("sol")
            elif -10 <= angle_mean <= 10:
                detected_zones.add("ön")
            elif 10 < angle_mean <= 30:
                detected_zones.add("sağ")

            self.get_logger().info(f"[ENGEL] x: {x_mean:.2f} m | y: {y_mean:.2f} m | uzaklık: {distance_mean:.2f} m | açı: {angle_mean:.2f}°")

            x_min, y_min, z_min = np.min(cluster_points[:, :3], axis=0)
            x_max, y_max, z_max = np.max(cluster_points[:, :3], axis=0)

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = msg.header.stamp
            marker.ns = "obstacles"
            marker.id = int(cluster_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (x_min + x_max) / 2
            marker.pose.position.y = (y_min + y_max) / 2
            marker.pose.position.z = (z_min + z_max) / 2
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(0.1, x_max - x_min)
            marker.scale.y = max(0.1, y_max - y_min)
            marker.scale.z = max(0.1, z_max - z_min)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker.lifetime.sec = 1

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        if len(marker_array.markers) > 0:
            self.obstacle_pub.publish(Int8(data=1))  # Engel algılandı, 1 gönder
            self.brake_pub.publish(Bool(data=True))
        else:
            self.obstacle_pub.publish(Int8(data=0))
            self.brake_pub.publish(Bool(data=False))

        # Sesli ve metinli uyarı tetikleme
        if detected_zones:
            warning_msg = None
            if "ön" in detected_zones:
                warning_msg = "Dikkat, önünde engel var!"
            elif "sol" in detected_zones:
                warning_msg = "Solunda engel var!"
            elif "sağ" in detected_zones:
                warning_msg = "Sağında engel var!"

            if warning_msg and self.last_warning_zone != warning_msg:
                self.get_logger().warn(warning_msg)
                os.system(f'spd-say "{warning_msg}"')
                self.last_warning_zone = warning_msg
        else:
            self.last_warning_zone = None

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
