#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import os
import math
import yaml
import numpy as np

class LaneStripPublisher(Node):
    def __init__(self):
        super().__init__('lane_strip_publisher')
        self.file_path = os.path.expanduser('~/evata_2025/ros2_ws/clicked_points.yaml')
        self.publisher_ = self.create_publisher(PointCloud2, '/lane_pointcloud', 10)
        self.timer = self.create_timer(1.0, self.publish_lane_strips)
        self.z_height = 0.1          # Sabit z koordinatı
        self.strip_width = 0.02      # Şerit genişliği (metre)
        self.points_per_segment = 50 # Her iki nokta arasında kaç nokta olacak (kalınlığı yaymak için)

        self.get_logger().info(f"Lane strip publisher started, reading from: {self.file_path}")

    def read_points(self):
        points = []
        if not os.path.exists(self.file_path):
            self.get_logger().warn(f"File not found: {self.file_path}")
            return points
        try:
            with open(self.file_path, 'r') as f:
                data = yaml.safe_load(f)
                if not data:
                    self.get_logger().warn("Empty or invalid yaml file")
                    return points
                for item in data:
                    points.append( (float(item['x']), float(item['y'])) )
        except Exception as e:
            self.get_logger().error(f"Error reading yaml file: {e}")
        return points

    def interpolate_segment(self, p1, p2, num_points):
        """p1-p2 arasında eşit aralıklarla x,y interpolasyonu"""
        x_vals = np.linspace(p1[0], p2[0], num_points)
        y_vals = np.linspace(p1[1], p2[1], num_points)
        return list(zip(x_vals, y_vals))

    def compute_normal_vector(self, p1, p2):
        """p1->p2 vektörüne dik birim vektör hesapla (2D)"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length == 0:
            return (0,0)
        nx = -dy / length
        ny = dx / length
        return (nx, ny)

    def publish_lane_strips(self):
        base_points = self.read_points()
        if len(base_points) < 2:
            self.get_logger().warn("En az 2 nokta gerekli.")
            return

        all_points = []

        # Her 2 nokta arasına şerit oluştur
        for i in range(0, len(base_points)-1, 2):  # 2 şerli atla, çünkü 2 nokta bir şerit
            if i+1 >= len(base_points):
                break
            p1 = base_points[i]
            p2 = base_points[i+1]

            line_points = self.interpolate_segment(p1, p2, self.points_per_segment)
            normal = self.compute_normal_vector(p1, p2)

            # Şerit genişliği yarısı:
            half_width = self.strip_width / 2

            # Her çizgi noktası için, dik yönde 3 paralel nokta koy (center ve yanlar)
            for (x, y) in line_points:
                # center
                all_points.append( (float(x), float(y), self.z_height) )


        # PointCloud2 mesaji oluştur
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = point_cloud2.create_cloud(header, fields, all_points)
        self.publisher_.publish(cloud_msg)
        self.get_logger().info(f"Published {len(all_points)} points forming { (len(base_points)//2) } lane strips")

def main(args=None):
    rclpy.init(args=args)
    node = LaneStripPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
