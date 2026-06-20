#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os

class ClickSaver(Node):
    def __init__(self):
        super().__init__('click_saver')
        
        self.file_path = os.path.expanduser('~/evata_2025/ros2_ws/clicked_points.yaml')
        self.points = []

        # Dosya varsa eski kayıtları oku
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    data = yaml.safe_load(f)
                    if data:
                        self.points = data
                        self.get_logger().info(f"Eski dosyadan {len(self.points)} nokta yüklendi.")
            except Exception as e:
                self.get_logger().warn(f"Eski dosya okunamadı: {e}")

        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.callback,
            10
        )

        self.get_logger().info("ClickSaver başlatıldı.")
        self.get_logger().info("RViz'de 'Publish Point' tool'u ile tıklayın.")
        self.get_logger().info(f"Dosya yolu: {self.file_path}")

    def callback(self, msg):
        x, y = msg.point.x, msg.point.y
        self.points.append({'x': float(x), 'y': float(y)})

        try:
            with open(self.file_path, 'w') as f:
                yaml.dump(self.points, f)
            self.get_logger().info(f"Nokta kaydedildi: ({x:.2f}, {y:.2f}) -> {self.file_path}")
        except Exception as e:
            self.get_logger().error(f"Dosya yazma hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ClickSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

