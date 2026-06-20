#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from datetime import datetime
import os


class GPSLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')

        # GPS verileri
        self.latitude = None
        self.longitude = None

        # Dosya yolu klasörü
        base_dir = "/home/otonom/evata_2025/ros2_ws/src/reel_evata/reel_evata/coordinats"
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.file_path = os.path.join(base_dir, f"kordinatlar_{timestamp}.txt")

        # Topic abonelikleri
        self.subscription_lat = self.create_subscription(
            Float32,
            '/stm/gps_latitude',
            self.latitude_callback,
            10
        )
        self.subscription_long = self.create_subscription(
            Float32,
            '/stm/gps_longitude',
            self.longitude_callback,
            10
        )

        # Kayıt zamanlayıcısı (1 Hz)
        self.timer = self.create_timer(1.0, self.log_coordinates)

        self.get_logger().info(f"GPS Logger başlatıldı. Dosya: {self.file_path}")

    def latitude_callback(self, msg):
        self.latitude = msg.data

    def longitude_callback(self, msg):
        self.longitude = msg.data

    def log_coordinates(self):
        if self.latitude is not None and self.longitude is not None:
            try:
                with open(self.file_path, "a") as file:
                    file.write(f"({self.latitude}, {self.longitude}),\n")
                self.get_logger().info(f"Kayıt: ({self.latitude}, {self.longitude})")
            except Exception as e:
                self.get_logger().error(f"Dosyaya yazılamadı: {e}")
        else:
            self.get_logger().warn("GPS verisi alınamadı, yazılamadı.")


def main(args=None):
    rclpy.init(args=args)
    node = GPSLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
