#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_lat = self.create_publisher(Float64, '/stm/gps_latitude', 10)
        self.publisher_long = self.create_publisher(Float64, '/stm/gps_longitude', 10)

        timer_period = 1.0  # saniyede bir yayın
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        lat_msg = Float64()
        long_msg = Float64()

        lat_msg.data = 41.056660
        long_msg.data = 28.819996

        self.publisher_lat.publish(lat_msg)
        self.get_logger().info(f'Latitude published: {lat_msg.data}')

        self.publisher_long.publish(long_msg)
        self.get_logger().info(f'Longitude published: {long_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
