import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
from shapely.geometry import Polygon

class MultiPolygonPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('multi_polygon_pointcloud_publisher')

        self.pc_pub = self.create_publisher(PointCloud2, '/lane_pointcloud', 10)

        # === POLYGON LİSTESİ BURAYA ===
        # Her alt liste bir polygon: [(x1,y1), (x2,y2), ...]
        self.polygons = [
            [(4.54, 1.4),(14.4, -0.4),(17.6, 16.7),(16.3, 18.1),(9.2, 19.2),(7.8, 17.7)],
            [(34.3, -4.55),(35.7,-3.64),(37.4,4.68),(35.9,6.25),(24.0,8.33),(22.3,7.16),(20.4,0.03),(21.8,-2.17)],
            [(41.7, -5.0),(43.0,-6.8),(54.0,-8.81),(55.0,-7.9),(56.7,0.8),(55.7,1.94),(45.0,4.02),(43.5,3.26)],
            [(61.4,-8.7),(62.1,-10.8),(68.2,-11.8), (69.5,-10.5),(71.0,-3.94),(73.6,-2.52),(74.4,0.6),(71.0,-1.3),(63.8,0.17),(62.8,-0.8)],
            [(64.6,8.34),(65.7,5.84),(70.6,4.63),(73.0,5.84),(76.2,24.2),(69.4,25.5),(67.8,24.1)],
            [(61.3,21.6),(60.7,27.4),(55.3,28.5),(47.8,24.1),(45.1,11.8),(45.7,10.3),(57.2,8.11),(58.2,9.81)],
            [(38.7,11.8),(36.6,14.8),(38.7,23.6),(41.4,25.5),(36.5,32.6),(28.7,33.8),(27.0,33.0),(23.5,16.3),(24.3,14.6)],
            [(69.6,33.4),(70.7,31.4),(75.6,30.5),(77.9,31.3),(79.5,39.3),(78.6,41.1),(73.5,42.2),(71.3,40.9)],
            [(51.5,42.8),(56.6,34.4),(61.6,33.3),(63.5,34.2),(65.1,42.5),(63.3,44.2),(53.9,46.3),(51.8,45.3)],
            [(44.5,42.4),(45.5,46.3),(44.2,48.0),(32.9,50.7),(30.8,49.9),(28.9,42.4),(29.8,39.9),(37.1,38.4)],
            [(48.6,32.6),(46.3,30.5),(43.9,31.8),(44.135,35.1),(46.2,35.9)]
        ]

        self.grid_resolution = 0.5  # Nokta aralığı (metre)
        self.z_height = 0.1         # Z koordinatı (metre)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)

    def publish_pointcloud(self):
        all_points = []

        for vertices in self.polygons:
            poly = Polygon(vertices)
            min_x, min_y, max_x, max_y = poly.bounds
            x_range = np.arange(min_x, max_x + self.grid_resolution, self.grid_resolution)
            y_range = np.arange(min_y, max_y + self.grid_resolution, self.grid_resolution)

            for x in x_range:
                for y in y_range:
                    # Küçük kare ile noktanın polygon içinde olup olmadığını kontrol et
                    if poly.contains(Polygon([(x, y), (x+0.001, y), (x, y+0.001)])):
                        all_points.append((np.float32(x), np.float32(y), np.float32(self.z_height)))

        # PointCloud2 oluşturma
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pc2_msg = point_cloud2.create_cloud(header, fields, all_points)
        self.pc_pub.publish(pc2_msg)

        self.get_logger().info(f"Published {len(all_points)} points from {len(self.polygons)} polygons")

def main(args=None):
    rclpy.init(args=args)
    node = MultiPolygonPointCloudPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
