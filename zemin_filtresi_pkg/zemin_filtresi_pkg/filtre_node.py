import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ZeminFiltresi(Node):
    def __init__(self):
        super().__init__('zemin_filtresi_node')
        
        # 3D Lidara ABONE OL
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',  # Aracındaki 3D lidar topiği
            self.lidar_callback,
            10)
            
        # 2. Temizlenmiş veriyi yeni bir isimle YAYINLA
        self.publisher = self.create_publisher(
            PointCloud2, 
            '/rslidar_filtered', # Haritalamanın artık dinleyeceği yeni topic
            10)
            
        self.get_logger().info('Filtre devrede! Yerdeki noktalar anlık olarak siliniyor...')

    def lidar_callback(self, msg):
        # Lidar'dan gelen noktaları okunabilir hale getir
        points = pc2.read_points(msg, skip_nans=True)
        
        filtered_points = []
        for p in points:
            # Z ekseni (p[2]) yüksekliği temsil eder. 
            # Lidar arabanın tepesinde olduğu için zemin eksilerdedir.
            # EĞER nokta Lidar'dan 0.8 metre veya daha aşağıdaysa (zeminse) SİL.
            # Değilse (duvar, araç, engelse) YENİ LİSTEYE EKLE.
            if p[2] > -1.0:  
                filtered_points.append(p)
                
        # Temizlenen noktalardan yeni bir ROS 2 mesajı paketle
        filtered_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
        
        # Ve sisteme geri fırlat!
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZeminFiltresi()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
