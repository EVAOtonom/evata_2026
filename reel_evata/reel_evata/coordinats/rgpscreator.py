import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32
import time

class XYLatLonMapper(Node):
    def __init__(self):
        super().__init__('xy_lat_lon_mapper')

        # Veriler ve zaman damgaları
        self.current_x = None
        self.current_y = None
        self.current_lat = None
        self.current_lon = None

        self.time_xy = None
        self.time_lat = None
        self.time_lon = None

        # Kayıt dosyası
        self.output_file = "/home/otonom/evata_2025/rgps1.txt"
        self.get_logger().info(f"📄 Kayıt dosyası: {self.output_file}")

        # Abonelikler
        self.create_subscription(Point, '/robot_xy', self.amcl_callback, 10)
        self.create_subscription(Float32, '/stm/gps_latitude', self.lat_callback, 10)
        self.create_subscription(Float32, '/stm/gps_longitude', self.lon_callback, 10)

        # Timer ile eşleştirme kontrolü (her 0.5 saniyede bir)
        self.timer = self.create_timer(0.5, self.try_save)

    def amcl_callback(self, msg):
        print(msg)
        self.current_x = msg.x
        self.current_y = msg.y
        self.time_xy = time.time()

    def lat_callback(self, msg):
        self.current_lat = msg.data
        self.time_lat = time.time()

    def lon_callback(self, msg):
        self.current_lon = msg.data
        self.time_lon = time.time()

    def try_save(self):
        now = time.time()
        threshold = 2.0  # saniye içinde gelenleri eşleştir
        # print(self.current_x)
        # print("***************")
        # print(self.current_y)
        # print("***************")
        # print(self.current_lat)
        # print("***************")
        # print(self.current_lon)
        # print("***************")
        if None in (self.current_x, self.current_y, self.current_lat, self.current_lon):
            return
            
        print(self.time_xy,self.time_lat,self.time_lon)

        if all(abs(now - t) < threshold for t in (self.time_xy, self.time_lat, self.time_lon)):
            line = f"{self.current_x:.6f} {self.current_y:.6f} {self.current_lat:.8f} {self.current_lon:.8f}\n"
            with open(self.output_file, 'a') as f:
                f.write(line)
            self.get_logger().info(f"💾 Kayıt: {line.strip()}")

            # Aynı verinin tekrar yazılmasını engelle
            self.current_x = self.current_y = None
            self.current_lat = self.current_lon = None
            self.time_xy = self.time_lat = self.time_lon = None

def main(args=None):
    rclpy.init(args=args)
    node = XYLatLonMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # shutdown daha önce çağrılmadıysa
            rclpy.shutdown()

if __name__ == '__main__':
    main()

