import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int8
from datetime import datetime
import os

class GPSAndSignLogger(Node):
    def __init__(self):
        super().__init__('gps_and_sign_logger')

        # Dosya yolu ve isimlendirme
        log_dir = os.path.join(os.path.expanduser('~'), 'ros2_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file_path = os.path.join(log_dir, 'gps_sign_log.txt')

        # Verileri tutmak için geçici değişkenler
        self.latest_lat = None
        self.latest_lon = None
        self.latest_sign = None
        self.latest_angle = None
        self.latest_odom = None
        self.latest_motor = None

        # Topic abonelikleri
        self.create_subscription(Float32, '/stm/gps_latitude', self.lat_callback, 10)
        self.create_subscription(Float32, '/stm/gps_longitude', self.lon_callback, 10)
        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)
        self.create_subscription(Int8, '/stm/read_wheel_angle', self.angle_callback, 10)
        self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)
        self.create_subscription(Int8, '/stm/motor_power', self.motor_callback, 10)

        self.get_logger().info(f"Log dosyası: {self.log_file_path}")

    def lat_callback(self, msg):
        self.latest_lat = msg.data
        self.write_log()

    def lon_callback(self, msg):
        self.latest_lon = msg.data
        self.write_log()

    def sign_callback(self, msg):
        self.latest_sign = msg.data
        self.write_log()

    def angle_callback(self, msg):
        self.latest_angle = msg.data
        self.write_log()

    def odom_callback(self, msg):
        self.latest_odom = msg.data
        self.write_log()

    def motor_callback(self, msg):
        self.latest_motor = msg.data
        self.write_log()

    def write_log(self):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # None kontrolü ve uygun formatlama
        lat_str = f"{self.latest_lat:.8f}" if self.latest_lat is not None else "None"
        lon_str = f"{self.latest_lon:.8f}" if self.latest_lon is not None else "None"
        sign_str = self.latest_sign if self.latest_sign is not None else "None"
        angle_str = str(self.latest_angle) if self.latest_angle is not None else "None"
        odom_str = f"{self.latest_odom:.2f}" if self.latest_odom is not None else "None"
        motor_str = str(self.latest_motor) if self.latest_motor is not None else "None"

        log_line = (f"{timestamp}, LAT: {lat_str}, LON: {lon_str}, SIGN: {sign_str}, "
                    f"ANGLE: {angle_str}, ODOM: {odom_str}, MOTOR: {motor_str}\n")

        with open(self.log_file_path, 'a') as f:
            f.write(log_line)

def main(args=None):
    rclpy.init(args=args)
    node = GPSAndSignLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
