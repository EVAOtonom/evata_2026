import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu  # IMU mesajı için

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.current_angle_deg = 0.0
        self.angular_z = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0


        # Parametreler
        self.declare_parameter('wheel_base_cm', 155.0)
        self.declare_parameter('steering_scale_factor', 0.53)
        self.wheel_base = self.get_parameter('wheel_base_cm').value
        self.steering_scale = self.get_parameter('steering_scale_factor').value

        # Yayıncılar
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Abonelikler
        self.create_subscription(Float32, '/stm/read_odometer', self.odom_callback, 10)
        self.create_subscription(Int32, '/stm/read_wheel_angle', self.angle_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)

        # Değişkenler
        self.last_odom = None
        self.last_time = self.get_clock().now()
        self.current_angle_deg = 0.0
        self.imu_yaw = None
        self.imu_yaw_offset = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0

        self.timer = self.create_timer(0.05, self.publish_odometry)  # 20 Hz yayın

    def angle_callback(self, msg: Int32):
        raw_value = msg.data
        self.current_angle_deg = raw_value * self.steering_scale
        self.get_logger().debug(f"Raw steering: {raw_value} -> Angle(deg): {self.current_angle_deg}")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        _, _, raw_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = raw_yaw + self.imu_yaw_offset  # çalışmazsa bu hesaba bir de initialpose_callback'e bak

    def odom_callback(self, msg: Float32):
        current_odom = msg.data
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if self.last_odom is None or dt <= 0.0:
            self.last_odom = current_odom
            self.last_time = now
            return

        # Odometre farkı (cm → m)
        delta_s_m = (current_odom - self.last_odom) / 100.0
        v = delta_s_m / dt  # m/s

        # Direksiyon açısına göre açısal hız hesabı
        wheelbase_m = self.wheel_base / 100.0
        steering_rad = math.radians(-self.current_angle_deg)
        self.angular_z = v / wheelbase_m * math.tan(steering_rad)

        # Yaw güncelle
        self.yaw += self.angular_z * dt

        # Konum güncelle
        dx = delta_s_m * math.cos(self.yaw)
        dy = delta_s_m * math.sin(self.yaw)
        self.x += dx * 100.0  # tekrar cm
        self.y += dy * 100.0

        self.last_odom = current_odom
        self.last_time = now


    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.x = pose.position.x * 100.0  # metre -> cm
        self.y = pose.position.y * 100.0

        _, _, initial_yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        self.yaw = initial_yaw
        if self.imu_yaw is not None:
            self.imu_yaw_offset = initial_yaw - (self.imu_yaw - self.imu_yaw_offset)  # çalışmazsa burayı değiştir
        else:
            self.imu_yaw_offset = 0.0

        self.imu_yaw = self.yaw  # Doğrulama için güncelle
        self.last_odom = None
        self.last_time = self.get_clock().now()

        self.get_logger().info(f"Initial pose set -> x: {self.x:.2f} cm, y: {self.y:.2f} cm, yaw: {math.degrees(self.yaw):.2f}°")

    def publish_odometry(self):
        now = self.get_clock().now()
        q = quaternion_from_euler(0.0, 0.0, self.yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x / 100.0
        odom_msg.pose.pose.position.y = self.y / 100.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = self.angular_z
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x / 100.0
        t.transform.translation.y = self.y / 100.0
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(t)
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
