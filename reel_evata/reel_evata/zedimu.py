import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class InitialPoseFromImu(Node):
    def __init__(self):
        super().__init__('initial_pose_from_imu')

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.imu_sub = self.create_subscription(Imu, '/zed/zed_node/imu/data', self.imu_callback, 10)

        self.sent = False  # Sadece bir kere gönderim için

        # Pozisyonu sabit veriyoruz (m)
        self.init_x = 0.0
        self.init_y = 0.0

    def imu_callback(self, msg: Imu):
        if self.sent:
            return  # Sadece ilk IMU mesajında gönder

        # IMU quaternion → yaw
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Yaw → quaternion (sadece z ekseni dönüşü)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        # Initial pose mesajı
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Harita koordinat sistemi

        pose_msg.pose.pose.position.x = self.init_x
        pose_msg.pose.pose.position.y = self.init_y
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        pose_msg.pose.covariance = [0.0] * 36  # Opsiyonel

        # Mesajı yayınla
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"Initial pose published → yaw: {yaw:.3f} rad ({yaw * 180.0 / 3.14159:.2f}°)")

        self.sent = True  # Bir daha göndermesin

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseFromImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

