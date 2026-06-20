import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class PclPoseToCovariance(Node):
    def __init__(self):
        super().__init__('pcl_pose_to_covariance')

        self.declare_parameter('input_topic', '/pcl_pose')
        self.declare_parameter('output_topic', '/pcl_pose_with_covariance')

        self.declare_parameter('cov_x', 0.05)
        self.declare_parameter('cov_y', 0.05)
        self.declare_parameter('cov_yaw', 0.05)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.cov_x = float(self.get_parameter('cov_x').value)
        self.cov_y = float(self.get_parameter('cov_y').value)
        self.cov_yaw = float(self.get_parameter('cov_yaw').value)

        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.callback,
            20
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_topic,
            20
        )

        self.get_logger().info(
            f"PCL pose converter started: {self.input_topic} -> {self.output_topic}"
        )

    def callback(self, msg: PoseStamped):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose = msg.pose

        cov = [0.0] * 36

        cov[0] = self.cov_x        # x
        cov[7] = self.cov_y        # y

        cov[14] = 999.0            # z
        cov[21] = 999.0            # roll
        cov[28] = 999.0            # pitch

        cov[35] = self.cov_yaw     # yaw

        out.pose.covariance = cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PclPoseToCovariance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
