import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf_transformations import (
    quaternion_from_euler,
    euler_from_quaternion,
    quaternion_multiply,
    quaternion_inverse,
)


class InitialPoseMapToOdom(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')

        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 30.0)

        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.map_to_odom = TransformStamped()
        self.map_to_odom.header.frame_id = self.map_frame
        self.map_to_odom.child_frame_id = self.odom_frame
        self.map_to_odom.transform.translation.x = 0.0
        self.map_to_odom.transform.translation.y = 0.0
        self.map_to_odom.transform.translation.z = 0.0
        self.map_to_odom.transform.rotation.x = 0.0
        self.map_to_odom.transform.rotation.y = 0.0
        self.map_to_odom.transform.rotation.z = 0.0
        self.map_to_odom.transform.rotation.w = 1.0

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initial_pose_callback,
            10
        )

        self.create_timer(
            1.0 / self.publish_rate,
            self.publish_map_to_odom
        )

        self.get_logger().info(
            f"InitialPose map->odom setter started:\n"
            f"  initialpose_topic: {self.initialpose_topic}\n"
            f"  map_frame        : {self.map_frame}\n"
            f"  odom_frame       : {self.odom_frame}\n"
            f"  base_frame       : {self.base_frame}\n"
            f"  publish_rate     : {self.publish_rate}"
        )

    def yaw_from_quat(self, q):
        _, _, yaw = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w
        ])
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"Cannot get {self.odom_frame} -> {self.base_frame} transform: {e}"
            )
            return

        desired_x = msg.pose.pose.position.x
        desired_y = msg.pose.pose.position.y
        desired_yaw = self.yaw_from_quat(msg.pose.pose.orientation)

        current_x = odom_to_base.transform.translation.x
        current_y = odom_to_base.transform.translation.y
        current_yaw = self.yaw_from_quat(odom_to_base.transform.rotation)

        map_to_odom_yaw = self.normalize_angle(desired_yaw - current_yaw)

        cos_yaw = math.cos(map_to_odom_yaw)
        sin_yaw = math.sin(map_to_odom_yaw)

        rotated_current_x = cos_yaw * current_x - sin_yaw * current_y
        rotated_current_y = sin_yaw * current_x + cos_yaw * current_y

        map_to_odom_x = desired_x - rotated_current_x
        map_to_odom_y = desired_y - rotated_current_y

        q = quaternion_from_euler(0.0, 0.0, map_to_odom_yaw)

        self.map_to_odom.header.frame_id = self.map_frame
        self.map_to_odom.child_frame_id = self.odom_frame

        self.map_to_odom.transform.translation.x = map_to_odom_x
        self.map_to_odom.transform.translation.y = map_to_odom_y
        self.map_to_odom.transform.translation.z = 0.0

        self.map_to_odom.transform.rotation.x = q[0]
        self.map_to_odom.transform.rotation.y = q[1]
        self.map_to_odom.transform.rotation.z = q[2]
        self.map_to_odom.transform.rotation.w = q[3]

        self.get_logger().info(
            f"Initial pose applied by changing map->odom:\n"
            f"  desired map pose:\n"
            f"    x   : {desired_x:.3f}\n"
            f"    y   : {desired_y:.3f}\n"
            f"    yaw : {math.degrees(desired_yaw):.2f} deg\n"
            f"  current odom pose:\n"
            f"    x   : {current_x:.3f}\n"
            f"    y   : {current_y:.3f}\n"
            f"    yaw : {math.degrees(current_yaw):.2f} deg\n"
            f"  new map->odom:\n"
            f"    x   : {map_to_odom_x:.3f}\n"
            f"    y   : {map_to_odom_y:.3f}\n"
            f"    yaw : {math.degrees(map_to_odom_yaw):.2f} deg"
        )

        self.publish_map_to_odom()

    def publish_map_to_odom(self):
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseMapToOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
