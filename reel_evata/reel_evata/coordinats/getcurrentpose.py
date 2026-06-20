import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import Buffer, TransformListener

class PoseGetter(Node):
    def __init__(self):
        super().__init__('pose_getter')

        # TF2 buffer ve listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher (tek topic: /robot_xy)
        self.pose_pub = self.create_publisher(Point, '/robot_xy', 10)

        # Timer ile sürekli pozisyon al
        self.timer = self.create_timer(0.5, self.get_pose)

    def get_pose(self):
        try:
            # base_link → map dönüşümünü al
            trans = self.tf_buffer.lookup_transform(
                'map',        # hedef frame
                'base_link',  # kaynak frame
                rclpy.time.Time()
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Log
            self.get_logger().info(f"📍 Map frame pozisyonu: X={x:.3f}, Y={y:.3f}")

            # Point mesajı oluştur ve yayınla
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = 0.0  # Z ekseni kullanılmıyor

            self.pose_pub.publish(point_msg)

        except Exception as e:
            self.get_logger().warn(f"TF dönüşümü alınamadı: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseGetter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
