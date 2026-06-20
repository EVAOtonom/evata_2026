import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json

class NavGoalOverrideActionClient(Node):
    def __init__(self):
        super().__init__('nav_goal_override_action_client')

        self.override_sent = False

        # Action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Levha dinleyicisi
        self.sign_sub = self.create_subscription(
            String,
            '/detected_signs',
            self.sign_callback,
            10
        )

        self.get_logger().info('NavGoalOverrideActionClient çalışıyor. "sol" tabelası bekleniyor...')

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()

        # Hedef pozisyon
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # düz ileri yön

        self._action_client.wait_for_server()
        self.get_logger().info('NavigateToPose action sunucusuna bağlanıldı.')

        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f'Yeni hedef gönderildi: x = {x}, y = {y}')

    def sign_callback(self, msg):
        if self.override_sent:
            return

        try:
            data = json.loads(msg.data)
            if 'sol' in data:
                self.get_logger().info('"Sol" tabelası tespit edildi! Hedef gönderiliyor.')
                self.send_goal(-14.85, -6.80)
                self.override_sent = True

        except json.JSONDecodeError:
            self.get_logger().warn(f"Geçersiz JSON alındı: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalOverrideActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
