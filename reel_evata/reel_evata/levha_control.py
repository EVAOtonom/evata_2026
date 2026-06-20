import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException
import math
import time

class RealGoalSender(Node):
    def __init__(self):
        super().__init__('real_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.main_goals = [
            {
                'x': 21.018768310546875,
                'y': -0.4190037250518799,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.5514227524168107,
                'ow': 0.834225957470198
            },
            {
                'x': 19.49641990661621,
                'y': 14.913789749145508,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.9471947967777883,
                'ow': 0.9471947967777883
            },
            {
                'x': 2.0612287521362305,
                'y': 5.090720176696777,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.6545659666592883,
                'ow': 0.7560048910499134
            }
        ]

        self.current_goal_index = 0
        self.goal_handle = None
        self.check_timer = self.create_timer(1.0, self.check_distance_to_goal)  # 1 Hz
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_index >= len(self.main_goals):
            self.get_logger().info("Tüm hedeflere ulaşıldı.")
            return

        goal = self.main_goals[self.current_goal_index]
        self.get_logger().info(f"Ana hedef {self.current_goal_index + 1} gönderiliyor...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.position.z = goal['z']
        goal_msg.pose.pose.orientation.x = goal['ox']
        goal_msg.pose.pose.orientation.y = goal['oy']
        goal_msg.pose.pose.orientation.z = goal['oz']
        goal_msg.pose.pose.orientation.w = goal['ow']

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass  # isteğe bağlı

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Hedef reddedildi!')
            return

        self.get_logger().info('Hedef kabul edildi.')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f"Hedef {self.current_goal_index + 1} tamamlandı. 10 saniye bekleniyor...")
        time.sleep(10)
        self.current_goal_index += 1
        self.send_next_goal()

    def check_distance_to_goal(self):
        if self.current_goal_index >= len(self.main_goals):
            return
        if self.goal_handle is None or not self.goal_handle.accepted:
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            goal = self.main_goals[self.current_goal_index]
            goal_x = goal['x']
            goal_y = goal['y']

            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)

            self.get_logger().info(f"Mesafe hedefe: {distance:.2f} m")

            if distance < 2.0:
                self.get_logger().info("Hedefe 2 metre yaklaşıldı. Hedef tamamlandı varsayılıyor.")
                self.goal_handle.cancel_goal_async()  # iptal ediyoruz
                time.sleep(2)
                self.current_goal_index += 1
                self.send_next_goal()

        except LookupException:
            self.get_logger().warn("TF dönüşümü alınamadı.")

def main(args=None):
    rclpy.init(args=args)
    node = RealGoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
