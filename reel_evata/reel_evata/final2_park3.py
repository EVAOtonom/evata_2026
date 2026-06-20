import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import time
import math
import json
import os


class FullMissionNode(Node):
    def __init__(self):
        super().__init__('full_mission_node')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)

        self.main_goals = [
            {
                'x': 48.84864807128906,
                'y': -11.686671257019043,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.09381233332298237,
                'ow': 0.9955898985608972
            },
            {
                'x': 58.777652740478516,
                'y': -8.517778396606445,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.5898966199066881,
                'ow': 0.8074787785587089
            },
            {
                'x': 64.6831283569336,
                'y': 23.875164031982422,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.609734654322509,
                'ow': 0.7926056089368851
            },
            {
                'x': 66.68405151367188,
                'y': 31.498315811157227,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.6238510018259288,
                'ow': 0.7815432985579142
            }
        ]
        self.current_main_index = 0

        self._in_diversion = False
        self.current_pose = None
        self.current_yaw = None
        self.processed_signs = set()

        self._active_goal_handle = None
        self._current_main_goal_msg = None
        self.motion_enabled = True 
        self.mode = 'normal'
        self.original_goal = None
        self.green_light_detected = False

        self.create_timer(0.5, self.check_goal_distance)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)

        self.get_logger().info(" Görev başlatılıyor...")
        self.send_next_main_goal()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = pose.position
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def _handle_red_light(self):
        if self.mode == 'traffic_light_wait':
            return

        self.motion_enabled = False
        self.green_light_detected = False  # Yeşil ışık bekleme başlangıcı

        motor_msg = Int8()
        motor_msg.data = 0
        self.motor_power_pub.publish(motor_msg)
        self.get_logger().info("Motor gücü kapatıldı (0 gönderildi).")

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

        if self._active_goal_handle:
            self.get_logger().info("Kırmızı ışık: duruluyor, yeşil ışık bekleniyor.")
            self.original_goal = self._current_main_goal_msg
            self._active_goal_handle.cancel_goal_async()

        self.mode = 'traffic_light_wait'

    def _handle_green_light(self):
        self.green_light_detected = True
        if self.mode == 'traffic_light_wait' and self.original_goal:
            self.get_logger().info("Yeşil ışık: kaldığı yerden devam ediliyor.")
            future = self._action_client.send_goal_async(self.original_goal)
            future.add_done_callback(self.main_goal_response_callback)
            self.original_goal = None

        self.motion_enabled = True
        self.mode = 'normal'

    # --- LEVHA CALLBACK ---
    def sign_callback(self, msg):
        try:
            detected = json.loads(msg.data)

            if self.mode == 'traffic_light_wait':
                if "yesil" in detected:
                    self.get_logger().info("Yeşil ışık algılandı (kırmızı moddayken).")
                    self._handle_green_light()
                else:
                    return
            else:
                if "kirmizi" in detected:
                    self.get_logger().info("Kırmızı ışık algılandı.")
                    self._handle_red_light()

                elif "yesil" in detected:
                    self.get_logger().info("Yeşil ışık algılandı.")
                    self._handle_green_light()

        except Exception as e:
            self.get_logger().error(f"Levha verisi işlenemedi: {e}")

    def send_next_main_goal(self):
        if self.current_main_index >= len(self.main_goals):
            self.get_logger().info(" Tüm ana hedeflere ulaşıldı.")
            return

        self.get_logger().info(f" Ana hedef {self.current_main_index + 1} gönderiliyor...")
        goal_dict = self.main_goals[self.current_main_index]
        goal_msg = self.create_pose_msg(goal_dict)
        self._current_main_goal_msg = goal_msg

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("❌ Action server hazır değil.")
            return

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.main_goal_response_callback)

    def main_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(" Ana hedef reddedildi.")
            return

        self.get_logger().info("✅ Ana hedef kabul edildi.")
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.main_goal_result_callback)
    
    def main_goal_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info("✅ Ana hedef tamamlandı.")
        except Exception as e:
            self.get_logger().error(f"❌ Ana hedef tamamlanırken hata: {e}")

    def check_goal_distance(self):
        if not self.current_pose or self.current_main_index >= len(self.main_goals):
            return

        target = self.main_goals[self.current_main_index]
        dist = math.hypot(
            target['x'] - self.current_pose.x,
            target['y'] - self.current_pose.y
        )

        if dist <= 1.0:
            self.get_logger().info(f"📍 Hedefe ulaşıldı ({dist:.2f} m). Sonraki hedefe geçiliyor...")
            if self._active_goal_handle:
                cancel_future = self._active_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._handle_goal_cancel)
            else:
                self._proceed_to_next_goal()

    def _handle_goal_cancel(self, future):
        self._proceed_to_next_goal()

    def _proceed_to_next_goal(self):
        self.current_main_index += 1
        self.send_next_main_goal()

    def create_pose_msg(self, pose_dict):
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose_dict['x']
        msg.pose.pose.position.y = pose_dict['y']
        msg.pose.pose.position.z = pose_dict['z']
        msg.pose.pose.orientation.x = pose_dict['ox']
        msg.pose.pose.orientation.y = pose_dict['oy']
        msg.pose.pose.orientation.z = pose_dict['oz']
        msg.pose.pose.orientation.w = pose_dict['ow']
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FullMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
