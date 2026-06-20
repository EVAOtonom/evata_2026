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


        # Ana hedefler
        self.main_goals = [
 
            {            
                'x': 15.202497482299805,
                'y': 46.36856460571289,
                'z': 0.0,      
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.36788391137131565,
                'ow': 0.9298717265054046
            },

            {            
                'x': 20.6191349029541,
                'y': 58.85987091064453,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.41054064504194604,
                'ow': 0.9118422992867478
            },

            {            
                'x': 32.21401596069336,
                'y': 65.34779357910156,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.08156023752522765,
                'ow': 0.9966684140950933

            },

            {            
                'x': 68.00450134277344,
                'y': 60.75019836425781,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz':-0.0428214645813623,
                'ow': 0.9990827404029694

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

        self.create_timer(0.5, self.check_goal_distance)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(" Görev başlatılıyor...")
        self.send_next_main_goal()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = pose.position
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        

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

