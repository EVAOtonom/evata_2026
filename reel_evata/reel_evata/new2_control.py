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
        self.goals = [       
            {"type": "mini",
                'x': 54.868648529052734,
                'y': 32.496917724609375,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.9527808893412679,
                'ow': 0.30365865195324593
            },
            {"type": "mini",
                'x': 46.75860595703125,
                'y': 39.54705047607422,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.9931568476422821,
                'ow': 0.11678816712854433
            },
            {"type": "mini",
                'x': 40.584407806396484,
                'y': 33.28961944580078,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.6648541537368162,
                'ow': 0.7469731951408309
            },
            {"type": "mini",
                'x': 41.75484085083008,
                'y': 15.032739639282227,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.7664225249720035,
                'ow': 0.6423367599752786
            },
            {"type": "mini",
                'x': 45.03278350830078,
                'y': 6.588950157165527,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.08950049995705449,
                'ow': 0.9959867772754
            },
            {"type": "main",
                'x': 54.20354461669922,
                'y': 4.770875453948975,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.1025946467758954,
                'ow': 0.9947232471662303
            },
            {"type": "mini",
                'x': 64.41436767578125,
                'y': 22.521116256713867,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.6508404689866165,
                'ow': 0.7592145177282117
            },    

        ]
        self.current_index = 0


        self._in_diversion = False
        self.current_pose = None
        self.current_yaw = None
        self.processed_signs = set()

        self._active_goal_handle = None
        self._current_goal_msg = None
        self.motion_enabled = True 
        self.mode = 'normal'
        self.original_goal = None

        self.create_timer(0.5, self.check_goal_distance)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(" Görev başlatılıyor...")
        self.send_next_goal()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = pose.position
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        

    def send_next_goal(self):
        if self.current_index >= len(self.goals):
            self.get_logger().info(" Tüm hedeflere ulaşıldı.")
            return

        goal_dict = self.goals[self.current_index]
        self.get_logger().info(f" Hedef {self.current_index + 1} gönderiliyor...")
        goal_msg = self.create_pose_msg(goal_dict)
        self._current_goal_msg = goal_msg

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("❌ Action server hazır değil.")
            return

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(" Hedef reddedildi.")
            return

        self.get_logger().info("✅ Hedef kabul edildi.")
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info("Hedef tamamlandı.")
        except Exception as e:
            self.get_logger().error(f"❌ Hedef tamamlanırken hata: {e}")


    def check_goal_distance(self):
        if not self.current_pose or self.current_index >= len(self.goals):
            return

        target = self.goals[self.current_index]
        dist = math.hypot(
            target['x'] - self.current_pose.x,
            target['y'] - self.current_pose.y
        )

        if dist <= 1.0:  
            self.get_logger().info(f"📍 Hedefe ulaşıldı ({dist:.2f} m). Sonraki hedefe geçiliyor...")
            if self._active_goal_handle:
                cancel_future = self._active_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda fut: self._handle_goal_cancel(target))
            else:
                self._handle_goal_cancel(target)

    def _handle_goal_cancel(self, target):
        if target["type"] == "main":
            self.get_logger().info("⏳ Ana hedefte 15 saniye bekleniyor...")
            time.sleep(15)
        self._proceed_to_next_goal()

    def _proceed_to_next_goal(self):
        self.current_index += 1
        self.send_next_goal()

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
