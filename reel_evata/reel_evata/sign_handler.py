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
from ament_index_python.packages import get_package_share_directory
import os
import time
import math
import json


class FullMissionNode(Node):
    def __init__(self):
        super().__init__('full_mission_node')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)


        # Ana hedefler
        self.main_goals = [
            {
                'x': 43.416873931884766,
      		'y': 50.23493957519531,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': 0.9980352324729775,
                'ow': 0.06265520523156783
            },
            {
                'x': 59.38241958618164,
                'y': 59.32367706298828,
                'z': 0.0,
                'ox': 0.0,
                'oy': 0.0,
                'oz': -0.12218488732067498,
                'ow': 0.9925073568041871
            },
        ]
        self.current_main_index = 0

        # Sağ ve sol sapma waypoint'leri
        package_share = get_package_share_directory('reel_evata')
        self.diversion_waypoint_file = os.path.join(package_share, 'diversion_waypoints.txt')
        
        self._in_diversion = False
        self.current_pose = None
        self.current_yaw = None
        self.processed_signs = set()

        self._active_goal_handle = None
        self._current_main_goal_msg = None
        self.motion_enabled = True 
        self.mode = 'normal'
        self.original_goal = None



        self.create_subscription(String, '/detected_signs', self.sign_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(" Görev başlatılıyor...")
        self.send_next_main_goal()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = pose.position
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        
    def load_diversion_waypoints_from_txt(self, filepath):
        waypoints = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) != 2:
                        continue
                    x, y = map(float, parts)
                    waypoints.append({
                        'x': x,
                        'y': y,
                        'z': 0.0,
                        'ox': 0.0,
                        'oy': 0.0,
                        'oz': 0.0,
                        'ow': 1.0  # yönsüz (yaw = 0)
                    })
            self.get_logger().info(f"{len(waypoints)} diversion waypoint yüklendi.")
        except Exception as e:
            self.get_logger().error(f"Waypoint dosyası okunamadı: {e}")
            return waypoints


    def send_nearest_right_waypoint(self):
        self.send_nearest_side_waypoint(target="right")

    def send_nearest_left_waypoint(self):
        self.send_nearest_side_waypoint(target="left")
   
    def decide_no_entry_diversion(self):
        self.send_nearest_side_waypoint("noentry")



    def send_forward_waypoint(self, distance=10.0):
        if not self.current_pose or self.current_yaw is None:
            self.get_logger().warn("Pozisyon veya yön bilgisi eksik.")
            return

        x = self.current_pose.x
        y = self.current_pose.y
        yaw = self.current_yaw

        forward_x = x + distance * math.cos(yaw)
        forward_y = y + distance * math.sin(yaw)

        self.get_logger().info(f"➡️ {distance} metre ileri sapma hedefi oluşturuluyor: ({forward_x:.2f}, {forward_y:.2f})")

        temp_goal = {
            'x': forward_x,
            'y': forward_y,
            'z': 0.0,
            'ox': 0.0,
            'oy': 0.0,
            'oz': math.sin(yaw / 2.0),
            'ow': math.cos(yaw / 2.0)
        }

        self.divert_to(temp_goal)


    def _handle_red_light(self):
        if self.mode == 'traffic_light_wait':
            return

        self.motion_enabled = False
        self.green_light_detected = False  # Yeşil ışık bekleme başlangıcı

        # Motor gücünü kapat
        motor_msg = Int8()
        motor_msg.data = 0
        self.motor_power_pub.publish(motor_msg)
        self.get_logger().info("Motor gücü kapatıldı (0 gönderildi).")

        # Robotu durdur
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


    def stop_sign(self):
        print("DURDUMMMMM")
        # Motor gücünü kapat
        motor_msg = Int8()
        motor_msg.data = 0
        self.motor_power_pub.publish(motor_msg)
        self.get_logger().info("Motor gücü kapatıldı (0 gönderildi).")

        # Robotu durdur
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

        time.sleep(5)

    def sign_callback(self, msg):
        try:
            detected = msg.data

            # Kırmızı ışıkta sadece yeşil ışık algılamasına izin ver
            if self.mode == 'traffic_light_wait':
                if "yesil" in detected:
                    self.get_logger().info("Yeşil ışık algılandı (kırmızı moddayken).")
                    self._handle_green_light()
                else:
                    # Kırmızı moddayken diğer levhaları görmezden gel
                    return
            else:
                # Normal modda tüm levhalar işlenir
                if "sag" in detected and "sag" not in self.processed_signs:
                    self.get_logger().info(" Sağ levhası algılandı.")
                    self.processed_signs.add("sag")
                    self.send_nearest_right_waypoint()

                elif "sol" in detected and "sol" not in self.processed_signs:
                    self.get_logger().info(" Sol levhası algılandı.")
                    self.processed_signs.add("sol")
                    self.send_nearest_left_waypoint()

                elif ("ileriden_saga" in detected and "ileriden_saga" not in self.processed_signs):
                    self.get_logger().info(" İleriden sağa levhası algılandı.")
                    self.processed_signs.add("ileriden_saga")
                    self.send_nearest_right_waypoint()

                elif ("ileriden_sola" in detected and "ileriden_sola" not in self.processed_signs):
                    self.get_logger().info(" İleriden sola levhası algılandı.")
                    self.processed_signs.add("ileriden_sola")
                    self.send_nearest_left_waypoint()

                elif "sagadonulmez" in detected and "sagadonulmez" not in self.processed_signs:
                    self.get_logger().info("Sağa dönülmez levhası algılandı.")
                    self.processed_signs.add("sagadonulmez")
                    self.send_forward_waypoint()

                elif "soladonulmez" in detected and "soladonulmez" not in self.processed_signs:
                    self.get_logger().info("Sola dönülmez levhası algılandı.")
                    self.processed_signs.add("soladonulmez")
                    self.send_forward_waypoint()

                elif "girisyok" in detected and "girisyok" not in self.processed_signs:
                    self.get_logger().info(" Girilmez levhası algılandı.")
                    self.processed_signs.add("girisyok")
                    self.decide_no_entry_diversion()

                elif "kirmizi" in detected:
                    self.get_logger().info("Kırmızı ışık algılandı.")
                    self._handle_red_light()

                elif "yesil" in detected:
                    self.get_logger().info("Yeşil ışık algılandı.")
                    self._handle_green_light()

                elif "dur" in detected:
                    self.get_logger().info("Dur algılandı.")
                    self.stop_sign()

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
        if self._in_diversion:
            self.get_logger().warn(" Sapma sırasında sonuç geldi, dikkate alınmıyor.")
            return
        self.get_logger().info(f" Ana hedef {self.current_main_index + 1} tamamlandı.")
        self.current_main_index += 1
        self.send_next_main_goal()

    def divert_to(self, waypoint_dict):
        if self._active_goal_handle:
            self.get_logger().info(" Aktif ana hedef iptal ediliyor...")
            self._active_goal_handle.cancel_goal_async()

        self._in_diversion = True
        waypoint_msg = self.create_pose_msg(waypoint_dict)
        self.get_logger().info("↪️ Sapma hedefi gönderiliyor...")
        future = self._action_client.send_goal_async(waypoint_msg)
        future.add_done_callback(self.diversion_goal_callback)

    def diversion_goal_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Sapma hedefi reddedildi.")
            self._in_diversion = False
            return

        self.get_logger().info("✅ Sapma hedefi kabul edildi.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_to_main_goal)

    def return_to_main_goal(self, future):
        self.get_logger().info("🔁 Sapma tamamlandı. Ana hedefe dönülüyor...")
        self._in_diversion = False
        if self._current_main_goal_msg:
            future = self._action_client.send_goal_async(self._current_main_goal_msg)
            future.add_done_callback(self.main_goal_response_callback)

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
