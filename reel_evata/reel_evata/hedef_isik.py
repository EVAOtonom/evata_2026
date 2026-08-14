#!/usr/bin/env python3

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SequentialNav2(Node):

    def __init__(self):
        super().__init__('sequential_nav2_goals')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # =========================================================
        # ROTA DOSYASI
        # =========================================================
        self.route_file = os.path.expanduser(
            "~/real_ws/src/reel_evata/reel_evata/rota.json"
        )

        self.goals = self._load_route()

        # ── Kırmızı ışık sistemi ─────────────────────────
        self.motion_enabled = True
        self.current_pose = None
        self.current_goal_handle = None
        self._red_light_timeout_timer = None

        self.traffic_light_zones = [
            {'x_min': -0.4, 'x_max': 10.8, 'y_min': 0.42, 'y_max': 48.3},
        ]

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_power_pub = self.create_publisher(Int8, '/stm/motor_power', 10)

        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(String, '/detected_signs', self._sign_callback, 10)

    def _load_route(self):
        """rota.json -> navigasyon.rota'dan (x, y, oz, ow, wait) listesi yükle."""

        try:
            with open(self.route_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(
                f'Rota dosyası okunamadı ({self.route_file}): {e}'
            )
            return []

        try:
            route_list = data['navigasyon']['rota']
        except (KeyError, TypeError) as e:
            self.get_logger().error(
                f'rota.json içinde navigasyon.rota bulunamadı: {e}'
            )
            return []

        goals = []

        for item in route_list:
            try:
                x = float(item['x'])
                y = float(item['y'])
                qz = float(item['oz'])
                qw = float(item['ow'])
                wait = float(item.get('wait', 0.0))
            except (KeyError, TypeError, ValueError) as e:
                point_id = item.get('id', '?') if isinstance(item, dict) else '?'
                self.get_logger().error(
                    f'rota.json içindeki {point_id} nolu nokta geçersiz: {e}'
                )
                continue

            goals.append((x, y, qz, qw, wait))

        self.get_logger().info(
            f'{len(goals)} Hedef yüklendi.'
        )

        return goals

    def _odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def _sign_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if 'kirmizi' in data:
                self._handle_red_light(data['kirmizi'])
            else:
                self._handle_green_light()
        except Exception as e:
            self.get_logger().error(f'Sign JSON parse hatası: {e}')

    def _in_traffic_light_zone(self):
        if self.current_pose is None:
            return False
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        for zone in self.traffic_light_zones:
            if zone['x_min'] <= x <= zone['x_max'] and \
               zone['y_min'] <= y <= zone['y_max']:
                return True
        return False

    def _stop_motors(self):
        motor_msg = Int8()
        motor_msg.data = 0
        self.motor_power_pub.publish(motor_msg)

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

        self.get_logger().info('Motor gücü kapatıldı (motor_power=0, cmd_vel=0).')

    def _red_light_timeout(self):
        if self._red_light_timeout_timer is not None:
            self._red_light_timeout_timer.cancel()
            self._red_light_timeout_timer = None

        self.get_logger().warn(
            '20 saniye geçti, ışık görülemedi - OTOMATIK HAREKETE GEÇİLİYOR'
        )
        self._handle_green_light()

    def _handle_red_light(self, distance):
        if not self._in_traffic_light_zone():
            self.get_logger().info(
                'Kırmızı ışık ama trigger zone dışındayız, görmezden geliniyor.'
            )
            return

        if self.motion_enabled:
            self.get_logger().info('KIRMIZI IŞIK (zone içi) - DURULUYOR')
            self.motion_enabled = False
            self._stop_motors()

            if self.current_goal_handle is not None:
                # future referansını sakla, yoksa GC toplayabilir
                self._cancel_future = self.current_goal_handle.cancel_goal_async()

        if self._red_light_timeout_timer is not None:
            self._red_light_timeout_timer.cancel()

        self._red_light_timeout_timer = self.create_timer(
            20.0, self._red_light_timeout
        )
        self.get_logger().info(
            'Kırmızı ışık görüldü - 20 saniye timeout başlatıldı (timer reset)'
        )

    def _handle_green_light(self):
        """Yeşil ışık kontrolü"""
        if self._red_light_timeout_timer is not None:
            self._red_light_timeout_timer.cancel()
            self._red_light_timeout_timer = None

        if not self.motion_enabled:
            self.get_logger().info('Kırmızı ışık yok - DEVAM EDİLİYOR')
            self.motion_enabled = True

    def _wait_until_motion_enabled(self):
        """motion_enabled tekrar True olana kadar (yeşil ışık ya da 20sn
        timeout) node'u canlı tut — _wait_seconds ile aynı spin_once paterni."""
        while not self.motion_enabled:
            rclpy.spin_once(self, timeout_sec=0.1)

    def create_goal(self, x, y, qz, qw):
        """NavigateToPose goal mesajı oluştur."""

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Orientation quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        return goal_msg

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        try:
            distance = feedback.distance_remaining

            self.get_logger().info(
                f'Kalan mesafe: {distance:.2f} m',
                throttle_duration_sec=5.0
            )
        except Exception:
            pass

    def _wait_seconds(self, wait_sec):
        """Durakta bekleme (rota.json'dan gelen süre kadar, senkron)."""

        if wait_sec is None or wait_sec <= 0.0:
            return

        self.get_logger().info(
            f'⏳ Bu durakta {wait_sec:.1f} sn bekleniyor...'
        )

        end_time = time.monotonic() + wait_sec

        while time.monotonic() < end_time:
            # Node'u canlı tut, callback'leri işle (0.1 sn adımlarla)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            'Bekleme tamamlandı, sıradaki durağa geçiliyor.'
        )

    def navigate_all(self):


        if not self.goals:
            self.get_logger().error(
                'rota.json boş veya yüklenemedi, hedef yok.'
            )
            return False


        self.nav_client.wait_for_server()


        # =========================================================
        # HEDEFLERİ SIRAYLA GÖNDER
        # =========================================================

        index = 0
        while index < len(self.goals):

            x, y, qz, qw, wait = self.goals[index]

            self.get_logger().info('==========================================')
            self.get_logger().info(f'HEDEF {index + 1}/{len(self.goals)}')
            self.get_logger().info(f'X={x:.3f}  Y={y:.3f}')

            # Kırmızı ışıktaysak yeni goal göndermeden önce yeşili bekle
            self._wait_until_motion_enabled()

            goal_msg = self.create_goal(x, y, qz, qw)

            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )

            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()

            if goal_handle is None:
                self.get_logger().error(f'Hedef {index + 1}: goal handle alınamadı.')
                return False

            if not goal_handle.accepted:
                self.get_logger().error(f'Hedef {index + 1} Nav2 tarafından REDDEDİLDİ.')
                return False

            self.current_goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result_response = result_future.result()
            self.current_goal_handle = None

            if result_response is None:
                self.get_logger().error(f'Hedef {index + 1}: sonuç alınamadı.')
                return False

            status = result_response.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'✅ HEDEF {index + 1} BAŞARILI')
                self._wait_seconds(wait)
                index += 1
                continue

            elif status == GoalStatus.STATUS_CANCELED:
                if not self.motion_enabled:
                    self.get_logger().warn(
                        f'⛔ HEDEF {index + 1} kırmızı ışık nedeniyle iptal edildi, '
                        f'yeşil ışık bekleniyor...'
                    )
                    self._wait_until_motion_enabled()
                    self.get_logger().info(
                        'Yeşil ışık/timeout — aynı hedef tekrar gönderiliyor.'
                    )
                    continue  # index İLERLEMİYOR, aynı hedef tekrar denenecek

                self.get_logger().error(f'❌ HEDEF {index + 1} İPTAL EDİLDİ.')
                return False

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error(f'❌ HEDEF {index + 1} ABORTED.')
                return False

            else:
                self.get_logger().error(
                    f'❌ HEDEF {index + 1} başarısız. Status={status}'
                )
                return False

        # =========================================================
        # TÜM HEDEFLER TAMAMLANDI
        # =========================================================

        self.get_logger().info('')
        self.get_logger().info('==========================================')
        self.get_logger().info('🏁 TÜM NAV2 HEDEFLERİ BAŞARIYLA TAMAMLANDI')
        return True


def main(args=None):

    rclpy.init(args=args)

    node = SequentialNav2()

    try:
        node.navigate_all()

    except KeyboardInterrupt:
        node.get_logger().warn(
            'Program kullanıcı tarafından durduruldu.'
        )

    except Exception as e:
        node.get_logger().error(
            f'Beklenmeyen hata: {e}'
        )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()