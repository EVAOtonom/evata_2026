#!/usr/bin/env python3

import copy
import json
import logging
import math
import os
import random
import time

import cv2
import numpy as np
import rclpy
import tf2_ros

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import ComputePathThroughPoses, FollowPath, NavigateToPose
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from ultralytics import YOLO

logging.getLogger('ultralytics').setLevel(logging.ERROR)


# ============================================================
# GENEL AYARLAR
# ============================================================

MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'
ODOM_TOPIC = '/odom'
AMCL_TOPIC = '/amcl_pose'


# ============================================================
# PARK ALANI TETİK POLİGONU
# ============================================================
#
# Kullanıcının verdiği ilk 4 /initialpose noktası.
# Noktalar dikdörtgen çevresinde sırasıyla verilmiştir.
#
PARK_TETIK_BOLGESI = [
    (-26.89486312866211, -10.26382827758789),
    (-29.311664581298828, -15.915367126464844),
    (-21.72671127319336, -20.228418350219727),
    (-19.718902587890625, -14.651239395141602),
]


# ============================================================
# PARK ALANINA GİRİNCE GİDİLECEK LEVHA TARAMA WAYPOINT'İ
# ============================================================
# 5. /initialpose:
# position:
#   x: -20.908710479736328
#   y: -16.73338508605957
# orientation:
#   z: -0.07437800594191862
#   w:  0.9972301199984405
# yaw ~= -8.53097 derece
#
STAGING_X = -20.908710479736328
STAGING_Y = -16.73338508605957
STAGING_YAW_DEG = -8.530969701038241


# ============================================================
# STAGING DURUŞ KONTROLÜ
# ============================================================

STOP_LINEAR_THRESHOLD = 0.05
STOP_ANGULAR_THRESHOLD = 0.05
STOP_HOLD_SECONDS = 1.0

# Normal route cancel edildikten sonra staging goal gönderme gecikmesi.
CANCEL_WAIT_SECONDS = 0.50


# ============================================================
# PARK LEVHASI TARAMA
# ============================================================

# Bu süre SADECE tabela hiç bulunamazsa fallback/random seçim için maksimum bekleme süresidir.
# Geçerli PARK tabelası görülür görülmez bu timeout BEKLENMEDEN park yolu çizilir.
PARK_SIGN_TIMEOUT_SECONDS = 20.0

# İlk geçerli PARK tespitinde park seçilip planner başlatılır.
PARK_REQUIRED_DETECTIONS = 1

PARK_CONFIDENCE_THRESHOLD = 0.60
DETECTION_INTERVAL = 0.15
YOLO_GENERAL_MIN_CONFIDENCE = 0.60
MIN_SIGN_DISTANCE = 0.70
MAX_SIGN_DISTANCE = 25.0

# Kamera GUI yalnızca park levhası taranırken açılır.
SHOW_CAMERA_DURING_PARK_SCAN = True
CAMERA_WINDOW_NAME = 'Park Levha Taramasi'

# PointCloud frame'i map'e bağlı değilse eski çalışan fallback.
LEGACY_SIGN_FRAME = 'camera_link'


# ============================================================
# CÜLLOP PARK GEOMETRİSİ
# ============================================================

# Final park noktasından önce aracın tamamen park yönüne dönmüş
# olarak ilerleyeceği düz giriş mesafesi.
DUZ_GIRIS_MESAFESI = 1.5

# Park 1-6 için ortak viraj hazırlık geometrisi.
VIRAJ_HAZIRLIK_MESAFESI = 5.0
VIRAJ_SAG_OFSET_MESAFESI = 3.0
VIRAJ_HAZIRLIK_YAW_OFFSET_DEG = 50.0

UC_ASAMALI_PARK_YERLERI = {1, 2, 3, 4, 5, 6}
IKI_ASAMALI_PARK_YERLERI = {7, 8}


# ============================================================
# NAV2 PARK PLANNER / CONTROLLER
# ============================================================

PLANNER_ID = 'ParkingGrid'
CONTROLLER_ID = 'ParkFollowPath'
GOAL_CHECKER_ID = 'general_goal_checker'
PROGRESS_CHECKER_ID = ''

NORMAL_XY_GOAL_TOLERANCE = 1.0
NORMAL_YAW_GOAL_TOLERANCE = 0.7

PARK_XY_GOAL_TOLERANCE = 0.20
PARK_YAW_GOAL_TOLERANCE = 0.10


# ============================================================
# 7 PARK NOKTASI - GÜNCEL /initialpose VERİLERİ
# ============================================================

PARK_NOKTALARI = {
    1: {
        'x': -1.4180430173873901,
        'y': -4.5178375244140625,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.2400015610698199,
        'ow': 0.9707725020230278,
    },

    2: {
        'x': -2.8074028491973877,
        'y': -6.667671203613281,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.23862987661932533,
        'ow': 0.9711105920463671,
    },

    3: {
        'x': -3.787271022796631,
        'y': -9.139251708984375,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.22251766012241805,
        'ow': 0.9749286594072635,
    },

    4: {
        'x': -4.957259654998779,
        'y': -11.318336486816406,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.2238163098615337,
        'ow': 0.9746313454019248,
    },

    5: {
        'x': -5.893252372741699,
        'y': -13.81916618347168,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.22836231312558686,
        'ow': 0.9735762188672911,
    },

    6: {
        'x': -7.355737209320068,
        'y': -15.80812931060791,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.23184274254282514,
        'ow': 0.9727532794754389,
    },

    7: {
        'x': -8.67197322845459,
        'y': -18.016464233398438,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.21689222045181097,
        'ow': 0.9761955565907289,
    },
    
    8: {
        'x': -5.4750494956970215,
        'y': -25.73324966430664,
        'z': 0.0,
        'ox': 0.0,
        'oy': 0.0,
        'oz': -0.7389591722127844,
        'ow': 0.6737502072746222,
    },
}


class SignDetectorParkingManager(Node):
    STATE_NORMAL = 'NORMAL'
    STATE_CANCELING = 'CANCELING'
    STATE_STAGING_NAV = 'STAGING_NAV'
    STATE_WAIT_STOP = 'WAIT_STOP'
    STATE_WAIT_PARK_SIGN = 'WAIT_PARK_SIGN'
    STATE_WAIT_PARK_SERVERS = 'WAIT_PARK_SERVERS'
    STATE_PARK_TOLERANCE = 'PARK_TOLERANCE'
    STATE_PARK_PLANNING = 'PARK_PLANNING'
    STATE_PARK_FOLLOWING = 'PARK_FOLLOWING'
    STATE_DONE = 'DONE'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('sign_detector_adaptive_parking_manager')

        # ----------------------------------------------------
        # YOLO / CAMERA
        # ----------------------------------------------------
        dir_path = os.path.dirname(os.path.realpath(__file__))
        workspace_root = dir_path.split('/install')[0]

        model_candidates = [
            os.path.join(dir_path, 'utils', 'bestHavva.pt'),
            os.path.join(
                workspace_root,
                'src',
                'reel_evata',
                'reel_evata',
                'utils',
                'bestHavva.pt',
            ),
        ]

        model_path = next(
            (p for p in model_candidates if os.path.exists(p)),
            model_candidates[0],
        )

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.fx = 277.0
        self.latest_pointcloud = None
        self.annotated_image = None
        self.last_detections = {}
        self.last_detection_time = 0.0
        self.camera_window_opened = False
        self.camera_window_failed = False

        # ----------------------------------------------------
        # TF
        # ----------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----------------------------------------------------
        # SUBSCRIBERS
        # ----------------------------------------------------
        self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.color_image_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.camera_info_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.point_cloud_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            20,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            AMCL_TOPIC,
            self.amcl_pose_callback,
            20,
        )

        # ----------------------------------------------------
        # PUBLISHERS
        # ----------------------------------------------------
        self.sign_publisher = self.create_publisher(
            String,
            '/detected_signs',
            10,
        )

        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.goal_poses_pub = self.create_publisher(
            PoseArray,
            '/parking/goal_poses',
            path_qos,
        )
        self.planner_path_pub = self.create_publisher(
            Path,
            '/parking/planner_path',
            path_qos,
        )
        self.follow_path_pub = self.create_publisher(
            Path,
            '/parking/follow_path',
            path_qos,
        )

        # ----------------------------------------------------
        # NAV2 ACTION CLIENTS
        # ----------------------------------------------------
        # Normal navigasyondan staging noktasına geçiş.
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        # Cüllop park planner/controller.
        self.planner_client = ActionClient(
            self,
            ComputePathThroughPoses,
            'compute_path_through_poses',
        )
        self.controller_client = ActionClient(
            self,
            FollowPath,
            'follow_path',
        )

        # ----------------------------------------------------
        # ACTIVE NAVIGATION CANCEL SERVICES
        # ----------------------------------------------------
        self.cancel_navigate_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal',
        )
        self.cancel_navigate_through_client = self.create_client(
            CancelGoal,
            '/navigate_through_poses/_action/cancel_goal',
        )
        self.cancel_follow_client = self.create_client(
            CancelGoal,
            '/follow_path/_action/cancel_goal',
        )

        # ----------------------------------------------------
        # CONTROLLER PARAMETER CLIENT
        # ----------------------------------------------------
        self.set_params_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
        )

        # ----------------------------------------------------
        # STATE / LOCALIZATION
        # ----------------------------------------------------
        self.state = self.STATE_NORMAL
        self.zone_triggered = False

        self.current_linear_speed = 999.0
        self.current_angular_speed = 999.0
        self.odom_received = False
        self.stopped_since = None

        self.latest_amcl_map_pose = None
        self.last_robot_map_pose = None
        self.first_position_logged = False

        # ----------------------------------------------------
        # STAGING
        # ----------------------------------------------------
        self.cancel_started_at = None
        self.staging_goal_sent = False
        self.staging_goal_handle = None

        # ----------------------------------------------------
        # PARK LEVHASI
        # ----------------------------------------------------
        self.park_detection_count = 0
        self.park_detection_points = []
        self.park_search_started_at = None
        self.selected_park_index = None
        self.selection_reason = None

        self.sign_tf_warning_logged = False
        self.legacy_tf_warning_logged = False

        # ----------------------------------------------------
        # CÜLLOP PARK RUNTIME
        # ----------------------------------------------------
        self.turn_pose = None
        self.align_pose = None
        self.final_pose = None
        self.goal_poses = []
        self.goal_labels = []
        self.planned_path = None

        self.feedback_counter = 0
        self.parking_started = False
        self.parking_finished = False

        self.tolerance_request_pending = False
        self.park_tolerance_active = False
        self.restore_request_pending = False
        self.restore_future = None

        self.control_timer = self.create_timer(0.10, self.control_loop)

        self.get_logger().info(
            '\n========== OTOMATİK PARK + CÜLLOP PARK ==========' '\n'
            f'Tetik poligonu: {PARK_TETIK_BOLGESI}\n'
            f'Staging: X={STAGING_X:.3f}, Y={STAGING_Y:.3f}, '
            f'Yaw={STAGING_YAW_DEG:.2f}°\n'
            f'Levha tarama timeout: {PARK_SIGN_TIMEOUT_SECONDS:.0f} sn (yalnızca tabela yoksa)\n'
            'Levha bulunursa: levhaya en yakın 8 park slotundan biri\n'
            'Levha bulunmazsa: Park 1-8 arasından RANDOM\n'
            f'Park planner/controller: {PLANNER_ID} / {CONTROLLER_ID}'
        )

    # ========================================================
    # GEOMETRİ
    # ========================================================

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        return (
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def create_pose(self, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    @staticmethod
    def point_on_segment(px, py, x1, y1, x2, y2, eps=1e-6):
        cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1)
        if abs(cross) > eps:
            return False
        dot = (px - x1) * (px - x2) + (py - y1) * (py - y2)
        return dot <= eps

    @classmethod
    def point_in_polygon(cls, x, y, polygon):
        for i in range(len(polygon)):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % len(polygon)]
            if cls.point_on_segment(x, y, x1, y1, x2, y2):
                return True

        inside = False
        j = len(polygon) - 1

        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            intersects = (
                (yi > y) != (yj > y)
                and x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            )
            if intersects:
                inside = not inside
            j = i

        return inside

    def get_robot_pose_map_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                rclpy.time.Time(),
            )
            q = transform.transform.rotation
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                self.quaternion_to_yaw(q.x, q.y, q.z, q.w),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    def get_robot_pose_map(self):
        if self.latest_amcl_map_pose is not None:
            return self.latest_amcl_map_pose
        return self.get_robot_pose_map_tf()

    # ========================================================
    # AMCL / ODOM
    # ========================================================

    def amcl_pose_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.latest_amcl_map_pose = (
            float(p.x),
            float(p.y),
            self.quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )

        if not self.first_position_logged:
            self.first_position_logged = True
            x, y, yaw = self.latest_amcl_map_pose
            inside = self.point_in_polygon(x, y, PARK_TETIK_BOLGESI)
            self.get_logger().warn(
                '\n[KONUM KONTROL - 1 KEZ]\n'
                f'/amcl_pose: X={x:.3f}, Y={y:.3f}, '
                f'Yaw={math.degrees(yaw):.1f}°\n'
                f'Park alanında mı: {"EVET" if inside else "HAYIR"}'
            )

    def odom_callback(self, msg):
        self.odom_received = True
        self.current_linear_speed = float(msg.twist.twist.linear.x)
        self.current_angular_speed = float(msg.twist.twist.angular.z)

    # ========================================================
    # ANA STATE MACHINE
    # ========================================================

    def control_loop(self):
        robot_pose = self.get_robot_pose_map()
        if robot_pose is not None:
            self.last_robot_map_pose = robot_pose

        # ----------------------------------------------------
        # 1) NORMAL NAVİGASYON -> PARK ALANINA GİRİŞ
        # ----------------------------------------------------
        if self.state == self.STATE_NORMAL:
            if self.zone_triggered or robot_pose is None:
                return

            x, y, _ = robot_pose
            if self.point_in_polygon(x, y, PARK_TETIK_BOLGESI):
                self.zone_triggered = True
                self.state = self.STATE_CANCELING
                self.cancel_started_at = time.monotonic()

                self.get_logger().warn(
                    '\n[PARK ALANI TETİKLENDİ]\n'
                    f'Araç park alanına girdi: X={x:.3f}, Y={y:.3f}\n'
                    'Mevcut normal Nav2 waypoint/route iptal ediliyor.'
                )
                self.cancel_active_navigation()
            return

        # ----------------------------------------------------
        # 2) CANCEL -> STAGING
        # ----------------------------------------------------
        if self.state == self.STATE_CANCELING:
            if self.cancel_started_at is None:
                self.cancel_started_at = time.monotonic()

            if (
                not self.staging_goal_sent
                and time.monotonic() - self.cancel_started_at >= CANCEL_WAIT_SECONDS
            ):
                self.send_staging_goal()
            return

        # ----------------------------------------------------
        # 3) STAGING BİTTİ -> GERÇEK DURUŞU DOĞRULA
        # ----------------------------------------------------
        if self.state == self.STATE_WAIT_STOP:
            linear_ok = abs(self.current_linear_speed) <= STOP_LINEAR_THRESHOLD
            angular_ok = abs(self.current_angular_speed) <= STOP_ANGULAR_THRESHOLD

            if linear_ok and angular_ok:
                now = time.monotonic()
                if self.stopped_since is None:
                    self.stopped_since = now

                if now - self.stopped_since >= STOP_HOLD_SECONDS:
                    self.start_park_sign_scan()
            else:
                self.stopped_since = None
            return

        # ----------------------------------------------------
        # 4) PARK LEVHASI: GÖRÜLÜR GÖRÜLMEZ PARKI BAŞLAT
        # ----------------------------------------------------
        if self.state == self.STATE_WAIT_PARK_SIGN:
            if self.park_search_started_at is None:
                self.park_search_started_at = time.monotonic()

            elapsed = time.monotonic() - self.park_search_started_at
            if elapsed >= PARK_SIGN_TIMEOUT_SECONDS:
                self.finish_park_sign_scan_and_select()
            return

        # ----------------------------------------------------
        # 5) CÜLLOP PARK SERVERLARINI BEKLE / PARKI BAŞLAT
        # ----------------------------------------------------
        if self.state == self.STATE_WAIT_PARK_SERVERS:
            self.try_start_selected_parking()
            return

    # ========================================================
    # AKTİF NORMAL NAVİGASYONU İPTAL ET
    # ========================================================

    def cancel_active_navigation(self):
        clients = (
            (self.cancel_navigate_client, 'NavigateToPose'),
            (self.cancel_navigate_through_client, 'NavigateThroughPoses'),
            (self.cancel_follow_client, 'FollowPath'),
        )

        for client, name in clients:
            if not client.wait_for_service(timeout_sec=0.05):
                self.get_logger().info(f'[CANCEL] {name} cancel servisi hazır değil; atlandı.')
                continue

            future = client.call_async(CancelGoal.Request())
            future.add_done_callback(
                lambda f, n=name: self.cancel_service_callback(f, n)
            )

    def cancel_service_callback(self, future, action_name):
        try:
            response = future.result()
            self.get_logger().info(
                f'[CANCEL] {action_name}: '
                f'{len(response.goals_canceling)} goal iptal sürecinde.'
            )
        except Exception as error:
            self.get_logger().warn(
                f'[CANCEL] {action_name} cevabı alınamadı: {error}'
            )

    # ========================================================
    # STAGING WAYPOINT
    # ========================================================

    def send_staging_goal(self):
        if self.staging_goal_sent:
            return

        if not self.navigate_client.wait_for_server(timeout_sec=0.05):
            # Server henüz hazır değilse state CANCELING'de kal ve tekrar dene.
            return

        self.staging_goal_sent = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(
            STAGING_X,
            STAGING_Y,
            0.0,
            math.radians(STAGING_YAW_DEG),
        )

        self.state = self.STATE_STAGING_NAV

        self.get_logger().warn(
            '\n[STAGING GOAL]\n'
            'Eski waypoint iptal edildi. Park tarama waypointi gönderiliyor.\n'
            f'X={STAGING_X:.3f}, Y={STAGING_Y:.3f}, '
            f'Yaw={STAGING_YAW_DEG:.2f}°'
        )

        future = self.navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.staging_goal_response_callback)

    def staging_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_with_error(f'[STAGING] Goal gönderilemedi: {error}')
            return

        if not goal_handle.accepted:
            self.finish_with_error('[STAGING] NavigateToPose goal reddedildi.')
            return

        self.staging_goal_handle = goal_handle
        self.get_logger().info('[STAGING] Goal kabul edildi.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.staging_result_callback)

    def staging_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.finish_with_error(f'[STAGING] Sonuç alınamadı: {error}')
            return

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.stopped_since = None
            self.state = self.STATE_WAIT_STOP
            self.get_logger().warn(
                '[STAGING] Waypoint tamamlandı. Araç tam duruşu doğrulanıyor.'
            )
            return

        self.finish_with_error(
            f'[STAGING] Başarısız. action_status={wrapped_result.status}'
        )

    # ========================================================
    # PARK LEVHASI TARAMASI - TESPİTTE ANINDA DEVAM
    # ========================================================

    def start_park_sign_scan(self):
        if self.state == self.STATE_WAIT_PARK_SIGN:
            return

        self.reset_park_confirmation()
        self.park_search_started_at = time.monotonic()
        self.state = self.STATE_WAIT_PARK_SIGN

        self.get_logger().warn(
            '\n[PARK LEVHASI TARAMASI BAŞLADI]\n'
            'Geçerli PARK tabelası görülür görülmez beklemeden park yolu çizilecek.\n'
            f'Tabela hiç görülmezse {PARK_SIGN_TIMEOUT_SECONDS:.0f} sn sonunda '
            '1-8 arasından random park seçilecek.'
        )

    def finish_park_sign_scan_and_select(self):
        if self.state != self.STATE_WAIT_PARK_SIGN:
            return

        elapsed = 0.0
        if self.park_search_started_at is not None:
            elapsed = time.monotonic() - self.park_search_started_at

        self.close_camera_window()

        if self.park_detection_count >= PARK_REQUIRED_DETECTIONS:
            # PARK_REQUIRED_DETECTIONS kadar geçerli tespit toplanmıştır.
            # Mevcut geçerli tabela map noktalarının ortalamasıyla park seçilir.
            avg_x = sum(p[0] for p in self.park_detection_points) / len(
                self.park_detection_points
            )
            avg_y = sum(p[1] for p in self.park_detection_points) / len(
                self.park_detection_points
            )

            selected_index, selected_distance = self.find_nearest_park(
                avg_x,
                avg_y,
            )

            self.selected_park_index = selected_index
            self.selection_reason = 'PARK_SIGN'

            self.get_logger().warn(
                '\n[PARK LEVHASI BULUNDU - ANINDA PARK BAŞLATILIYOR]\n'
                f'Tespit süresi: {elapsed:.1f} sn\n'
                f'Geçerli tespit: {self.park_detection_count}\n'
                f'Tabela ortalama map konumu: X={avg_x:.3f}, Y={avg_y:.3f}\n'
                f'En yakın park: #{selected_index}\n'
                f'Tabela -> park mesafesi: {selected_distance:.2f} m'
            )
        else:
            self.selected_park_index = random.choice(
                sorted(PARK_NOKTALARI.keys())
            )
            self.selection_reason = 'RANDOM_TIMEOUT'

            self.get_logger().warn(
                '\n[PARK LEVHASI YOK - RANDOM PARK]\n'
                f'Tarama süresi: {elapsed:.1f} sn\n'
                f'Geçerli tespit: {self.park_detection_count}\n'
                f'Random seçilen park: #{self.selected_park_index}'
            )

        self.park_search_started_at = None
        self.state = self.STATE_WAIT_PARK_SERVERS

    # ========================================================
    # CAMERA / POINTCLOUD / TF
    # ========================================================

    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]

    def get_point_from_pointcloud(self, center_x, center_y):
        if self.latest_pointcloud is None:
            return None, None, None

        try:
            cloud = self.latest_pointcloud
            center_x = int(min(max(center_x, 0), cloud.width - 1))
            center_y = int(min(max(center_y, 0), cloud.height - 1))

            x_offset = next(f.offset for f in cloud.fields if f.name == 'x')
            y_offset = next(f.offset for f in cloud.fields if f.name == 'y')
            z_offset = next(f.offset for f in cloud.fields if f.name == 'z')

            point_offset = center_y * cloud.row_step + center_x * cloud.point_step
            data = cloud.data

            x = np.frombuffer(
                data,
                dtype=np.float32,
                count=1,
                offset=point_offset + x_offset,
            )[0]
            y = np.frombuffer(
                data,
                dtype=np.float32,
                count=1,
                offset=point_offset + y_offset,
            )[0]
            z = np.frombuffer(
                data,
                dtype=np.float32,
                count=1,
                offset=point_offset + z_offset,
            )[0]

            if any(math.isnan(v) or math.isinf(v) for v in (x, y, z)):
                return None, None, None

            return float(x), float(y), float(z)
        except Exception:
            return None, None, None

    def transform_xyz_with_frame(self, x, y, z, source_frame):
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0

        transform = self.tf_buffer.lookup_transform(
            MAP_FRAME,
            source_frame,
            rclpy.time.Time(),
        )

        transformed = do_transform_pose(pose.pose, transform)
        return (
            float(transformed.position.x),
            float(transformed.position.y),
            float(transformed.position.z),
        )

    def transform_camera_point_to_map(self, x, y, z):
        cloud_frame = None
        if self.latest_pointcloud is not None:
            cloud_frame = self.latest_pointcloud.header.frame_id

        if cloud_frame:
            try:
                return self.transform_xyz_with_frame(x, y, z, cloud_frame)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                pass

        try:
            result = self.transform_xyz_with_frame(
                x,
                y,
                z,
                LEGACY_SIGN_FRAME,
            )

            if not self.legacy_tf_warning_logged:
                self.legacy_tf_warning_logged = True
                self.get_logger().warn(
                    '[LEVHA TF] PointCloud frame map ağacına bağlı değil; '
                    f'fallback source={LEGACY_SIGN_FRAME} kullanılıyor.'
                )
            return result

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            if not self.sign_tf_warning_logged:
                self.sign_tf_warning_logged = True
                self.get_logger().error(
                    '[LEVHA TF - 1 KEZ] map dönüşümü yapılamıyor. '
                    f'cloud_frame={cloud_frame}, fallback={LEGACY_SIGN_FRAME}. '
                    f'Hata: {error}'
                )
            return None, None, None

    # ========================================================
    # YOLO
    # ========================================================

    def run_yolo(self, image):
        results = self.model(image, imgsz=960, verbose=False)
        detections = {}

        for result in results:
            for box in result.boxes:
                confidence = float(box.conf)
                if confidence <= YOLO_GENERAL_MIN_CONFIDENCE:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_name = self.model.names[int(box.cls)]

                old = detections.get(class_name)
                if old is None or confidence > old[4]:
                    detections[class_name] = (
                        x1,
                        y1,
                        x2,
                        y2,
                        confidence,
                    )

        return detections

    def reset_park_confirmation(self):
        self.park_detection_count = 0
        self.park_detection_points = []

    def process_park_detection(self, detections):
        # Park levhası yalnızca staging noktasındaki tarama durumunda seçime etki eder.
        if self.state != self.STATE_WAIT_PARK_SIGN:
            return

        park_detection = None
        for class_name, detection in detections.items():
            if class_name.lower() == 'park':
                park_detection = detection
                break

        if park_detection is None:
            return

        x1, y1, x2, y2, confidence = park_detection
        if confidence <= PARK_CONFIDENCE_THRESHOLD:
            return

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        camera_x, camera_y, camera_z = self.get_point_from_pointcloud(
            center_x,
            center_y,
        )
        if camera_x is None:
            return

        distance = math.sqrt(
            camera_x ** 2 + camera_y ** 2 + camera_z ** 2
        )
        if distance < MIN_SIGN_DISTANCE or distance > MAX_SIGN_DISTANCE:
            return

        map_x, map_y, _ = self.transform_camera_point_to_map(
            camera_x,
            camera_y,
            camera_z,
        )
        if map_x is None:
            return

        self.park_detection_count += 1
        self.park_detection_points.append((map_x, map_y))

        self.get_logger().warn(
            f'[PARK LEVHASI] geçerli={self.park_detection_count} | '
            f'conf={confidence:.2f} | dist={distance:.2f} m | '
            f'map=({map_x:.2f}, {map_y:.2f})'
        )

        # Kritik değişiklik:
        # Gerekli tespit sayısına ulaşınca 20 sn timeout'u BEKLEME.
        # En yakın parkı seç ve doğrudan Cüllop planner/controller aşamasına geç.
        if self.park_detection_count >= PARK_REQUIRED_DETECTIONS:
            self.finish_park_sign_scan_and_select()

    def find_nearest_park(self, sign_x, sign_y):
        best_index = None
        best_distance = float('inf')

        for park_index, park in PARK_NOKTALARI.items():
            distance = math.hypot(
                float(park['x']) - sign_x,
                float(park['y']) - sign_y,
            )
            if distance < best_distance:
                best_distance = distance
                best_index = park_index

        return best_index, best_distance

    def color_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='rgb8',
            )
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            now = time.monotonic()
            if now - self.last_detection_time >= DETECTION_INTERVAL:
                self.last_detection_time = now
                self.last_detections = self.run_yolo(cv_image)

                # 20 sn park taraması aktifse park tespitlerini topla.
                self.process_park_detection(self.last_detections)

                # Eski birleşik levha node davranışı: diğer algıları publish et.
                sign_data = {}
                for class_name, detection in self.last_detections.items():
                    x1, y1, x2, y2, _confidence = detection
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    camera_x, camera_y, camera_z = self.get_point_from_pointcloud(
                        center_x,
                        center_y,
                    )

                    if camera_x is not None:
                        distance = math.sqrt(
                            camera_x ** 2 + camera_y ** 2 + camera_z ** 2
                        )
                    else:
                        distance = self.calculate_distance(x1, y1, x2, y2)

                    if MIN_SIGN_DISTANCE <= distance <= MAX_SIGN_DISTANCE:
                        sign_data[class_name] = round(distance, 2)

                if sign_data:
                    publish_msg = String()
                    publish_msg.data = json.dumps(sign_data)
                    self.sign_publisher.publish(publish_msg)

            # Park taraması sırasında kullanıcıya kamera penceresi göster.
            if (
                SHOW_CAMERA_DURING_PARK_SCAN
                and self.state == self.STATE_WAIT_PARK_SIGN
                and not self.camera_window_failed
            ):
                self.annotated_image = cv_image.copy()

                for class_name, detection in self.last_detections.items():
                    x1, y1, x2, y2, confidence = detection
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    camera_x, camera_y, camera_z = self.get_point_from_pointcloud(
                        center_x,
                        center_y,
                    )
                    if camera_x is not None:
                        distance = math.sqrt(
                            camera_x ** 2 + camera_y ** 2 + camera_z ** 2
                        )
                    else:
                        distance = self.calculate_distance(x1, y1, x2, y2)

                    self._draw_box(
                        x1,
                        y1,
                        x2,
                        y2,
                        class_name,
                        distance,
                        confidence,
                    )

                try:
                    if self.annotated_image.shape[0] > 0:
                        small_image = cv2.resize(
                            self.annotated_image,
                            (
                                max(1, self.annotated_image.shape[1] // 2),
                                max(1, self.annotated_image.shape[0] // 2),
                            ),
                        )
                        cv2.imshow(CAMERA_WINDOW_NAME, small_image)
                        self.camera_window_opened = True
                        cv2.waitKey(1)
                except Exception as gui_error:
                    self.camera_window_failed = True
                    self.get_logger().warn(
                        '[KAMERA GUI] Pencere açılamadı; '
                        '20 sn tarama ve YOLO çalışmaya devam edecek. '
                        f'Hata: {gui_error}'
                    )

        except Exception as error:
            self.get_logger().error(f'[IMAGE] {error}')

    def close_camera_window(self):
        if not self.camera_window_opened:
            return
        try:
            cv2.destroyWindow(CAMERA_WINDOW_NAME)
            cv2.waitKey(1)
        except Exception:
            pass
        self.camera_window_opened = False

    def calculate_distance(self, x1, y1, x2, y2):
        real_width = 0.5
        bbox_width = max(x2 - x1, 1)
        return (real_width * self.fx) / bbox_width * 1.7

    def _draw_box(
        self,
        x1,
        y1,
        x2,
        y2,
        class_name,
        distance,
        confidence,
    ):
        if class_name.lower() == 'park' or 'durak' in class_name.lower():
            color = (0, 0, 255)
        else:
            color = (0, 255, 0)

        cv2.rectangle(
            self.annotated_image,
            (x1, y1),
            (x2, y2),
            color,
            2,
        )

        label = f'{class_name}: {distance:.2f}m ({confidence:.2f})'
        cv2.putText(
            self.annotated_image,
            label,
            (x1, max(15, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )

    # ========================================================
    # CÜLLOP PARK: SERVER HAZIRLIK + TOLERANCE
    # ========================================================

    def try_start_selected_parking(self):
        if self.parking_started or self.parking_finished:
            return

        if self.selected_park_index not in PARK_NOKTALARI:
            self.finish_with_error(
                f'Geçersiz park index: {self.selected_park_index}'
            )
            return

        if not self.odom_received:
            return

        planner_ready = self.planner_client.wait_for_server(timeout_sec=0.05)
        controller_ready = self.controller_client.wait_for_server(timeout_sec=0.05)
        param_ready = self.set_params_client.service_is_ready()

        missing = []
        if not planner_ready:
            missing.append('compute_path_through_poses')
        if not controller_ready:
            missing.append('follow_path')
        if not param_ready:
            missing.append('/controller_server/set_parameters')

        if missing:
            return

        self.parking_started = True
        self.state = self.STATE_PARK_TOLERANCE

        self.get_logger().warn(
            '\n[CÜLLOP PARK BAŞLATILIYOR]\n'
            f'Seçilen park: #{self.selected_park_index}\n'
            f'Seçim nedeni: {self.selection_reason}\n'
            f'Planner: {PLANNER_ID}\n'
            f'Controller: {CONTROLLER_ID}'
        )

        self.request_park_goal_tolerance()

    def make_goal_tolerance_request(self, xy_tolerance, yaw_tolerance):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'general_goal_checker.xy_goal_tolerance',
                Parameter.Type.DOUBLE,
                float(xy_tolerance),
            ).to_parameter_msg(),
            Parameter(
                'general_goal_checker.yaw_goal_tolerance',
                Parameter.Type.DOUBLE,
                float(yaw_tolerance),
            ).to_parameter_msg(),
        ]
        return request

    @staticmethod
    def parameter_response_successful(response):
        if response is None:
            return False, 'SetParameters cevabı boş.'

        if len(response.results) != 2:
            return (
                False,
                f'2 parametre sonucu bekleniyordu, {len(response.results)} geldi.',
            )

        for result in response.results:
            if not result.successful:
                return False, result.reason

        return True, ''

    def request_park_goal_tolerance(self):
        if self.tolerance_request_pending:
            return

        self.tolerance_request_pending = True
        request = self.make_goal_tolerance_request(
            PARK_XY_GOAL_TOLERANCE,
            PARK_YAW_GOAL_TOLERANCE,
        )

        self.get_logger().info(
            'Park goal tolerance ayarlanıyor: '
            f'XY={PARK_XY_GOAL_TOLERANCE:.2f} m, '
            f'Yaw={PARK_YAW_GOAL_TOLERANCE:.3f} rad'
        )

        future = self.set_params_client.call_async(request)
        future.add_done_callback(self.park_goal_tolerance_result_callback)

    def park_goal_tolerance_result_callback(self, future):
        self.tolerance_request_pending = False

        try:
            response = future.result()
        except Exception as error:
            self.finish_with_error(
                f'Park goal tolerance servisi hata verdi: {error}'
            )
            return

        successful, reason = self.parameter_response_successful(response)
        if not successful:
            self.finish_with_error(
                f'Park goal tolerance ayarlanamadı: {reason}'
            )
            return

        self.park_tolerance_active = True
        self.get_logger().info(
            'PARK goal tolerance AKTİF: '
            f'XY={PARK_XY_GOAL_TOLERANCE:.2f} m | '
            f'Yaw={PARK_YAW_GOAL_TOLERANCE:.3f} rad'
        )

        self.create_parking_targets(self.selected_park_index)
        self.state = self.STATE_PARK_PLANNING
        self.request_path_through_poses()

    # ========================================================
    # CÜLLOP PARK HEDEFLERİNİ OLUŞTUR
    # ========================================================

    def create_parking_targets(self, park_index):
        goal = PARK_NOKTALARI[park_index]

        goal_x = float(goal['x'])
        goal_y = float(goal['y'])
        goal_z = float(goal['z'])

        final_yaw = self.quaternion_to_yaw(
            float(goal['ox']),
            float(goal['oy']),
            float(goal['oz']),
            float(goal['ow']),
        )

        heading_x = math.cos(final_yaw)
        heading_y = math.sin(final_yaw)

        # Tüm parklarda düzleşme pozu.
        align_x = goal_x - DUZ_GIRIS_MESAFESI * heading_x
        align_y = goal_y - DUZ_GIRIS_MESAFESI * heading_y

        self.align_pose = self.create_pose(
            align_x,
            align_y,
            goal_z,
            final_yaw,
        )
        self.final_pose = self.create_pose(
            goal_x,
            goal_y,
            goal_z,
            final_yaw,
        )

        self.turn_pose = None
        self.goal_poses = []
        self.goal_labels = []

        # PARK 1-6: viraj hazırlık + düzleşme + final.
        if park_index in UC_ASAMALI_PARK_YERLERI:
            right_x = math.sin(final_yaw)
            right_y = -math.cos(final_yaw)

            turn_x = (
                align_x
                - VIRAJ_HAZIRLIK_MESAFESI * heading_x
                + VIRAJ_SAG_OFSET_MESAFESI * right_x
            )
            turn_y = (
                align_y
                - VIRAJ_HAZIRLIK_MESAFESI * heading_y
                + VIRAJ_SAG_OFSET_MESAFESI * right_y
            )
            turn_yaw = self.normalize_angle(
                final_yaw + math.radians(VIRAJ_HAZIRLIK_YAW_OFFSET_DEG)
            )

            self.turn_pose = self.create_pose(
                turn_x,
                turn_y,
                goal_z,
                turn_yaw,
            )

            self.goal_poses = [
                copy.deepcopy(self.turn_pose),
                copy.deepcopy(self.align_pose),
                copy.deepcopy(self.final_pose),
            ]
            self.goal_labels = [
                'VİRAJ BAŞLANGIÇ POSE',
                'DÜZLEŞME POSE',
                'FINAL PARK POSE',
            ]

        # PARK 7-8: düzleşme + final.
        elif park_index in IKI_ASAMALI_PARK_YERLERI:
            self.goal_poses = [
                copy.deepcopy(self.align_pose),
                copy.deepcopy(self.final_pose),
            ]
            self.goal_labels = [
                'DÜZLEŞME POSE',
                'FINAL PARK POSE',
            ]
        else:
            self.finish_with_error(f'Park modu tanımsız: #{park_index}')
            return

        self.get_logger().info(
            f'\nPark #{park_index} için {len(self.goal_poses)} yönlü pose oluşturuldu:'
        )

        for index, (label, pose) in enumerate(
            zip(self.goal_labels, self.goal_poses),
            start=1,
        ):
            q = pose.pose.orientation
            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.get_logger().info(
                f'{index}. {label} | '
                f'X={pose.pose.position.x:.3f} | '
                f'Y={pose.pose.position.y:.3f} | '
                f'Yaw={math.degrees(yaw):.2f}°'
            )

        self.publish_goal_poses()

    def publish_goal_poses(self):
        msg = PoseArray()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = [copy.deepcopy(pose.pose) for pose in self.goal_poses]
        self.goal_poses_pub.publish(msg)

    # ========================================================
    # COMPUTE PATH THROUGH POSES
    # ========================================================

    def request_path_through_poses(self):
        if len(self.goal_poses) < 2:
            self.finish_with_error('Planner için yeterli goal pose yok.')
            return

        now = self.get_clock().now().to_msg()
        for pose in self.goal_poses:
            pose.header.frame_id = MAP_FRAME
            pose.header.stamp = now

        goal = ComputePathThroughPoses.Goal()
        goal.goals = [copy.deepcopy(pose) for pose in self.goal_poses]
        goal.planner_id = PLANNER_ID

        # Başlangıç pozunu Nav2 TF'den alır.
        goal.use_start = False

        route_text = '\n'.join(
            f'{index} -> {label}'
            for index, label in enumerate(self.goal_labels, start=1)
        )

        self.get_logger().info(
            '\nComputePathThroughPoses gönderiliyor.\n'
            f'Planner: {PLANNER_ID}\n'
            f'Park: #{self.selected_park_index}\n'
            f'Pose sayısı: {len(self.goal_poses)}\n'
            f'{route_text}'
        )

        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self.planner_goal_response_callback)

    def planner_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_with_error(f'Planner goal gönderilemedi: {error}')
            return

        if not goal_handle.accepted:
            self.finish_with_error(
                'ComputePathThroughPoses planner goal reddedildi.'
            )
            return

        self.get_logger().info(
            'ComputePathThroughPoses planner goal kabul edildi.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.planner_result_callback)

    def planner_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.finish_with_error(f'Planner sonucu alınamadı: {error}')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            result = wrapped_result.result
            error_code = getattr(
                result,
                'error_code',
                'Humble sürümünde mevcut değil',
            )
            error_msg = getattr(result, 'error_msg', '')
            self.finish_with_error(
                'ComputePathThroughPoses başarısız. '
                f'Action status={wrapped_result.status}, '
                f'error_code={error_code}, error={error_msg}'
            )
            return

        path = wrapped_result.result.path
        if path is None or len(path.poses) < 2:
            self.finish_with_error('Planner geçerli path oluşturamadı.')
            return

        self.planned_path = path
        self.planner_path_pub.publish(path)

        self.get_logger().info(
            '\nPlanner path oluşturuldu.\n'
            f'Goal pose sayısı: {len(self.goal_poses)}\n'
            f'Path pose sayısı: {len(path.poses)}'
        )

        self.send_path_to_controller(path)

    # ========================================================
    # FOLLOW PATH
    # ========================================================

    def send_path_to_controller(self, path):
        if path is None or len(path.poses) < 2:
            self.finish_with_error('Controller için geçerli path yok.')
            return

        path.header.stamp = self.get_clock().now().to_msg()
        if not path.header.frame_id:
            path.header.frame_id = MAP_FRAME

        for pose in path.poses:
            pose.header.frame_id = path.header.frame_id
            pose.header.stamp = path.header.stamp

        self.follow_path_pub.publish(path)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = CONTROLLER_ID
        goal.goal_checker_id = GOAL_CHECKER_ID

        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = PROGRESS_CHECKER_ID

        self.state = self.STATE_PARK_FOLLOWING

        self.get_logger().info(
            '\nPath controller\'a gönderiliyor.\n'
            f'Controller: {CONTROLLER_ID}\n'
            f'Goal checker: {GOAL_CHECKER_ID}'
        )

        future = self.controller_client.send_goal_async(
            goal,
            feedback_callback=self.controller_feedback_callback,
        )
        future.add_done_callback(self.controller_goal_response_callback)

    def controller_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_with_error(f'Controller goal gönderilemedi: {error}')
            return

        if not goal_handle.accepted:
            self.finish_with_error('FollowPath controller goal reddedildi.')
            return

        self.get_logger().info(
            f'Park #{self.selected_park_index} yolu '
            f'{CONTROLLER_ID} tarafından kabul edildi.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.controller_result_callback)

    def controller_feedback_callback(self, feedback_msg):
        self.feedback_counter += 1
        if self.feedback_counter % 10 != 0:
            return

        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_to_goal', None)
        speed = getattr(feedback, 'speed', None)

        texts = []
        if distance is not None:
            texts.append(f'Kalan mesafe: {distance:.2f} m')
        if speed is not None:
            texts.append(f'Hız: {speed:.2f} m/s')
        if texts:
            self.get_logger().info(' | '.join(texts))

    def controller_result_callback(self, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self.finish_with_error(f'Controller sonucu alınamadı: {error}')
            return

        status = wrapped_result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.parking_finished = True
            self.state = self.STATE_DONE
            self.get_logger().warn(
                '\n===========================================\n'
                f'PARK #{self.selected_park_index} BAŞARIYLA TAMAMLANDI\n'
                f'Seçim nedeni: {self.selection_reason}\n'
                f'Pose sayısı: {len(self.goal_poses)}\n'
                f'Planner/Controller: {PLANNER_ID} / {CONTROLLER_ID}\n'
                '==========================================='
            )
            self.restore_normal_goal_tolerance()
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.parking_finished = True
            self.state = self.STATE_ERROR
            self.get_logger().warn('Park FollowPath iptal edildi.')
            self.restore_normal_goal_tolerance()
            return

        result = wrapped_result.result
        error_code = getattr(result, 'error_code', 'bilinmiyor')
        error_msg = getattr(result, 'error_msg', '')

        self.parking_finished = True
        self.state = self.STATE_ERROR
        self.get_logger().error(
            'Controller path takip başarısız. '
            f'status={status}, error_code={error_code}, error={error_msg}'
        )
        self.restore_normal_goal_tolerance()

    # ========================================================
    # NORMAL GOAL TOLERANCE GERİ YÜKLE
    # ========================================================

    def restore_normal_goal_tolerance(self):
        if not self.park_tolerance_active:
            return self.restore_future

        if self.restore_request_pending:
            return self.restore_future

        if not self.set_params_client.service_is_ready():
            self.get_logger().warn(
                'controller_server set_parameters servisi hazır değil; '
                'normal tolerance geri yüklenemedi.'
            )
            return None

        self.restore_request_pending = True

        request = self.make_goal_tolerance_request(
            NORMAL_XY_GOAL_TOLERANCE,
            NORMAL_YAW_GOAL_TOLERANCE,
        )

        self.restore_future = self.set_params_client.call_async(request)
        self.restore_future.add_done_callback(
            self.restore_goal_tolerance_result_callback
        )
        return self.restore_future

    def restore_goal_tolerance_result_callback(self, future):
        self.restore_request_pending = False

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Normal goal tolerance geri yüklenirken hata: {error}'
            )
            return

        successful, reason = self.parameter_response_successful(response)
        if not successful:
            self.get_logger().error(
                f'Normal goal tolerance geri yüklenemedi: {reason}'
            )
            return

        self.park_tolerance_active = False
        self.get_logger().info(
            'NORMAL goal tolerance geri yüklendi: '
            f'XY={NORMAL_XY_GOAL_TOLERANCE:.2f} m | '
            f'Yaw={NORMAL_YAW_GOAL_TOLERANCE:.3f} rad'
        )

    # ========================================================
    # HATA
    # ========================================================

    def finish_with_error(self, message):
        self.get_logger().error(message)
        self.parking_finished = True
        self.state = self.STATE_ERROR
        self.close_camera_window()
        self.restore_normal_goal_tolerance()


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorParkingManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Otomatik park + cüllop park node kapatılıyor.')
    finally:
        node.close_camera_window()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
