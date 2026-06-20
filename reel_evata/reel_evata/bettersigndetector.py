#!/usr/bin/env python3.10

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import logging
from sensor_msgs_py import point_cloud2
from ament_index_python.packages import get_package_share_directory
import math
import json
import time
import tf2_ros

logging.getLogger('ultralytics').setLevel(logging.ERROR)

class SignDetectorWithNavigation(Node):
    def __init__(self):
        super().__init__('sign_detector_navigation_node')

        dir_path = os.path.dirname(os.path.realpath(__file__))
        src_dir = dir_path.split('/install')[0]  # install kısmını çıkar
        model_path = os.path.join(src_dir, 'src', 'reel_evata', 'reel_evata', 'utils', 'sol300best.pt')

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.fx = 277.0
        self.tracked_signs = {}
        self.latest_pointcloud = None
        self.last_detection_time = time.time()
        self.detection_interval = 0.15
        
        # Park levhası tespit sayacı ve kontrol değişkenleri
        self.consecutive_parking_detections = 0
        self.required_consecutive_detections = 10
        self.navigation_sent = False
        self.last_parking_coordinates = None
        
        # Map and navigation related
        self.parking_locations = {}  # Store detected parking sign locations
        self.map_data = None
        self.map_origin = None
        self.map_resolution = 0.05
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(Image, "/zed2i/zed_node/rgb/image_rect_color", self.color_image_callback, 10)
        self.create_subscription(CameraInfo, "/zed2i/zed_node/rgb/camera_info", self.camera_info_callback, 10)
        self.create_subscription(PointCloud2, "/zed2i/zed_node/point_cloud/cloud_registered", self.point_cloud_callback, 10)

        
        # Publishers
        self.sign_publisher = self.create_publisher(String, "/detected_signs", 10)
        
        # Navigation action client
        self.navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Navigation related variables
        self.parking_navigation_queue = []  # Queue of parking locations to visit
        self.current_parking_index = 0
        self.auto_navigate = False  # Flag to enable automatic navigation to all parking spots
        
        self.get_logger().info("Sign Detector with Navigation initialized")

    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg
    
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]

    def get_parking_sign_position_from_pointcloud(self, center_x, center_y):
        """Get parking sign position from pointcloud relative to vehicle"""
        if self.latest_pointcloud is None:
            return None, None, None
        
        try:
            width = self.latest_pointcloud.width
            height = self.latest_pointcloud.height
            center_x = min(center_x, width - 1)
            center_y = min(center_y, height - 1)
            index = center_y * width + center_x

            gen = point_cloud2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=False)
            for i, pt in enumerate(gen):
                if i == index:
                    x, y, z = pt
                    if math.isnan(z) or math.isinf(z) or math.isnan(x) or math.isnan(y):
                        return None, None, None
                    return x, y, z
            return None, None, None
        except Exception as e:
            self.get_logger().error(f"PointCloud coordinate extraction error: {e}")
            return None, None, None

    def transform_to_map_frame(self, x, y, z, source_frame="camera_link"):
        """Transform coordinates from camera frame to map frame"""
        try:
            # Create a pose in the camera frame
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = source_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = float(z)
            pose_stamped.pose.orientation.w = 1.0

            # Transform to map frame
            transform = self.tf_buffer.lookup_transform('map', source_frame, rclpy.time.Time())
            transformed_pose = do_transform_pose(pose_stamped.pose, transform)
            
            return transformed_pose.position.x - 3.0, transformed_pose.position.y, transformed_pose.position.z
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return None, None, None

    def navigate_to_parking_sign(self, map_x, map_y):
        """Navigate to parking sign location using the same format as your example"""
        # Create goal dictionary in the same format as your example
        goal = {
            'x': map_x ,  # Stop 1m before the parking sign
            'y': map_y,
            'z': 0.0,
            'ox': 0.0,
            'oy': 0.0,
            'oz': 0.0,
            'ow': 1.0  # Facing forward
        }
        
        self.get_logger().info(f"Navigating to parking sign at: ({map_x:.2f}, {map_y:.2f})")
        self.send_goal(goal)

    def send_goal(self, goal):
        """Send navigation goal using the same pattern as your example"""
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

        self.navigate_action_client.wait_for_server()
        self._send_goal_future = self.navigate_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        pass

    def goal_response_callback(self, future):
        """Handle goal response using the same pattern as your example"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected :(')
            return

        self.get_logger().info('Navigation goal accepted.')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result and shutdown after completion"""
        result = future.result()
        self.get_logger().info('Navigation to parking sign completed! Shutting down...')
        
        # Kodu kapat
        cv2.destroyAllWindows()
        rclpy.shutdown()

    def color_image_callback(self, msg):
        # Eğer hedef konum zaten gönderilmişse, işlemeyi durdur
        if self.navigation_sent:
            return
            
        now = time.time()
        if now - self.last_detection_time < self.detection_interval:
            return
        self.last_detection_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            original_image = cv_image.copy()

            resized_image = cv2.resize(cv_image, (640, 360))
            scale_x = cv_image.shape[1] / 640
            scale_y = cv_image.shape[0] / 360

            results = self.model(resized_image, verbose=False)
            self.annotated_image = original_image

            detected_signs = {}
            sign_data = {}
            parking_sign_detected = False

            for r in results:
                for box in r.boxes:
                    if box.conf > 0.6:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        x1 = int(x1 * scale_x)
                        y1 = int(y1 * scale_y)
                        x2 = int(x2 * scale_x)
                        y2 = int(y2 * scale_y)
                        class_name = self.model.names[int(box.cls)]
                        confidence = float(box.conf)
                        detected_signs[class_name] = (x1, y1, x2, y2, 0, confidence)

            updated_tracked_signs = {}
            for class_name, values in self.tracked_signs.items():
                if class_name in detected_signs:
                    updated_tracked_signs[class_name] = detected_signs[class_name]
                elif values[4] < 5:
                    updated_tracked_signs[class_name] = (*values[:4], values[4] + 1, values[5])
            for class_name, values in detected_signs.items():
                if class_name not in updated_tracked_signs:
                    updated_tracked_signs[class_name] = values
            self.tracked_signs = updated_tracked_signs

            for class_name, (x1, y1, x2, y2, _, confidence) in self.tracked_signs.items():
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                distance = self.calculate_distance_pointcloud(center_x, center_y)

                if distance == -1:
                    distance = self.calculate_distance(x1, y1, x2, y2)

                if distance < 0.7 or distance > 25.0:
                    continue

                self._draw_box(x1, y1, x2, y2, class_name, distance, confidence)
                sign_data[class_name] = round(distance, 2)

                # Park levhası tespit edildi mi kontrol et
                if "park" in class_name.lower() or "durak" in class_name.lower():
                    parking_sign_detected = True
                    
                    # Koordinatları al ve sakla
                    camera_x, camera_y, camera_z = self.get_parking_sign_position_from_pointcloud(center_x, center_y)
                    if camera_x is not None:
                        map_x, map_y, map_z = self.transform_to_map_frame(camera_x, camera_y, camera_z)
                        if map_x is not None:
                            self.last_parking_coordinates = (map_x, map_y)

                if class_name.lower() == "kirmizi":
                    if distance <= 7.0:
                        publish_data = {class_name: round(distance, 2)}
                        msg = String()
                        msg.data = json.dumps(publish_data)
                        self.sign_publisher.publish(msg)
                        self.get_logger().info(f"Published (kirmizi levha): {msg.data}")
                    
                if (class_name.lower() == "durak" and distance <= 12.0) or (class_name.lower() != "durak" and distance <= 7.0):
                    publish_data = {class_name: round(distance, 2)}
                    msg = String()
                    msg.data = json.dumps(publish_data)
                    self.sign_publisher.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")

            # Park levhası tespit kontrolü
            if parking_sign_detected:
                self.consecutive_parking_detections += 1
                self.get_logger().info(f"Consecutive parking detections: {self.consecutive_parking_detections}/{self.required_consecutive_detections}")
                
                # 10 kez arka arkaya tespit edildi mi?
                if self.consecutive_parking_detections >= self.required_consecutive_detections and not self.navigation_sent:
                    if self.last_parking_coordinates is not None:
                        self.get_logger().info("10 consecutive parking sign detections reached! Sending navigation goal...")
                        self.navigate_to_parking_sign(self.last_parking_coordinates[0], self.last_parking_coordinates[1])
                        self.navigation_sent = True
                    else:
                        self.get_logger().warn("No valid parking coordinates available for navigation")
            else:
                # Park levhası tespit edilmedi, sayacı sıfırla
                if self.consecutive_parking_detections > 0:
                    self.get_logger().info("Parking sign not detected, resetting counter")
                self.consecutive_parking_detections = 0

            if sign_data:
                msg = String()
                msg.data = json.dumps(sign_data)
                self.sign_publisher.publish(msg)

            if self.annotated_image.shape[0] > 0:
                small_image = cv2.resize(self.annotated_image, (self.annotated_image.shape[1]//2, self.annotated_image.shape[0]//2))
                cv2.imshow("Levha Tespiti", small_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def calculate_distance_pointcloud(self, center_x, center_y):
        if self.latest_pointcloud is None:
            return -1
        try:
            width = self.latest_pointcloud.width
            height = self.latest_pointcloud.height
            center_x = min(center_x, width - 1)
            center_y = min(center_y, height - 1)
            index = center_y * width + center_x

            gen = point_cloud2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), skip_nans=False)
            for i, pt in enumerate(gen):
                if i == index:
                    x, y, z = pt
                    if math.isnan(z) or math.isinf(z):
                        return -1
                    return math.sqrt(x**2 + y**2 + z**2)
            return -1
        except Exception as e:
            self.get_logger().error(f"PointCloud error: {e}")
            return -1

    def calculate_distance(self, x1, y1, x2, y2):
        real_width = 0.5
        bbox_width = max(x2 - x1, 1)
        return (real_width * self.fx) / bbox_width * 1.7

    def _draw_box(self, x1, y1, x2, y2, class_name, distance, confidence):
        color = (0, 0, 255) if "park" == class_name.lower() or "durak" in class_name.lower() else (0, 255, 0)
        cv2.rectangle(self.annotated_image, (x1, y1), (x2, y2), color, 2)
        label = f"{class_name}: {distance:.2f}m ({confidence:.2f})"
        cv2.putText(self.annotated_image, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorWithNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
