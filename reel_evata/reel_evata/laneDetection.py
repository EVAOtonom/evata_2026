import rclpy
from std_msgs.msg import Int8
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.ximgproc
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import math
import os
import numpy as np
import time
import torch
from reel_evata.utils.utils import \
    time_synchronized, select_device, increment_path, \
    scale_coords, xyxy2xywh, non_max_suppression, split_for_trace_model, \
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result, \
    AverageMeter, LoadImages

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
     
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    
    return img, ratio, (dw, dh)

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/zedm/zed_node/rgb/image_rect_color',
            self.image_callback,
            1)
        self.create_subscription(PointCloud2, "/zedm/zed_node/point_cloud/cloud_registered", self.point_cloud_callback, 1)

        self.bridge = CvBridge()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        src_dir = dir_path.split('/install')[0]  # install kısmını çıkar
        weights = os.path.join(src_dir, 'src', 'reel_evata', 'reel_evata', 'utils', 'yolopv2.pt')
        device = "0"
        model = torch.jit.load(weights)
        device = select_device(device)
        half = device.type != 'cpu'  # half precision only supported on CUDA
        model = model.to(device)
        self.latest_pointcloud = None

        if half:
            model.half()  # to FP16  
        model.eval()
        self.model = model
        self.device = device
        self.filtered_pointcloud_pub = self.create_publisher(PointCloud2, '/lane_pointcloud', 10)
        self.image_center_x = 640  # Görselin orta noktası (1280x720 için)

        self.last_process_time = 0.0
        self.process_interval = 1.0 / 50  # 5 FPS
        self.cmd_vel_publisher = self.create_publisher(Int8, "/stm/steering_angle", 10)

#############
    def calculate_steering_angle(self, mid_points):
        """Orta noktaların x koordinatlarının ortalamasına göre dönme açısını hesapla."""
        if not mid_points:
            return 0.0  # Orta nokta yoksa düz git

        #Tüm orta noktaların x koordinatlarını al
        x_coords = [point[0] for point in mid_points]

        # X koordinatlarının ortalamasını hesapla
        avg_x = sum(x_coords) / len(x_coords)

        # Görselin orta noktasına göre sapma miktarını hesapla
        deviation = avg_x - self.image_center_x

        # Sapma miktarını normalize et (örneğin, -1.0 ile 1.0 arasında)
        max_deviation = self.image_center_x  # Maksimum sapma (640 piksel)
        steering_angle = deviation / max_deviation  # -1.0 (sola) ile 1.0 (sağa) arasında

        return steering_angle*140
       
    def publish_cmd_vel(self, steering_angle):
        msg = Int8()
        steering_value_int = int(steering_angle)
        msg.data = steering_value_int
        # Navigasyonla denenicekse alltaki satır yorum satırı. Sadece şerit takipse  yorumu kaldır. ****************************************************************
        #self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Publishing to /stm/steering_angle: {msg.data} (Calculated: {steering_angle:.2f})", throttle_duration_sec=0.5)



##############
    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_process_time < self.process_interval:
            return
        self.last_process_time = current_time

        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect(self.latest_image)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')
    
    def point_cloud_callback(self, msg):
        self.latest_pointcloud = msg
        # Store camera info if available (you might need to subscribe to camera info topic)
        if hasattr(msg, 'header'):
            self.latest_pointcloud_header = msg.header


    def detect(self, source, imgsz=640, conf_thres=0.3, iou_thres=0.45, 
               device='0', classes=None, agnostic_nms=False,):
        stride = 32
        model = self.model
        device = self.device
        half = device.type != 'cpu'
        img0 = cv2.resize(source, (1280,720), interpolation=cv2.INTER_LINEAR)
        #img0 = source
        img = letterbox(img0, imgsz, stride=stride)[0]
        
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        t0 = time.time()
        im0s = img0
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        with torch.no_grad():
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            t2 = time_synchronized()

        tw1 = time_synchronized()
        pred = split_for_trace_model(pred, anchor_grid)
        tw2 = time_synchronized()

        # Apply NMS
        t3 = time_synchronized()
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)
        t4 = time_synchronized()

        da_seg_mask = driving_area_mask(seg)
        ll_seg_mask = lane_line_mask(ll)
        

        
        ll_seg_mask_for_thinning_uint8 = (ll_seg_mask.astype(np.uint8) * 255)
        
        thinned_ll_mask_255 = cv2.ximgproc.thinning(ll_seg_mask_for_thinning_uint8, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
        
        thinned_ll_mask_for_show = (thinned_ll_mask_255 / 255).astype(ll_seg_mask.dtype)
        
        # Kameranın orta noktasını belirleme
        mid_point = im0s.shape[1] // 2

        # Şerit çizgilerinin koordinatlarını bulma
        y_coords, x_coords = np.where(thinned_ll_mask_for_show == 1)

        roi_y_start = 450
        roi_y_end = 720
                   
        sol_ust=0
        sag_ust=1280	
        sol_alt=0	
        sag_alt=1280
        
        def get_roi_x_bounds(y):
            if y < roi_y_start or y > roi_y_end:
                return None, None
            # Lineer interpolasyon ile x1 ve x2 değerlerini hesapla
            x1 = int(sol_ust + (y - roi_y_start) * (sol_alt - sol_ust) / (roi_y_end - roi_y_start))
            x2 = int(sag_ust + (y - roi_y_start) * (sag_alt - sag_ust) / (roi_y_end - roi_y_start))
            return x1, x2
                   

        roi_mask = (y_coords >= roi_y_start) & (y_coords <= roi_y_end)
        roi_y_coords = y_coords[roi_mask]
        roi_x_coords = x_coords[roi_mask]

        mid_points = []
        fallback_points = []  # Yedek noktalar için liste
        all_points = [] # PointCloud paylaşımı için
        


        # Her 10 y değeri için işlem yapma
        for y in range(roi_y_start, roi_y_end + 1):
            x1, x2 = get_roi_x_bounds(y)
            if x1 is None or x2 is None:
                continue

            # Belirli bir y değeri için x sınırlarını uygula
            y_mask = roi_y_coords == y
            x_values = roi_x_coords[y_mask & (roi_x_coords >= x1) & (roi_x_coords <= x2)]
            
            
            if len(x_values) >= 2:
                all_points.extend([(x, y) for x in x_values])
                # Birbirinden en az 100 piksel uzak olan iki x değeri bulma
                x_values_sorted = np.sort(x_values)
                x_diff = np.diff(x_values_sorted)
                valid_pairs = np.where(x_diff >= 400)[0]
                if len(valid_pairs) > 0:
                    x1_pair = x_values_sorted[valid_pairs[0]]
                    x2_pair = x_values_sorted[valid_pairs[0] + 1]
                    mid_x = (x1_pair + x2_pair) // 2
                    mid_points.append((mid_x, y))  # Orta noktayı listeye ekle
                    #print(f"y = {y}, x1 = {x1_pair}, x2 = {x2_pair}, Orta Nokta = ({mid_x}, {y})")
                else:
                    for x in x_values:
                        if x < self.image_center_x:
                            # Sol şerit ise: 335 pixel ekle
                            adjusted_x = x - 380
                            
                        else:
                            # Sağ şerit ise: 335 pixel çıkar
                            adjusted_x = x - 380

                        # ROI sınırları içinde kalacak şekilde ayarla
                        adjusted_x = max(x1, min(x2, adjusted_x))
                        if adjusted_x > x1:
                            fallback_points.append((adjusted_x, y))
                            #print(f"Fallback: y = {y}, min_x = {adjusted_x}, adjusted_x = {adjusted_x}")
        if all_points and self.latest_pointcloud is not None:
            try:
                matched_points = []
                pc_height = self.latest_pointcloud.height
                pc_width = self.latest_pointcloud.width

                # Get point cloud data as numpy array
                pc_data = point_cloud2.read_points_numpy(self.latest_pointcloud)

                # Reshape to height x width x fields
                pc_data = pc_data.reshape((pc_height, pc_width, -1))

                # Camera intrinsic parameters (adjust these to match your camera)
                fx = 700.0  # Focal length in pixels (x)
                fy = 700.0  # Focal length in pixels (y)
                cx = 640.0   # Principal point (x)
                cy = 360.0   # Principal point (y)

                for (x_pixel, y_pixel) in all_points:
                    # Convert from image coordinates to normalized device coordinates
                    u = x_pixel
                    v = y_pixel

                    # Skip if coordinates are out of bounds
                    if u < 0 or u >= pc_width or v < 0 or v >= pc_height:
                        continue

                    # Get corresponding 3D point
                    point = pc_data[int(v), int(u)]
                    x, y, z = point[:3]

                    # Filter invalid points
                    if not math.isnan(x) and not (math.isnan(y)) and not (math.isnan(z)):
                        # Transform to world coordinates if needed
                        # (This depends on your coordinate frames)
                        matched_points.append((x, y, z))

                # Hayali şerit noktaları (5 metre geride, 6 metre genişlikte)
                #fake_lane_points = []
                #z_pos = 0.1   # Zemin seviyesinde olacak (costmap'e girmesi için)
                #y_min = -5.0
                #y_max = 5.0
                #x_pos = -5.0
                #step = 0.2    # Nokta aralığı

                #y = y_min
                #while y <= y_max:
#                    fake_lane_points.append((
 #                       np.float32(x_pos),
  #                      np.float32(y),
   #                     np.float32(z_pos)
    #                ))
     #               y += step

                if matched_points:
                    # Tüm matched_points'i de float32 yap
                    matched_points = [
                        (np.float32(px), np.float32(py), np.float32(pz))
                        for px, py, pz in matched_points
                    ]

                    # Hayali şerit noktalarını ekle
                    #matched_points.extend(fake_lane_points)
                    header = self.latest_pointcloud_header
                    header.frame_id = "zed_camera_link"

                    fields = [
                        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    ]

                    pc2_msg = point_cloud2.create_cloud(header, fields, matched_points)
                    self.filtered_pointcloud_pub.publish(pc2_msg)

            except Exception as e:
                self.get_logger().error(f"Error creating filtered PointCloud: {str(e)}")

        # Eğer mid_points boşsa, fallback_points'i kullan
        if not mid_points and fallback_points:
            mid_points = fallback_points
            print("Using fallback points")

        # ROI alanını ana görselde belirginleştirme (dikdörtgen çizme)
        roi_top_left = (get_roi_x_bounds(roi_y_start)[0], roi_y_start)
        roi_top_right = (get_roi_x_bounds(roi_y_start)[1], roi_y_start)
        roi_bottom_left = (get_roi_x_bounds(roi_y_end)[0], roi_y_end)
        roi_bottom_right = (get_roi_x_bounds(roi_y_end)[1], roi_y_end)
        roi_pts = np.array([roi_top_left, roi_top_right, roi_bottom_right, roi_bottom_left], np.int32)
        roi_pts = roi_pts.reshape((-1, 1, 2))
        cv2.polylines(im0s, [roi_pts], isClosed=True, color=(255, 0, 0), thickness=2)


        if len(mid_points) >= 2:
            for i in range(1, len(mid_points)):
                cv2.line(im0s, mid_points[i - 1], mid_points[i], (0, 255, 0), thickness=2)
            # Orta noktaları daire olarak çiz
            for point in mid_points:
                cv2.circle(im0s, point, radius=5, color=(0, 0, 255), thickness=-1)

            # Tekerlek açısını hesapla ve gönder
            steering_angle = self.calculate_steering_angle(mid_points)
            # Navigasyonla denenicekse alltaki satır yorum satırı. Sadece şerit takipse  yorumu kaldır. ****************************************************************
            self.publish_cmd_vel(steering_angle)

            
        

            # Process detections
        for i, det in enumerate(pred):  # detections per image
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0s.shape).round()
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    plot_one_box(xyxy, im0s, line_thickness=3)
            # Show result
            show_seg_result(im0s, (da_seg_mask, thinned_ll_mask_for_show), is_demo=True)
            if len(mid_points) >= 2:
                for i in range(1, len(mid_points)):
                    cv2.line(im0s, mid_points[i - 1], mid_points[i], (0, 255, 0), thickness=2)
                # Orta noktaları daire olarak çiz
                for point in mid_points:
                    cv2.circle(im0s, point, radius=5, color=(0, 0, 255), thickness=-1)
            cv2.imshow("hasan",im0s)
            #cv2.imshow("ROI", roi_im0s)
            cv2.waitKey(1)
        #print(f'Done. ({time.time() - t0:.3f}s)')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
