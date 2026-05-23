#!/usr/bin/env python3
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from ultralytics import YOLO
#from message_filters import Subscriber, ApproximateTimeSynchronizer


class RGBDLiveNode(Node):
    def __init__(self):
        super().__init__('rgbd_live_node')

        # Parameters
        self.declare_parameter('rgb_topic', '/kinect/image_raw')
        self.declare_parameter('depth_topic', '/kinect/depth/image_raw')
        self.declare_parameter('rgb_info_topic', '/kinect/camera_info')
        self.declare_parameter('depth_info_topic', '/kinect/depth/camera_info')
        self.declare_parameter('debug_topic', '/rgbd_model/debug_image')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('sync_slop', 0.08)
        self.declare_parameter('show_window', True)
        self.declare_parameter('depth_scale', 0.001)  # 16UC1 mm -> meters
        self.declare_parameter('model_width', 320)
        self.declare_parameter('model_height', 240)

        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        rgb_info_topic = self.get_parameter('rgb_info_topic').get_parameter_value().string_value
        depth_info_topic = self.get_parameter('depth_info_topic').get_parameter_value().string_value
        debug_topic = self.get_parameter('debug_topic').get_parameter_value().string_value
        queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value

        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        self.model_width = self.get_parameter('model_width').get_parameter_value().integer_value
        self.model_height = self.get_parameter('model_height').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.model_pallet = YOLO("/home/montasser/ws_lidar/models/bestt.pt")
        self.model_pocket = YOLO("/home/montasser/ws_lidar/models/besttt.pt")
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)

        self.rgb_K: Optional[np.ndarray] = None
        self.depth_K: Optional[np.ndarray] = None
        self.rgb_D: Optional[np.ndarray] = None
        self.depth_D: Optional[np.ndarray] = None

        self.create_subscription(CameraInfo, rgb_info_topic, self.rgb_info_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_cb, qos_profile_sensor_data)

        self.latest_rgb_msg = None
        self.latest_depth_msg = None
        self.last_processed_time = 0.0

        self.create_subscription(Image, rgb_topic, self.rgb_cb, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self.depth_cb, qos_profile_sensor_data)

        self.last_log_time = time.time()
        self.frame_count = 0
        self.total_infer_ms = 0.0

        self.get_logger().info('RGB-D live node started.')
        self.get_logger().info(f'RGB topic:   {rgb_topic}')
        self.get_logger().info(f'Depth topic: {depth_topic}')
        self.get_logger().info(f'RGB info:    {rgb_info_topic}')
        self.get_logger().info(f'Depth info:  {depth_info_topic}')
        self.get_logger().info(f'Debug topic: {debug_topic}')
    def rgb_cb(self, msg: Image) -> None:
        if self.latest_rgb_msg is None:
            self.get_logger().info('First RGB frame received')
    
        self.latest_rgb_msg = msg
        self.try_process()

    def depth_cb(self, msg: Image) -> None:
        if self.latest_depth_msg is None:
            self.get_logger().info('First depth frame received')
    
        self.latest_depth_msg = msg
        self.try_process()

    def try_process(self) -> None:
        if self.latest_rgb_msg is None or self.latest_depth_msg is None:
            return

        now = time.time()

        # limit processing a little so we do not process too many duplicate pairs
        if now - self.last_processed_time < 0.03:
            return

        self.last_processed_time = now
        self.synced_cb(self.latest_rgb_msg, self.latest_depth_msg)
    def rgb_info_cb(self, msg: CameraInfo) -> None:
        self.rgb_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.rgb_D = np.array(msg.d, dtype=np.float64)

    def depth_info_cb(self, msg: CameraInfo) -> None:
        self.depth_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.depth_D = np.array(msg.d, dtype=np.float64)

    def synced_cb(self, rgb_msg: Image, depth_msg: Image) -> None:
        t0 = time.perf_counter()
        self.get_logger().info('Received synchronized RGB-D frame')
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f'Failed to convert RGB image: {e}')
            return

        try:
            # Keep native depth format
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        # Convert depth to float meters
        depth_m = self.convert_depth_to_meters(depth)

        # Preprocess for model
        rgb_in, depth_in = self.preprocess(rgb, depth_m)

        # Run model
        result = self.run_model(rgb_in, depth_in)

        # Visualize on original image
        debug = self.draw_result(rgb.copy(), depth_m, result)

        # Stats
        infer_ms = (time.perf_counter() - t0) * 1000.0
        self.frame_count += 1
        self.total_infer_ms += infer_ms

        now = time.time()
        if now - self.last_log_time >= 1.0:
            avg_ms = self.total_infer_ms / max(self.frame_count, 1)
            fps = self.frame_count / max(now - self.last_log_time, 1e-6)
            self.get_logger().info(
                f'Live RGB-D | fps={fps:.2f} | avg_total_ms={avg_ms:.2f}'
            )
            self.last_log_time = now
            self.frame_count = 0
            self.total_infer_ms = 0.0

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = rgb_msg.header
        self.debug_pub.publish(debug_msg)

        # Optional local display
        if self.show_window:
            cv2.imshow('RGB-D Model Debug', debug)
            cv2.waitKey(1)

    def convert_depth_to_meters(self, depth: np.ndarray) -> np.ndarray:
        """
        Supports common cases:
        - uint16 depth in mm -> meters
        - float32 depth already in meters
        """
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * float(self.depth_scale)
        elif depth.dtype == np.float32:
            depth_m = depth.copy()
        else:
            depth_m = depth.astype(np.float32)

        # Mark invalid values
        depth_m[~np.isfinite(depth_m)] = 0.0
        depth_m[depth_m < 0.0] = 0.0
        return depth_m

    def preprocess(self, rgb: np.ndarray, depth_m: np.ndarray):
        return rgb, depth_m

    def run_model(self, rgb_in: np.ndarray, depth_in: np.ndarray) -> dict:
        results_pallet = self.model_pallet(rgb_in, verbose=False)
        results_pocket = self.model_pocket(rgb_in, verbose=False)

        detections = []

        # Pallets
        if len(results_pallet) > 0 and len(results_pallet[0].boxes) > 0:
            for box in results_pallet[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = f"{self.model_pallet.names[cls_id]} {conf:.2f}"

                detections.append({
                    "type": "pallet",
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "label": label,
                    "conf": conf,
                })

        # Pockets
        if len(results_pocket) > 0 and len(results_pocket[0].boxes) > 0:
            for box in results_pocket[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = f"{self.model_pocket.names[cls_id]} {conf:.2f}"

                detections.append({
                    "type": "pocket",
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "label": label,
                    "conf": conf,
                })

        return {
            "found": len(detections) > 0,
            "detections": detections
        }
    def rgb_info_cb(self, msg: CameraInfo) -> None:
        self.rgb_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.rgb_D = np.array(msg.d, dtype=np.float64)

    def depth_info_cb(self, msg: CameraInfo) -> None:
        self.depth_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.depth_D = np.array(msg.d, dtype=np.float64)
        
    def draw_result(self, rgb: np.ndarray, depth_m: np.ndarray, result: dict) -> np.ndarray:
        h, w = rgb.shape[:2]

        # Small depth colormap preview
        depth_vis = np.clip(depth_m, 0.0, 5.0)
        depth_vis = (depth_vis / 5.0 * 255.0).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        depth_vis = cv2.resize(depth_vis, (w // 4, h // 4))
        rgb[10:10 + depth_vis.shape[0], 10:10 + depth_vis.shape[1]] = depth_vis

        if not result.get("found", False):
            cv2.putText(
                rgb, "No detections", (10, h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA
            )
            return rgb

        for det in result["detections"]:
            x1, y1, x2, y2 = det["x1"], det["y1"], det["x2"], det["y2"]
            label = det["label"]

            if det["type"] == "pallet":
                color = (255, 0, 0)   # blue
            else:
                color = (0, 255, 0)   # green

            cv2.rectangle(rgb, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                rgb, label, (x1, max(20, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA
            )

            # Depth at center of box
            cx_box = int((x1 + x2) / 2)
            cy_box = int((y1 + y2) / 2)

            if 0 <= cx_box < depth_m.shape[1] and 0 <= cy_box < depth_m.shape[0]:
                z = float(depth_m[cy_box, cx_box])

                cv2.circle(rgb, (cx_box, cy_box), 4, color, -1)

                if z > 0.0:
                    depth_text = f"Z={z:.2f}m"
                    cv2.putText(
                        rgb, depth_text, (x1, min(h - 10, y2 + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA
                    )

                    # Optional 3D coordinates using depth intrinsics
                    if self.depth_K is not None:
                        fx = self.depth_K[0, 0]
                        fy = self.depth_K[1, 1]
                        cx = self.depth_K[0, 2]
                        cy = self.depth_K[1, 2]

                        X = (cx_box - cx) * z / fx
                        Y = (cy_box - cy) * z / fy

                        xyz_text = f"X={X:.2f} Y={Y:.2f} Z={z:.2f}"
                        cv2.putText(
                            rgb, xyz_text, (x1, min(h - 10, y2 + 40)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA
                        )

        return rgb


def main(args=None):
    rclpy.init(args=args)
    node = RGBDLiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
