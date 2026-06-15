import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge  # converts between ROS `Image` messages and OpenCV numpy arrays

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


# Colours per model: (B, G, R) for OpenCV drawing
_MODEL_COLOURS_BGR = [(0, 200, 0), (200, 100, 0)]   # green, blue
# RGBA for RViz markers
_MODEL_COLOURS_RGBA = [(0.0, 0.8, 0.0, 0.8), (0.0, 0.4, 0.8, 0.8)]
# Estimated-depth marker colour (yellow) — used when depth comes from bounding box size
_ESTIMATED_COLOUR_RGBA = (0.9, 0.9, 0.0, 0.8)


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('model1_path', '')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('fixed_frame', 'camera_depth_optical_frame') # coordinate frame for 3D positions
        self.declare_parameter('handle_real_width_m', 0.02)

        model1_path = self.get_parameter('model1_path').get_parameter_value().string_value
        self.threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.handle_real_width_m = self.get_parameter('handle_real_width_m').get_parameter_value().double_value

        if not model1_path:
            self.get_logger().error(
                'model1_path parameter must be set. '
                'Pass it via launch file or --ros-args -p model1_path:=/path/to/model.pt'
            )
            raise SystemExit(1)

        from ultralytics import YOLO
        self.get_logger().info(f'Loading model: {model1_path}')
        self.model1 = YOLO(model1_path)
        self.get_logger().info('Model loaded.')

        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_depth = None  # stores the latest depth Image msg, so RGB cb can use it without synchronization

        self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self._camera_info_cb,
            10,
        )

        # Independent subscribers — store latest depth, process in RGB callback
        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self._image_cb, 10)

        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/perception/annotated_image', 10) # annotated image for RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/object_markers', 10) # 3D marker for grasp node

        self.get_logger().info('PerceptionNode ready — waiting for images.')

    # ------------------------------------------------------------------
    # _camera_info_cb() only saves first camera intrinsincs message
    def _camera_info_cb(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(
                f'Camera intrinsics received: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}'
            )
    # ------------------------------------------------------------------
    # _depth_cb() overwrites stored depth message with new one
    def _depth_cb(self, msg):
        self.latest_depth = msg

    # ------------------------------------------------------------------
    # _image_cb handles() detection, depth lookup, 3D projection and publishing
    def _image_cb(self, rgb_msg):
        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera_info...', throttle_duration_sec=5.0)
            return
        if self.latest_depth is None:
            self.get_logger().warn('Waiting for depth image...', throttle_duration_sec=5.0)
            return

        # Convert ROS images to numpy
        rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='passthrough')
        # depth_cv is float32 in metres from libgazebo_ros_camera depth sensor
        if depth_cv.dtype != np.float32:
            depth_cv = depth_cv.astype(np.float32) / 1000.0  # mm → m fallback

        # Camera intrinsics
        # K = [ fx   0  cx ]
        #     [  0  fy  cy ]
        #     [  0   0   1 ]
        # fx, fy - focal lenght in pixels     cx, cy - principal points
        K = self.camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        results = [self.model1(rgb_cv, conf=self.threshold, verbose=False)]

        annotated = rgb_cv.copy()
        marker_array = MarkerArray()
        marker_id = 0

        for model_idx, model_results in enumerate(results):
            colour_bgr = _MODEL_COLOURS_BGR[model_idx]
            colour_rgba = _MODEL_COLOURS_RGBA[model_idx]
            boxes = model_results[0].boxes

            for box in boxes:
                # Bounding box corners
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist()) # bounding box pixel coordinates 1-top left 2- bottom right
                conf = float(box.conf[0]) # confidence
                cls_id = int(box.cls[0]) # class
                cls_name = model_results[0].names[cls_id] # class name - 'handle'

                # Centre pixel
                u = (x1 + x2) // 2
                v = (y1 + y2) // 2

                # Depth lookup — median over a 9x9 patch for robustness
                h, w = depth_cv.shape[:2]
                u_c = max(0, min(u, w - 1)) # clamping to w-1 and h-1 to prevent reading outside the image
                v_c = max(0, min(v, h - 1))
                patch = depth_cv[max(0, v_c - 4):v_c + 5, max(0, u_c - 4):u_c + 5]
                valid = patch[(patch > 0.1) & np.isfinite(patch)]
                depth_m = float(np.median(valid)) if valid.size > 0 else 0.0

                depth_estimated = False
                # bounding box depth estimation fallback
                # Pinhole camera model: pixel_width = (real_width * fx) / depth
                if depth_m <= 0.0 or math.isnan(depth_m) or math.isinf(depth_m):
                    # For handles (model1), fall back to bounding box size estimation
                    pixel_width = x2 - x1
                    if model_idx == 0 and pixel_width > 0 and self.handle_real_width_m > 0:
                        depth_m = (fx * self.handle_real_width_m) / pixel_width
                        depth_label = f'~{depth_m:.2f}m'
                        has_depth = True
                        depth_estimated = True
                    else:
                        depth_label = '?m'
                        has_depth = False
                else:
                    depth_label = f'{depth_m:.2f}m'
                    has_depth = True

                # Draw bounding box and label
                cv2.rectangle(annotated, (x1, y1), (x2, y2), colour_bgr, 2)
                label = f'{cls_name} {conf:.2f}  {depth_label}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw + 4, y1), colour_bgr, -1)
                cv2.putText(
                    annotated, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
                )

                if not has_depth:
                    continue

                # 3D position in camera_depth_optical_frame: Z forward, X right, Y down
                # Pinhole camera back-projection formula: u = fx * X/Z + cx
                X = (u_c - cx) * depth_m / fx
                Y = (v_c - cy) * depth_m / fy
                Z = depth_m

                # Sphere marker at object centre
                sphere = Marker()
                sphere.header.frame_id = self.fixed_frame
                sphere.header.stamp = rgb_msg.header.stamp
                sphere.ns = f'model{model_idx + 1}_sphere'
                sphere.id = marker_id
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position = Point(x=X, y=Y, z=Z)
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08
                c = _ESTIMATED_COLOUR_RGBA if depth_estimated else colour_rgba
                sphere.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
                sphere.lifetime = Duration(sec=1)
                marker_array.markers.append(sphere)
                marker_id += 1

                # Text marker above the sphere
                text = Marker()
                text.header = sphere.header
                text.ns = f'model{model_idx + 1}_text'
                text.id = marker_id
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position = Point(x=X, y=Y - 0.12, z=Z)
                text.pose.orientation.w = 1.0
                text.scale.z = 0.07
                text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text.text = f'{cls_name}\n{depth_m:.2f}m'
                text.lifetime = Duration(sec=1)
                marker_array.markers.append(text)
                marker_id += 1

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.annotated_pub.publish(out_msg)

        # Publish markers
        self.marker_pub.publish(marker_array)
    # -----------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
