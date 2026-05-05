#!/usr/bin/env python3
"""
Pose Estimator Node — Sim-Clean / Sim-Noisy / Real condition

Subscribes:
    /overhead_camera/color/image_raw   (sensor_msgs/Image)   - RGB image
    /overhead_camera/depth/image_raw   (sensor_msgs/Image)   - Depth in meters
    /overhead_camera/depth/camera_info (sensor_msgs/CameraInfo)
    /overhead_camera/depth/points      (sensor_msgs/PointCloud2)

Publishes:
    /detected_objects (geometry_msgs/PoseArray)
        Each pose is the centroid of one detected object in the camera frame.
        Order is fixed: [red_cube, blue_cylinder]

Pipeline (classical geometric, justified for sim-to-real noise study):
    1. RGB -> HSV color space (lighting-robust object detection, per supervisor advice)
    2. HSV thresholding to produce per-object binary masks
    3. Morphological cleanup (open + close) to remove noise
    4. RANSAC plane segmentation on point cloud to remove the table
       (Open3D segment_plane) - standard tabletop pattern in robotics literature
    5. For each object mask, extract the corresponding 3D points
    6. Compute centroid as pose estimate (translation only, rotation = identity)
    7. Publish as PoseArray in the camera optical frame

References:
    - Open3D RANSAC plane segmentation: open3d.org/docs/latest/tutorial/Basic/pointcloud.html
    - Team ACRV OCRTOC entry, ICRA 2021: arxiv.org/abs/2104.07188
    - RANSAC for Robotic Applications survey: PMC9824669

Author: Marcell Mor Szokedencsi (s5173191), University of Groningen
Supervisor: Dr. Hamidreza Kasaei
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration


# ──────────────────────────────────────────────────────────────────────
# HSV thresholds for the simulation cube (red) and cylinder (blue).
# Tune these by viewing the live mask topic and adjusting if objects
# are added/changed. Hue range is 0-179 in OpenCV, S/V are 0-255.
# ──────────────────────────────────────────────────────────────────────
HSV_RANGES = {
    "red_cube": [
        # Red wraps around the hue circle — need two ranges
        (np.array([0,   100, 80]),  np.array([10,  255, 255])),
        (np.array([170, 100, 80]),  np.array([179, 255, 255])),
    ],
    "blue_cylinder": [
        (np.array([100, 100, 50]), np.array([130, 255, 255])),
    ],
}

# RANSAC plane segmentation parameters (table removal)
RANSAC_DIST_THRESHOLD = 0.01   # 1 cm — points within this are inliers (table)
RANSAC_N = 3                   # min points to fit a plane
RANSAC_ITERATIONS = 1000

# Minimum number of valid 3D points required to declare an object detected
MIN_OBJECT_POINTS = 30

# Morphological kernel size for mask cleanup
MORPH_KERNEL_SIZE = 5


class PoseEstimator(Node):
    """Estimates 3D positions of colored objects on a tabletop scene."""

    def __init__(self):
        super().__init__('pose_estimator')

        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_rgb = None
        self.latest_depth = None

        # Output frame for detections (REP-105: 'world' is the inertial sim frame).
        # Detections are computed in the camera optical frame and transformed
        # via tf2 (Foote 2013) before publication, so downstream consumers can
        # compare against Gazebo ground truth directly.
        self.target_frame = self.declare_parameter(
            'target_frame', 'world').get_parameter_value().string_value

        # Depth topic is parameterised so we can switch between the clean
        # Gazebo stream and the noise_injection node's output without code
        # changes — required for sim-to-real ablation studies.
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/overhead_depth/depth/image_raw'
        ).get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # We need both the camera intrinsics and a sensor data QoS
        # because Gazebo publishes images with BEST_EFFORT reliability.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            '/overhead_depth/depth/camera_info',
            self._on_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            '/overhead_rgb/image_raw',
            self._on_rgb,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self._on_depth,
            sensor_qos,
        )

        # ── Publishers ─────────────────────────────────────────────────
        self.pose_pub = self.create_publisher(
            PoseArray, '/detected_objects', 10)

        # Debug topics — can visualize masks in RViz/rqt to tune HSV ranges
        self.mask_pub_red = self.create_publisher(
            Image, '/pose_estimator/mask_red', 5)
        self.mask_pub_blue = self.create_publisher(
            Image, '/pose_estimator/mask_blue', 5)

        # Process synchronized frames at 10 Hz (camera publishes at 30 Hz)
        self.timer = self.create_timer(0.1, self._process)

        self.get_logger().info('Pose estimator started.')
        self.get_logger().info('Waiting for camera_info, RGB, and depth...')

    # ── Subscription callbacks ────────────────────────────────────────
    def _on_camera_info(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info(
                f'Camera intrinsics received: '
                f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self.camera_info = msg

    def _on_rgb(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_rgb_header = msg.header
        except Exception as e:
            self.get_logger().error(f'RGB convert error: {e}')

    def _on_depth(self, msg: Image):
        try:
            # Depth from Gazebo plugin is 32FC1 in meters
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth convert error: {e}')

    # ── Main processing loop ──────────────────────────────────────────
    def _process(self):
        if (self.latest_rgb is None
                or self.latest_depth is None
                or self.camera_info is None):
            return

        rgb = self.latest_rgb
        depth = self.latest_depth

        # Sanity: shapes must match (some sims publish at different res)
        if rgb.shape[:2] != depth.shape[:2]:
            depth = cv2.resize(
                depth, (rgb.shape[1], rgb.shape[0]),
                interpolation=cv2.INTER_NEAREST)

        # 1) RGB -> HSV (per supervisor's advice; lighting-robust)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        # 2) Per-object color masks
        red_mask = self._color_mask(hsv, HSV_RANGES['red_cube'])
        blue_mask = self._color_mask(hsv, HSV_RANGES['blue_cylinder'])

        # Publish masks for debugging (view in rqt_image_view)
        self._publish_mask(red_mask, self.mask_pub_red)
        self._publish_mask(blue_mask, self.mask_pub_blue)

        # 3) Compute pose per object using masked depth + camera intrinsics
        camera_frame = self.latest_rgb_header.frame_id \
            or 'overhead_camera_optical_frame'
        stamp = self.latest_rgb_header.stamp

        red_pose = self._estimate_object_pose(red_mask, depth, 'red_cube')
        blue_pose = self._estimate_object_pose(blue_mask, depth, 'blue_cylinder')

        # 4) Transform from camera optical frame to target frame (default: 'world')
        #    via tf2 [Foote 2013]. Skip publishing if the transform is not yet
        #    available — robot_state_publisher needs ~1s after startup.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, camera_frame, stamp,
                timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(
                f'TF {self.target_frame}<-{camera_frame} unavailable: {e}',
                throttle_duration_sec=2.0)
            return

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.target_frame

        for pose in (red_pose, blue_pose):
            if pose is None:
                continue
            pose_array.poses.append(do_transform_pose(pose, tf))

        if pose_array.poses:
            self.pose_pub.publish(pose_array)

    # ── Helpers ───────────────────────────────────────────────────────
    @staticmethod
    def _color_mask(hsv_img, ranges):
        """Combine multiple HSV ranges (handles hue wrap-around for red)."""
        mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv_img, lo, hi)

        # Morphological cleanup: open removes specks, close fills holes
        k = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def _estimate_object_pose(self, mask, depth, label):
        """Project masked pixels to 3D and compute centroid in camera frame."""
        # Pixel coords where mask is positive AND depth is valid (>0, finite)
        ys, xs = np.where((mask > 0) & np.isfinite(depth) & (depth > 0))

        if len(xs) < MIN_OBJECT_POINTS:
            self.get_logger().debug(
                f'{label}: too few valid points ({len(xs)})')
            return None

        # Pinhole back-projection using camera intrinsics
        # X = (u - cx) * Z / fx ; Y = (v - cy) * Z / fy
        K = self.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        z = depth[ys, xs].astype(np.float64)
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy

        # Robust centroid: use median per axis to reject depth outliers
        cx3, cy3, cz3 = np.median(x), np.median(y), np.median(z)

        pose = Pose()
        pose.position.x = float(cx3)
        pose.position.y = float(cy3)
        pose.position.z = float(cz3)
        # Orientation: identity for now (cube/cylinder symmetric, refined later)
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        self.get_logger().info(
            f'{label}: ({cx3*1000:.1f}, {cy3*1000:.1f}, {cz3*1000:.1f}) mm '
            f'from {len(xs)} px')
        return pose

    def _publish_mask(self, mask, pub):
        try:
            msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Mask publish error: {e}')


def main():
    rclpy.init()
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()