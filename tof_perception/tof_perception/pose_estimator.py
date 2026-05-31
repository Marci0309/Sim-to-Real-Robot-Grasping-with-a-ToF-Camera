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
# Order of OBJECT_LABELS sets the order of poses in /detected_objects.
OBJECT_LABELS = ["coke_can", "banana", "golf_ball", "blue_box"]

HSV_RANGES = {
    "coke_can": [
        # Red wraps around the hue circle — need two ranges
        (np.array([  0, 120,  70]), np.array([ 10, 255, 255])),
        (np.array([170, 120,  70]), np.array([179, 255, 255])),
    ],
    "banana": [
        (np.array([ 20, 100, 100]), np.array([ 35, 255, 255])),
    ],
    "golf_ball": [
        # Magenta primitive — separated from red/banana to avoid hue overlap
        (np.array([140,  80,  80]), np.array([165, 255, 255])),
    ],
    "blue_box": [
        (np.array([100, 120,  50]), np.array([130, 255, 255])),
    ],
}

# Per-object overlay colors (BGR) for the annotated debug view.
VIZ_COLORS = {
    "coke_can":  (0, 0, 255),
    "banana":    (0, 200, 255),
    "golf_ball": (255, 0, 255),
    "blue_box":  (255, 0, 0),
}

# Minimum number of valid 3D points required to declare an object detected
MIN_OBJECT_POINTS = 30

# Morphological kernel size for HSV mask cleanup (labeling only — blob
# geometry comes from the depth-based segmentation below)
MORPH_KERNEL_SIZE = 5

# Depth-based object segmentation. We segment "object" pixels as those
# whose depth is significantly closer to the top-down camera than the
# table plane. This sidesteps texture-dependent biases in the HSV mask
# (e.g. the Coca-Cola label is off-centered on the cylinder mesh, so the
# label-only centroid lands ~85 mm from the can geometric center). HSV
# is still used, but only to *label* each depth blob with the right
# object name. Method follows analyze_object_noise.py.
OBJECT_HEIGHT_MIN_M = 0.005   # 5 mm above table → counts as object
MIN_OBJECT_AREA_PX  = 100     # smaller blobs are noise specks
# When matching a depth blob to an HSV label we require this many
# overlapping pixels — guards against random color noise pulling a blob.
MIN_HSV_OVERLAP_PX  = 50


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
        self.mask_pubs = {
            label: self.create_publisher(
                Image, f'/pose_estimator/mask_{label}', 5)
            for label in OBJECT_LABELS
        }

        # On-body grasp point per object (distance-transform peak of the depth
        # blob -> the thickest interior body point). For a curved banana this is
        # ON the body, unlike the area centroid which sits in the concave notch.
        self.grasp_pub = self.create_publisher(PoseArray, '/grasp_points', 10)
        # Annotated camera view: HSV tint + depth-blob outline + centroid + grasp.
        self.debug_pub = self.create_publisher(
            Image, '/pose_estimator/debug_view', 5)

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

        # 1) RGB -> HSV (per supervisor's advice; lighting-robust). HSV is
        #    now used only to label depth blobs with the correct object
        #    name — the actual silhouette comes from depth segmentation.
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        masks = {label: self._color_mask(hsv, HSV_RANGES[label])
                 for label in OBJECT_LABELS}
        for label, mask in masks.items():
            self._publish_mask(mask, self.mask_pubs[label])

        # 2) Depth-based silhouette segmentation: anything closer to the
        #    camera than the table plane by more than OBJECT_HEIGHT_MIN_M
        #    is an object pixel. Table plane is estimated as the median of
        #    valid depth values (table dominates a top-down FOV).
        above_table = self._above_table_mask(depth)

        # 3) Connected components: each object is one blob. cv2 wants u8.
        n_lab, lab, cc_stats, _ = cv2.connectedComponentsWithStats(
            above_table.astype(np.uint8), 8)

        # 4) For each blob, find which HSV mask overlaps it most, and use
        #    THAT as the label. This pairs the *depth-defined silhouette*
        #    (correct geometric extent) with the *color-defined identity*
        #    (correct object name).
        camera_frame = self.latest_rgb_header.frame_id \
            or 'overhead_camera_optical_frame'
        stamp = self.latest_rgb_header.stamp

        label_to_pose = {}
        label_to_grasp = {}
        viz = {}   # label -> (centroid_px, grasp_px) for the debug overlay
        for comp in range(1, n_lab):
            if cc_stats[comp, cv2.CC_STAT_AREA] < MIN_OBJECT_AREA_PX:
                continue
            blob = (lab == comp)
            best_label, best_overlap = None, 0
            for lbl, m in masks.items():
                ov = int(np.count_nonzero(blob & (m > 0)))
                if ov > best_overlap:
                    best_overlap, best_label = ov, lbl
            if best_label is None or best_overlap < MIN_HSV_OVERLAP_PX:
                continue
            if best_label in label_to_pose:
                continue  # already assigned to a (larger, earlier) blob
            p = self._estimate_pose_from_blob(blob, depth, best_label)
            if p is not None:
                label_to_pose[best_label] = p
                g = self._grasp_from_blob(blob, depth, masks[best_label])
                if g is not None:
                    label_to_grasp[best_label] = g[0]
                    viz[best_label] = (g[2], g[1])   # (centroid_px, grasp_px)

        # Annotated camera view (image-space; published every frame so it is
        # available even before tf comes up).
        self._publish_debug(rgb, masks, above_table, viz)

        # Preserve OBJECT_LABELS order in the published PoseArray
        poses = [label_to_pose.get(lbl) for lbl in OBJECT_LABELS]

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

        for pose in poses:
            if pose is None:
                continue
            pose_array.poses.append(do_transform_pose(pose, tf))

        if pose_array.poses:
            self.pose_pub.publish(pose_array)

        # Grasp points: same tf + header, one pose per detected object.
        grasp_array = PoseArray()
        grasp_array.header = pose_array.header
        for lbl in OBJECT_LABELS:
            g = label_to_grasp.get(lbl)
            if g is not None:
                grasp_array.poses.append(do_transform_pose(g, tf))
        if grasp_array.poses:
            self.grasp_pub.publish(grasp_array)

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

    @staticmethod
    def _above_table_mask(depth):
        """Pixels significantly closer to the camera than the table plane.

        Top-down camera: table is the FARTHEST visible surface, so its
        depth dominates the histogram. Estimate the table depth as the
        median of valid depths (table covers most of the FOV). Anything
        OBJECT_HEIGHT_MIN_M closer is an object.
        """
        valid = np.isfinite(depth) & (depth > 0)
        if not valid.any():
            return np.zeros_like(depth, dtype=bool)
        table_depth = float(np.median(depth[valid]))
        return valid & (depth < table_depth - OBJECT_HEIGHT_MIN_M)

    def _estimate_pose_from_blob(self, blob, depth, label):
        """Project blob pixels to 3D and compute centroid in camera frame."""
        ys, xs = np.where(blob & np.isfinite(depth) & (depth > 0))

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

    def _grasp_from_blob(self, blob, depth, hsv_mask):
        """On-body grasp point for a parallel-jaw gripper.

        Computed on the object's COLOURED pixels = depth blob INTERSECTED with
        the HSV mask, so the point is on the actual visible body (not a depth
        halo just past the edge). Then:
          - compact blob (can / box / ball): the area centroid is on the body,
            so grasp at the centroid (the geometric centre);
          - curved banana: the centroid falls in the concave notch (off-body),
            so grasp at the (smoothed) distance-transform PEAK = the most
            interior / thickest stretch of the body, where a parallel-jaw
            gripper has the most material to close on.

        Returns (Pose in camera frame, grasp_px, centroid_px) or None.
        """
        valid = blob & (hsv_mask > 0) & np.isfinite(depth) & (depth > 0)
        if int(np.count_nonzero(valid)) < MIN_OBJECT_POINTS:
            valid = blob & np.isfinite(depth) & (depth > 0)   # fallback
        ys, xs = np.where(valid)
        if len(xs) < MIN_OBJECT_POINTS:
            return None
        dist = cv2.distanceTransform(valid.astype(np.uint8), cv2.DIST_L2, 5)
        dmax = float(dist.max())
        if dmax <= 0.0:
            return None
        cenx, ceny = float(np.mean(xs)), float(np.mean(ys))
        ci, cj = int(round(ceny)), int(round(cenx))
        centroid_on_body = (0 <= ci < dist.shape[0] and 0 <= cj < dist.shape[1]
                            and dist[ci, cj] >= 0.5 * dmax)
        if centroid_on_body:
            gx_px, gy_px = int(round(cenx)), int(round(ceny))
        else:
            smooth = cv2.GaussianBlur(dist, (0, 0), 2.0)   # stabilise the peak
            _, _, _, (px, py) = cv2.minMaxLoc(smooth)
            gx_px, gy_px = int(px), int(py)
        gz = float(depth[gy_px, gx_px])
        if not np.isfinite(gz) or gz <= 0:
            return None
        K = self.camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        pose = Pose()
        pose.position.x = float((gx_px - cx) * gz / fx)
        pose.position.y = float((gy_px - cy) * gz / fy)
        pose.position.z = gz
        pose.orientation.w = 1.0
        return pose, (gx_px, gy_px), (int(cenx), int(ceny))

    def _publish_debug(self, rgb, masks, above_table, viz):
        """Annotated camera view: HSV-matched regions tinted per object, the
        depth-blob silhouette outlined, and the centroid (dot) + grasp point (X)
        marked. View live with: ros2 run rqt_image_view rqt_image_view
        (select /pose_estimator/debug_view)."""
        try:
            tint = rgb.copy()
            for lbl in OBJECT_LABELS:
                tint[masks[lbl] > 0] = VIZ_COLORS[lbl]
            dbg = cv2.addWeighted(tint, 0.35, rgb, 0.65, 0)
            cnts, _ = cv2.findContours(above_table.astype(np.uint8),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(dbg, cnts, -1, (0, 255, 0), 1)
            for lbl, (cen_px, grasp_px) in viz.items():
                cv2.circle(dbg, cen_px, 4, (255, 255, 255), -1)
                cv2.drawMarker(dbg, grasp_px, (0, 0, 0),
                               cv2.MARKER_CROSS, 16, 2)
                cv2.putText(dbg, lbl, (cen_px[0] + 6, cen_px[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                            cv2.LINE_AA)
            cv2.putText(dbg,
                        "tint=HSV  green=depth-blob  dot=centroid  X=grasp-point",
                        (8, dbg.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
        except Exception as e:
            self.get_logger().warn(f'debug view error: {e}')

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