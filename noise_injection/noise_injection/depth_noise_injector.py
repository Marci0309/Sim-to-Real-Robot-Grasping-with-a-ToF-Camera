#!/usr/bin/env python3
"""
Depth Noise Injector — augments a clean simulated depth image with
literature-grounded RGB-D / ToF sensor noise so the perception pipeline
can be evaluated under realistic degradation.

Subscribes:
    <input_topic>          (sensor_msgs/Image, 32FC1, depth in metres)
    <input_info_topic>     (sensor_msgs/CameraInfo) - mirrored only

Publishes:
    <output_topic>         (sensor_msgs/Image, 32FC1, noised depth)
    <output_info_topic>    (sensor_msgs/CameraInfo) - identical to input

Noise model (BSc thesis scope; spatially-iid; multipath out of scope):
    1. Axial Gaussian:    sigma_z(z) = a + b*(z - z0)^2  per pixel
       Source: Nguyen, Izadi, Lovell, "Modeling Kinect Sensor Noise for
               Improved 3D Reconstruction and Tracking", 3DIMPVT 2012,
               Eq. 3. (a, b, z0) = (0.0012, 0.0019, 0.4) for the Kinect.
    2. Lateral pixel jitter: per-pixel (du, dv) ~ N(0, sigma_L^2 px)
       Source: Nguyen 2012, Eq. 1-2 (incidence-angle term dropped — the
               overhead camera looks down the table normal, so theta ~ 0
               and the angular term is negligible).
    3. Edge / flying pixels: at pixels where ||grad z|| > tau, with
       probability p_edge replace the depth by alpha*z_near + (1-alpha)*z_far,
       alpha ~ U(0,1). Source: Foix, Alenya, Torras, "Lock-in ToF Cameras:
       A Survey", IEEE Sensors J. 2011; Sarbolandi, Lefloch, Kolb, "Kinect
       range sensing: SL vs ToF", CVIU 2015.
    4. Random dropout: Bernoulli(p_drop) per pixel -> NaN.
       Source: Sarbolandi 2015 §4 reports outlier rates 1-5%% in real ToF.

Reproducibility: numpy default_rng with two independent streams (Gaussian,
mask) so toggling dropout/edge does not shift the Gaussian sequence.
The seed is exposed as a ROS parameter and logged at startup.

Limitations (acknowledged for thesis honesty):
 - Spatial correlation between neighbouring pixels not modelled (real ToF
   noise is correlated; Foix 2011, Sarbolandi 2015).
 - Multipath / mixed-pixel artefacts at scene concavities not modelled.
 - Quantization omitted: at z=1.4 m the Khoshelham 2012 disparity-quantum
   is < 0.1 mm, well below other noise sources.

Author: Marcell Mor Szokedencsi (s5173191), University of Groningen
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class DepthNoiseInjector(Node):
    def __init__(self):
        super().__init__('depth_noise_injector')

        # ── Parameters (all values traceable to the cited papers) ──────
        # Topic plumbing
        self.declare_parameter('input_topic',       '/overhead_depth/depth/image_raw')
        self.declare_parameter('output_topic',      '/overhead_depth/depth/image_noisy')
        self.declare_parameter('input_info_topic',  '/overhead_depth/depth/camera_info')
        self.declare_parameter('output_info_topic', '/overhead_depth/depth/camera_info_noisy')

        # Axial Gaussian — Nguyen 2012 Eq.3 coefficients for Kinect
        self.declare_parameter('axial_a',  0.0012)
        self.declare_parameter('axial_b',  0.0019)
        self.declare_parameter('axial_z0', 0.4)
        self.declare_parameter('axial_scale', 1.0)   # multiplier for ablation curves

        # Lateral pixel jitter — Nguyen 2012 Eq.1 const term
        self.declare_parameter('lateral_sigma_px', 0.8)

        # Edge / flying pixels — Foix 2011, Sarbolandi 2015
        self.declare_parameter('edge_grad_threshold_m', 0.05)  # 5 cm depth jump
        self.declare_parameter('edge_probability',      0.30)

        # Dropout — Sarbolandi 2015 §4 (1–5%% baseline)
        self.declare_parameter('dropout_probability',   0.02)

        # RNG seed — Tobin 2017 §3 recommends seeding ablations
        self.declare_parameter('seed', 42)

        # Resolve once at init (kept const for the run; restart to change)
        self.in_topic       = self.get_parameter('input_topic').value
        self.out_topic      = self.get_parameter('output_topic').value
        self.in_info_topic  = self.get_parameter('input_info_topic').value
        self.out_info_topic = self.get_parameter('output_info_topic').value
        self.a              = self.get_parameter('axial_a').value
        self.b              = self.get_parameter('axial_b').value
        self.z0             = self.get_parameter('axial_z0').value
        self.axial_scale    = self.get_parameter('axial_scale').value
        self.lat_sigma      = self.get_parameter('lateral_sigma_px').value
        self.edge_tau       = self.get_parameter('edge_grad_threshold_m').value
        self.p_edge         = self.get_parameter('edge_probability').value
        self.p_drop         = self.get_parameter('dropout_probability').value
        seed                = self.get_parameter('seed').value

        # Two independent streams: continuous noise vs discrete masks.
        # Keeps ablations independent — toggling dropout doesn't reshuffle
        # the Gaussian draws (cf. Tobin 2017).
        ss = np.random.SeedSequence(seed)
        s1, s2 = ss.spawn(2)
        self.rng_gauss = np.random.default_rng(s1)
        self.rng_mask  = np.random.default_rng(s2)

        # ── ROS plumbing ───────────────────────────────────────────────
        # SensorDataQoS: BEST_EFFORT, depth=5 — matches Gazebo camera plugin.
        # A mismatched QoS would silently fail to subscribe.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.bridge = CvBridge()
        self.pub_depth = self.create_publisher(Image, self.out_topic, sensor_qos)
        self.pub_info  = self.create_publisher(CameraInfo, self.out_info_topic, sensor_qos)

        self.create_subscription(Image, self.in_topic,
                                 self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self.in_info_topic,
                                 self._on_info, sensor_qos)

        self.get_logger().info(
            f'depth_noise_injector started | seed={seed} '
            f'axial_scale={self.axial_scale} p_drop={self.p_drop} '
            f'p_edge={self.p_edge}')
        self.get_logger().info(f'  in : {self.in_topic}')
        self.get_logger().info(f'  out: {self.out_topic}')

    # ── Camera info pass-through (intrinsics unchanged by noise) ───────
    def _on_info(self, msg: CameraInfo):
        self.pub_info.publish(msg)

    # ── Main callback: noise pipeline ──────────────────────────────────
    def _on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, '32FC1').astype(np.float32)
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        # Validity mask: never inject "valid" noise into already-invalid
        # pixels. Gazebo's depth plugin emits 0 or NaN outside [near, far].
        valid = np.isfinite(depth) & (depth > 0)

        out = depth.copy()

        # 1) Axial Gaussian — Nguyen 2012 Eq.3
        # sigma_z(z) = a + b * (z - z0)^2  [m]
        # Per-pixel sigma depends on local depth, so noise scales with range.
        sigma_z = self.a + self.b * (depth - self.z0) ** 2          # (H,W)
        sigma_z = np.clip(sigma_z, 0.0, None) * self.axial_scale
        noise_z = self.rng_gauss.standard_normal(depth.shape).astype(np.float32) \
                  * sigma_z.astype(np.float32)
        out = out + noise_z * valid                                 # zero-out invalid

        # 2) Lateral pixel jitter — Nguyen 2012 Eq.1 (theta ~ 0 for overhead)
        # Each output pixel samples its source from a nearby pixel offset by
        # (du, dv) ~ N(0, sigma_L^2). This re-projects edges with the same
        # spatial uncertainty as the real sensor.
        if self.lat_sigma > 0.0:
            h, w = depth.shape
            vv, uu = np.mgrid[0:h, 0:w].astype(np.int32)
            du = self.rng_gauss.normal(0.0, self.lat_sigma, depth.shape).astype(np.int32)
            dv = self.rng_gauss.normal(0.0, self.lat_sigma, depth.shape).astype(np.int32)
            src_u = np.clip(uu + du, 0, w - 1)
            src_v = np.clip(vv + dv, 0, h - 1)
            out = out[src_v, src_u]
            valid = valid[src_v, src_u]

        # 3) Edge / flying pixels — Foix 2011; Sarbolandi 2015
        # Compute gradient on ORIGINAL depth (so edges are not noise-driven).
        # Where |grad| > tau, with prob p_edge replace by alpha*z_near +
        # (1-alpha)*z_far. We approximate (z_near, z_far) by (min, max) over
        # a 3x3 neighbourhood — the standard "mixed pixel" formulation.
        if self.p_edge > 0.0:
            gx = np.abs(np.gradient(depth, axis=1))
            gy = np.abs(np.gradient(depth, axis=0))
            edge_mask = (gx + gy) > self.edge_tau
            flip = self.rng_mask.random(depth.shape) < self.p_edge
            sel = edge_mask & flip & valid
            if np.any(sel):
                # 3x3 min/max via simple shifts (avoid scipy dependency).
                shifted = np.stack([
                    np.roll(depth,  1, axis=0), np.roll(depth, -1, axis=0),
                    np.roll(depth,  1, axis=1), np.roll(depth, -1, axis=1),
                    depth,
                ], axis=0)
                z_near = shifted.min(axis=0)
                z_far  = shifted.max(axis=0)
                alpha = self.rng_mask.random(depth.shape).astype(np.float32)
                mixed = alpha * z_near + (1.0 - alpha) * z_far
                out = np.where(sel, mixed, out)

        # 4) Random dropout — Sarbolandi 2015
        # Set NaN so downstream `np.isfinite` filters drop them, mimicking
        # real ToF "no return" pixels.
        if self.p_drop > 0.0:
            drop = self.rng_mask.random(depth.shape) < self.p_drop
            out[drop] = np.nan

        # Re-stamp with the INPUT header (preserve sim time + frame_id) so
        # TF lookups on the consumer side stay valid (Foote 2013).
        out_msg = self.bridge.cv2_to_imgmsg(out.astype(np.float32), encoding='32FC1')
        out_msg.header = msg.header
        self.pub_depth.publish(out_msg)


def main():
    rclpy.init()
    node = DepthNoiseInjector()
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
