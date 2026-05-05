#!/usr/bin/env python3
"""
Pose Evaluator — compares /detected_objects against Gazebo ground truth.

Subscribes:
    /detected_objects  (geometry_msgs/PoseArray, frame_id='world')
        Order is fixed: [red_cube, blue_cylinder] (per pose_estimator).
    /model_states      (gazebo_msgs/ModelStates)
        Provides world-frame poses of every spawned model.

Logs per-frame Euclidean position error to a CSV file:
    timestamp, cube_err_x, cube_err_y, cube_err_z, cube_err_norm,
               cyl_err_x,  cyl_err_y,  cyl_err_z,  cyl_err_norm

Ground truth: the depth-based centroid in pose_estimator estimates the
*visible top surface* of each object, not its geometric centre, because
an overhead camera only sees top pixels. We therefore compare against
top-surface positions (model centre + half_height), per the visible-vs-
full-pose distinction in Hodan et al., "BOP: Benchmark for 6D Object
Pose Estimation", ECCV 2018, §4. Half-heights are exposed as ROS
parameters so they can be adjusted if the world geometry changes.

Metrics computed on shutdown (Hodan 2018, §3.2 conventions):
 - Mean absolute error per axis
 - RMSE per axis and 3D Euclidean RMSE
 - Detection rate = #frames with detection / #frames with ground truth

Author: Marcell Mor Szokedencsi (s5173191), University of Groningen
"""

import csv
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates


class PoseEvaluator(Node):
    def __init__(self):
        super().__init__('pose_evaluator')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('detected_topic',  '/detected_objects')
        self.declare_parameter('truth_topic',     '/model_states')
        self.declare_parameter('cube_model_name', 'cube')
        self.declare_parameter('cyl_model_name',  'cylinder')
        # Half-heights add to GT centre to give the visible-top z-coord.
        # Cube edge 5 cm -> half = 0.025; cylinder length 8 cm -> half = 0.04.
        self.declare_parameter('cube_half_height', 0.025)
        self.declare_parameter('cyl_half_height',  0.040)
        # Output CSV path; each run appends so sweeps can concatenate.
        default_csv = os.path.expanduser('~/thesis_ws/src/evaluation/results/run.csv')
        self.declare_parameter('output_csv', default_csv)
        # Tag string written into every row so multiple runs can be
        # distinguished post-hoc (e.g. "axial_scale=2.0").
        self.declare_parameter('run_tag', 'default')

        self.detected_topic  = self.get_parameter('detected_topic').value
        self.truth_topic     = self.get_parameter('truth_topic').value
        self.cube_name       = self.get_parameter('cube_model_name').value
        self.cyl_name        = self.get_parameter('cyl_model_name').value
        self.cube_dz         = self.get_parameter('cube_half_height').value
        self.cyl_dz          = self.get_parameter('cyl_half_height').value
        self.csv_path        = self.get_parameter('output_csv').value
        self.run_tag         = self.get_parameter('run_tag').value

        # ── Internal state ─────────────────────────────────────────────
        # Latest GT poses (numpy 3-vectors), updated by callback.
        self.gt_cube = None
        self.gt_cyl  = None
        # Per-axis error history (lists of np.ndarray shape (3,)) for
        # final RMSE/MAE computation.
        self.cube_errors = []
        self.cyl_errors  = []
        self.frames_with_truth    = 0
        self.frames_with_cube_det = 0
        self.frames_with_cyl_det  = 0

        # CSV writer — open append, write header only if file is new.
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        write_header = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv = csv.writer(self.csv_file)
        if write_header:
            self.csv.writerow([
                'tag', 't_sec', 't_nsec',
                'cube_dx', 'cube_dy', 'cube_dz', 'cube_norm',
                'cyl_dx',  'cyl_dy',  'cyl_dz',  'cyl_norm',
            ])

        # ── ROS plumbing ───────────────────────────────────────────────
        # /model_states is published RELIABLE/VOLATILE by gazebo_ros_state,
        # /detected_objects RELIABLE by our pose_estimator -> default QoS OK.
        self.create_subscription(PoseArray,
                                 self.detected_topic, self._on_detected, 10)
        self.create_subscription(ModelStates,
                                 self.truth_topic, self._on_truth, 10)

        self.get_logger().info(
            f'pose_evaluator started | tag="{self.run_tag}" '
            f'csv={self.csv_path}')

    # ── Callbacks ─────────────────────────────────────────────────────
    def _on_truth(self, msg: ModelStates):
        # Resolve named models -> world-frame xyz, plus per-object z-offset
        # so we compare against the visible top surface (Hodan 2018).
        try:
            i_cube = msg.name.index(self.cube_name)
            p = msg.pose[i_cube].position
            self.gt_cube = np.array([p.x, p.y, p.z + self.cube_dz])
        except ValueError:
            pass
        try:
            i_cyl = msg.name.index(self.cyl_name)
            p = msg.pose[i_cyl].position
            self.gt_cyl = np.array([p.x, p.y, p.z + self.cyl_dz])
        except ValueError:
            pass

    def _on_detected(self, msg: PoseArray):
        # Need GT before we can score. Wait silently otherwise.
        if self.gt_cube is None or self.gt_cyl is None:
            return
        self.frames_with_truth += 1

        # pose_estimator's contract: poses are appended in the order
        # [red_cube, blue_cylinder] but only when each was detected, so a
        # frame with only one detection has len(poses)==1. We can't tell
        # which is which from PoseArray alone -> nearest-neighbour match
        # against the two ground-truth points (a standard greedy 2-class
        # assignment; safe because the objects are >10 cm apart, far
        # larger than any plausible noise-induced error).
        det = [np.array([p.position.x, p.position.y, p.position.z])
               for p in msg.poses]

        cube_pos, cyl_pos = self._assign(det)
        stamp = msg.header.stamp

        cube_err = cube_pos - self.gt_cube if cube_pos is not None else None
        cyl_err  = cyl_pos  - self.gt_cyl  if cyl_pos  is not None else None

        if cube_err is not None:
            self.cube_errors.append(cube_err)
            self.frames_with_cube_det += 1
        if cyl_err is not None:
            self.cyl_errors.append(cyl_err)
            self.frames_with_cyl_det += 1

        row = [self.run_tag, stamp.sec, stamp.nanosec]
        row += list(cube_err) + [float(np.linalg.norm(cube_err))] \
               if cube_err is not None else ['', '', '', '']
        row += list(cyl_err) + [float(np.linalg.norm(cyl_err))] \
               if cyl_err is not None else ['', '', '', '']
        self.csv.writerow(row)

    def _assign(self, detections):
        """Return (cube_pos, cyl_pos), each None if unmatched.
        Greedy nearest-neighbour against the two GT points."""
        cube_pos = cyl_pos = None
        if not detections:
            return None, None
        if len(detections) == 1:
            d = detections[0]
            if np.linalg.norm(d - self.gt_cube) < np.linalg.norm(d - self.gt_cyl):
                cube_pos = d
            else:
                cyl_pos = d
            return cube_pos, cyl_pos
        # 2 detections, 2 GT: pick the assignment with smaller total error.
        d0, d1 = detections[0], detections[1]
        cost_a = np.linalg.norm(d0 - self.gt_cube) + np.linalg.norm(d1 - self.gt_cyl)
        cost_b = np.linalg.norm(d0 - self.gt_cyl)  + np.linalg.norm(d1 - self.gt_cube)
        if cost_a <= cost_b:
            return d0, d1
        return d1, d0

    # ── Final summary on shutdown ─────────────────────────────────────
    def summarise(self):
        def stats(errs, label):
            if not errs:
                self.get_logger().info(f'{label}: no detections')
                return
            E = np.stack(errs)                        # (N, 3) in metres
            mae  = np.mean(np.abs(E), axis=0) * 1000  # mm per axis
            rmse = np.sqrt(np.mean(E ** 2, axis=0)) * 1000
            rmse_3d = math.sqrt(np.mean(np.sum(E ** 2, axis=1))) * 1000
            self.get_logger().info(
                f'{label} | N={len(errs)} | '
                f'MAE(mm)=[{mae[0]:.2f}, {mae[1]:.2f}, {mae[2]:.2f}] | '
                f'RMSE(mm)=[{rmse[0]:.2f}, {rmse[1]:.2f}, {rmse[2]:.2f}] | '
                f'RMSE_3D={rmse_3d:.2f}')

        stats(self.cube_errors, 'red_cube ')
        stats(self.cyl_errors,  'blue_cyl ')
        if self.frames_with_truth > 0:
            cube_rate = self.frames_with_cube_det / self.frames_with_truth
            cyl_rate  = self.frames_with_cyl_det  / self.frames_with_truth
            self.get_logger().info(
                f'detection rate: cube={cube_rate:.3f} '
                f'cyl={cyl_rate:.3f} (over {self.frames_with_truth} frames)')


def main():
    rclpy.init()
    node = PoseEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summarise()
        try:
            node.csv_file.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
