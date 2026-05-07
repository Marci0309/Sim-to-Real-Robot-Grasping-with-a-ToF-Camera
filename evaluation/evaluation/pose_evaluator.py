#!/usr/bin/env python3
"""
Pose Evaluator — compares /detected_objects against Gazebo ground truth.

Subscribes:
    /detected_objects  (geometry_msgs/PoseArray, frame_id='world')
        Order is fixed: OBJECT_LABELS (per pose_estimator).
    /model_states      (gazebo_msgs/ModelStates)

Logs per-frame Euclidean position error to a CSV.

Ground truth: depth-based centroid in pose_estimator estimates the *visible
top surface* of each object, so we compare against (model centre +
half_height) per Hodan et al., "BOP: Benchmark for 6D Object Pose
Estimation", ECCV 2018, §4.

Author: Marcell Mor Szokedencsi (s5173191), University of Groningen
"""

import csv
import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from gazebo_msgs.msg import ModelStates


# Per-object metadata: model name in /model_states + half-height (m) used
# to convert Gazebo body-centre Z to the visible-top Z that pose_estimator
# returns. Keep the order in sync with pose_estimator.OBJECT_LABELS.
OBJECTS = [
    # (label, gazebo model name, half_height in metres)
    ("coke_can",  "coke_can",  0.033),    # lying on side -> top is at body centre + radius
    ("banana",    "banana",    0.020),    # box thickness 0.04
    ("golf_ball", "golf_ball", 0.0214),   # sphere radius
    ("blue_box",  "blue_box",  0.015),    # box thickness 0.030
]


class PoseEvaluator(Node):
    def __init__(self):
        super().__init__('pose_evaluator')

        self.declare_parameter('detected_topic', '/detected_objects')
        self.declare_parameter('truth_topic',    '/model_states')
        default_csv = os.path.expanduser('~/thesis_ws/src/evaluation/results/run.csv')
        self.declare_parameter('output_csv', default_csv)
        self.declare_parameter('run_tag', 'default')

        self.detected_topic = self.get_parameter('detected_topic').value
        self.truth_topic    = self.get_parameter('truth_topic').value
        self.csv_path       = self.get_parameter('output_csv').value
        self.run_tag        = self.get_parameter('run_tag').value

        # Per-object state: ground-truth top-surface position, and lists of
        # frame-by-frame error vectors used for end-of-run statistics.
        self.gt = {label: None for label, _, _ in OBJECTS}
        self.errors = {label: [] for label, _, _ in OBJECTS}
        self.frames_with_truth = 0
        self.frames_with_det = {label: 0 for label, _, _ in OBJECTS}

        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        write_header = not os.path.exists(self.csv_path)
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.csv = csv.writer(self.csv_file)
        if write_header:
            header = ['tag', 't_sec', 't_nsec']
            for label, _, _ in OBJECTS:
                header += [f'{label}_dx', f'{label}_dy',
                           f'{label}_dz', f'{label}_norm']
            self.csv.writerow(header)

        self.create_subscription(PoseArray, self.detected_topic,
                                 self._on_detected, 10)
        self.create_subscription(ModelStates, self.truth_topic,
                                 self._on_truth, 10)

        self.get_logger().info(
            f'pose_evaluator started | tag="{self.run_tag}" '
            f'csv={self.csv_path}')

    def _on_truth(self, msg: ModelStates):
        for label, model_name, half_h in OBJECTS:
            try:
                i = msg.name.index(model_name)
            except ValueError:
                continue
            p = msg.pose[i].position
            self.gt[label] = np.array([p.x, p.y, p.z + half_h])

    def _on_detected(self, msg: PoseArray):
        if any(g is None for g in self.gt.values()):
            return
        self.frames_with_truth += 1

        # pose_estimator publishes only objects it detected, in the
        # OBJECT_LABELS order but with gaps. We recover the assignment by
        # nearest neighbour against ground truth — safe because objects
        # are >10cm apart, larger than any plausible noise-induced error.
        det = [np.array([p.position.x, p.position.y, p.position.z])
               for p in msg.poses]
        assignment = self._assign(det)

        row = [self.run_tag, msg.header.stamp.sec, msg.header.stamp.nanosec]
        for label, _, _ in OBJECTS:
            err = assignment.get(label)
            if err is None:
                row += ['', '', '', '']
            else:
                self.errors[label].append(err)
                self.frames_with_det[label] += 1
                row += [err[0], err[1], err[2], float(np.linalg.norm(err))]
        self.csv.writerow(row)

    def _assign(self, detections):
        """Greedy nearest-neighbour: each detection -> closest unmatched GT.
        Returns dict {label: error_vec or None}."""
        result = {label: None for label, _, _ in OBJECTS}
        if not detections:
            return result
        unmatched = list(result.keys())
        for d in detections:
            if not unmatched:
                break
            best = min(unmatched,
                       key=lambda lbl: np.linalg.norm(d - self.gt[lbl]))
            result[best] = d - self.gt[best]
            unmatched.remove(best)
        return result

    def summarise(self):
        for label, _, _ in OBJECTS:
            errs = self.errors[label]
            if not errs:
                self.get_logger().info(f'{label}: no detections')
                continue
            E = np.stack(errs)
            mae  = np.mean(np.abs(E), axis=0) * 1000
            rmse = np.sqrt(np.mean(E ** 2, axis=0)) * 1000
            rmse_3d = math.sqrt(np.mean(np.sum(E ** 2, axis=1))) * 1000
            self.get_logger().info(
                f'{label:>10} | N={len(errs)} | '
                f'MAE(mm)=[{mae[0]:.2f}, {mae[1]:.2f}, {mae[2]:.2f}] | '
                f'RMSE(mm)=[{rmse[0]:.2f}, {rmse[1]:.2f}, {rmse[2]:.2f}] | '
                f'RMSE_3D={rmse_3d:.2f}')

        if self.frames_with_truth > 0:
            rates = ' '.join(
                f'{label}={self.frames_with_det[label] / self.frames_with_truth:.3f}'
                for label, _, _ in OBJECTS)
            self.get_logger().info(
                f'detection rate ({self.frames_with_truth} frames): {rates}')


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
