#!/usr/bin/env python3
"""Closed-loop pick-and-place for the xArm 1S sim (Phase 4A).

Drives the arm through a HOME -> pre-grasp -> grasp -> lift -> place -> home
sequence for each target object, using the analytical IK in ``arm_ik`` and the
existing ``arm_controller`` / ``gripper_controller``. Two target sources:

* ``ground_truth`` (bring-up): grasp the object's true ``/model_states`` pose.
  Perception error is zero, so every grasp succeeds -- use this to verify the
  arm physically reaches each object before trusting the perception path.
* ``perception`` (the experiment): grasp the pose reported by the perception
  node on ``/detected_objects``. A per-object *capture test* decides success:
  the grasp succeeds iff the commanded grasp point is within ``capture_tol`` of
  the object's true XY (the gripper jaws straddle the object). This is the
  perception -> manipulation coupling the thesis Sub-Question 3 asks about.

Grasp realisation is a CONTACT-TRIGGERED fixed-joint attach (proposal Sec 4.4
asks only for a "stable and repeatable predefined grasp"). The effort-controlled
fingers close ONTO the object and the ``xarm1s_link_attacher`` world plugin
welds it to the gripper only once it detects two-finger contact (following
GazeboGraspFix and Gazebo's own physics::Gripper) -- so the gripper must
genuinely grip the object before it is carried (no grip-on-air, no
teleport-attach). The weld is a kinematic grasp constraint standing in for
Gazebo Classic's unreliable sustained parallel-jaw friction on small/round
objects; it is armed via ``/grasp/attach`` and removed on ``/grasp/detach``, and
on release the object drops the last cm onto the plate and settles under
friction. Success and placement error are computed ANALYTICALLY from the
commanded-vs-true offset at grasp time and do NOT depend on the physical carry,
so the fast statistical sweep (``execute_motion:=false``) is unaffected.

Results are appended to ``manip_results.csv`` (tagged by ``run_tag`` like the
pose-evaluator's ``run.csv``) and summarised on Ctrl-C.
"""
import csv
import math
import os
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from xarm1s_manipulation import arm_ik as ik

# Per-object grasp data. Detections are matched to objects by nearest nominal XY
# (see perceived_xy), so the publish order of /detected_objects does not matter.
#   nominal      : world XY used to pick the right detection / yaw branch.
#                  Objects moved in to r~0.18 (2026-05-30) so the arm can take a
#                  near-vertical (~85 deg) top-down approach instead of a shallow
#                  ~44 deg one -- the gripper now descends ONTO the object.
#   grasp_z      : fingertip target = 0.755 (~5 mm above the table). The ~59 mm
#                  prongs hang BELOW the gripper body, so driving the fingertip
#                  near the table makes the prongs descend AROUND the object body
#                  (it looks enveloped) rather than hovering above it -- so the
#                  closing jaws straddle the object and make the two-finger
#                  contact the grasp plugin needs in order to weld.
#   capture_tol  : max horizontal commanded-vs-true error for a successful grasp
#                  (~ gripper jaw half-opening minus the object half-width)
#   bias         : sim-only mesh-bias correction (mean detected-truth offset
#                  measured from the sim_clean rows of run.csv); subtracted from
#                  the perceived target so manipulation isolates noise, not the
#                  constant Sketchfab-mesh artefact. Zero for the primitives.
#   minor_axis   : horizontal WORLD direction the jaws should close ACROSS (the
#                  object's narrow/graspable axis); the wrist roll J2 is computed
#                  to align the gripper closing-axis with this (research: close
#                  across the minor principal axis). None = rotationally
#                  symmetric (sphere), any roll. coke lies with its axis along
#                  world Y (pose roll=pi/2) so jaws close across X; banana long
#                  axis ~X and blue_box narrow side ~Y so both close across Y.
OBJECTS: Dict[str, dict] = {
    #   grasp_width  : object width across the minor axis = how far to close the
    #                  jaws (Phase A uses this known value; Phase B will get it
    #                  from the perceived blob's minor-axis extent).
    #   place        : this object's own spot on the green drop_plate (centre
    #                  PLATE_XY). Four distinct spots in a diamond so the objects
    #                  cluster on the plate rather than piling onto one point. All
    #                  four spots verified reachable + same-branch (no flip).
    #   half_h       : object half-height -> resting z on the table (0.75+half_h)
    #                  when pinned on the plate after placement.
    #   place_settle : True = set the object down on its spot at release instead
    #                  of a free drop. Used for the 52mm coke: it fills the 55mm
    #                  jaw so the fingertip pads cannot fully clear it, and a free
    #                  release lets the residual pad contact spring it off; the
    #                  pickup is still a real contact-triggered grasp.
    "coke_can":  dict(model="coke_can",  nominal=(0.14, 0.11), grasp_z=0.755,
                      capture_tol=0.0015, bias=(0.0, 0.0), minor_axis=(1.0, 0.0),
                      grasp_width=0.052, place=(0.27, 0.07), half_h=0.026,
                      place_settle=True),  # (55-52)/2, at the jaw size limit
    "banana":    dict(model="banana",    nominal=(0.14, -0.11), grasp_z=0.755,
                      # bias now 0: the banana VISUAL was re-centered onto its link
                      # origin in model.sdf (the footprint-centroid offset is removed
                      # at the source), so the old fixed per-pose bias hack -- which
                      # only worked at one yaw -- is obsolete.
                      capture_tol=0.010, bias=(0.0, 0.0), minor_axis=(0.0, 1.0),
                      grasp_width=0.035, place=(0.27, -0.07), half_h=0.015),  # (55-35)/2
    "golf_ball": dict(model="golf_ball", nominal=(-0.14, 0.11), grasp_z=0.755,
                      capture_tol=0.006, bias=(0.0, 0.0), minor_axis=None,
                      grasp_width=0.043, place=(0.19, 0.05), half_h=0.0214),  # (55-43)/2
    "blue_box":  dict(model="blue_box",  nominal=(-0.14, -0.11), grasp_z=0.755,
                      capture_tol=0.0075, bias=(0.0, 0.0), minor_axis=(0.0, 1.0),
                      grasp_width=0.040, place=(0.19, -0.05), half_h=0.015),  # (55-40)/2
}

GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
# Jaw kinematics (fingers rotate about Y): the gap between the finger inner-face
# fingertips is gap(lj) = 2*PX*cos(lj) - 2*PZ*sin(lj) = R*cos(lj + delta), where
# (PX, PZ) is the inner-face fingertip in the gripper frame (measured from the
# meshes). This lets us close to a TARGET WIDTH (grip-to-contact) instead of
# slamming to the crossed "closed" pose.
GRIPPER_PX, GRIPPER_PZ = 0.0015, 0.059
JAW_OPEN = -0.459              # left-finger angle for 55 mm open (matches the real gripper)
# The fingers are EFFORT-controlled, so the jaws are commanded to close
# GRIP_SQUEEZE below the object's width: the finger then presses the object with
# a bounded, soft force and stalls at the surface (it cannot penetrate + fling
# it, the position-control failure). The grasp plugin sees the resulting
# two-finger contact and welds; the squeeze keeps that contact alive during the
# carry. 6 mm keeps the position error -- and hence the effort -- gentle.
GRIP_SQUEEZE = 0.006           # close this much tighter than the object width
GRIP_OPEN = [JAW_OPEN, -JAW_OPEN]


def _finger_angle_for_width(width: float) -> float:
    """Left-finger joint angle whose jaw gap equals `width` m (right = -this)."""
    R = 2.0 * math.hypot(GRIPPER_PX, GRIPPER_PZ)
    delta = math.atan2(GRIPPER_PZ, GRIPPER_PX)
    return -delta + math.acos(max(-1.0, min(1.0, width / R)))


def grip_positions(width: float) -> List[float]:
    """Gripper [left, right] joint positions that close to a jaw gap of `width`."""
    lj = _finger_angle_for_width(width)
    return [lj, -lj]


HOME_CFG = {n: 0.0 for n in ik.ARM_JOINT_ORDER}   # arm straight up, clear

PRE_DIST = 0.08      # pre-grasp / retreat offset back along the approach axis
# Objects are placed on the green drop_plate (centre PLATE_XY; see
# tabletop.world). Each object goes to its own spot on the plate (OBJECTS
# 'place') so they cluster rather than pile. Release height PLACE_Z is above the
# table: the object must NOT be carried down onto the table while still rigidly
# joined (the table-contact force then fights the joint and explodes). It is
# released here; the coke is set down on its spot by place_at_rest.
PLATE_XY = (0.23, 0.0)
PLACE_Z = 0.79


def _duration(t: float) -> Duration:
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def _rpy_to_quat(r: float, p: float, y: float):
    """(roll, pitch, yaw) -> (x, y, z, w) quaternion (matches reset_objects.py)."""
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class GraspExecutor(Node):
    def __init__(self) -> None:
        super().__init__("grasp_executor")
        cb = ReentrantCallbackGroup()

        # --- parameters ---
        self.run_tag = self.declare_parameter("run_tag", "sim_clean").value
        self.target_source = self.declare_parameter(
            "target_source", "ground_truth").value
        self.objects = list(self.declare_parameter(
            "objects", list(OBJECTS.keys())).value)
        self.n_trials = int(self.declare_parameter("n_trials", 1).value)
        # execute_motion: True = drive the arm + grasp the object (demo).
        # False = sample perception and compute the (analytic) capture/placement
        # metric without moving the arm -- fast, for N-trial statistics.
        self.execute_motion = bool(
            self.declare_parameter("execute_motion", True).value)
        self.bias_correction = bool(
            self.declare_parameter("bias_correction", True).value)
        self.placement_xy = list(self.declare_parameter(
            "placement_xy", [0.0, 0.22]).value)
        self.move_time = float(self.declare_parameter("move_time", 3.0).value)
        self.settle_time = float(self.declare_parameter("settle_time", 0.6).value)
        # settle_place: after the gentle release, also pin the object exactly on
        # its plate spot with a zero-velocity set_entity_state (the old
        # behaviour). Default False = let it drop the last cm and settle under
        # friction (a real release, no teleport).
        self.settle_place = bool(
            self.declare_parameter("settle_place", False).value)
        # vary_placement: success-rate sweep mode. For each object (one on the
        # table at a time), teleport it to n_trials DIFFERENT reachable poses
        # (varied position + orientation) and record the analytic capture test at
        # each -> full reachable-workspace coverage (supervisor's request). No arm
        # motion. placement_seed makes the sampled poses reproducible.
        self.vary_placement = bool(
            self.declare_parameter("vary_placement", False).value)
        self.placement_seed = int(
            self.declare_parameter("placement_seed", 0).value)
        self.robot_model = self.declare_parameter("robot_model", "xarm1s").value
        self.csv_path = os.path.expanduser(self.declare_parameter(
            "csv_path",
            "~/thesis_ws/src/evaluation/results/manip_results.csv").value)

        # --- state caches ---
        self._lock = threading.Lock()
        self.latest_detections: Optional[PoseArray] = None
        self.latest_grasps: Optional[PoseArray] = None
        self.latest_models: Optional[ModelStates] = None

        self.create_subscription(PoseArray, "/detected_objects",
                                 self._on_det, 10, callback_group=cb)
        self.create_subscription(PoseArray, "/grasp_points",
                                 self._on_grasps, 10, callback_group=cb)
        self.create_subscription(ModelStates, "/model_states",
                                 self._on_models, 10, callback_group=cb)

        self.grip_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        # contact-triggered grasp: /grasp/attach ARMS the xarm1s_grasp_plugin to
        # weld the object once both fingers contact it; /grasp/detach releases.
        self.attach_pub = self.create_publisher(String, "/grasp/attach", 10)
        self.detach_pub = self.create_publisher(String, "/grasp/detach", 10)
        self.arm_action = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory", callback_group=cb)
        self.set_state_service = self.declare_parameter(
            "set_state_service", "/set_entity_state").value
        self.set_state = self.create_client(
            SetEntityState, self.set_state_service, callback_group=cb)

        # precompute per-object approach tilt from the nominal grasp point
        self.tilt: Dict[str, float] = {}
        for name, d in OBJECTS.items():
            tgt = (d["nominal"][0], d["nominal"][1], d["grasp_z"])
            res = ik.feasible_approach(tgt, min_clearance=0.75)
            self.tilt[name] = res[0] if res else math.radians(35.0)

        self.records: List[dict] = []
        self.get_logger().info(
            f"grasp_executor: tag={self.run_tag} source={self.target_source} "
            f"n_trials={self.n_trials} bias_corr={self.bias_correction} "
            f"objects={self.objects}")

    # ----- subscription callbacks -----
    def _on_det(self, msg: PoseArray) -> None:
        with self._lock:
            self.latest_detections = msg

    def _on_grasps(self, msg: PoseArray) -> None:
        with self._lock:
            self.latest_grasps = msg

    def _on_models(self, msg: ModelStates) -> None:
        with self._lock:
            self.latest_models = msg

    # ----- blocking helpers (executor spins in a background thread) -----
    @staticmethod
    def _wait(future, timeout: float):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return None
        return future.result()

    def move_arm(self, cfg: Dict[str, float], duration: Optional[float] = None) -> bool:
        duration = self.move_time if duration is None else duration
        if not self.arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("arm action server unavailable")
            return False
        traj = JointTrajectory()
        traj.joint_names = list(ik.ARM_JOINT_ORDER)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in ik.to_arm_array(cfg)]
        pt.time_from_start = _duration(duration)
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        gh = self._wait(self.arm_action.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            self.get_logger().error("arm goal rejected / timed out")
            return False
        self._wait(gh.get_result_async(), duration + 10.0)
        time.sleep(self.settle_time)
        return True

    def move_through(self, cfgs: List[Dict[str, float]],
                     seg_time: Optional[float] = None) -> bool:
        """Drive the arm through a sequence of configs as ONE trajectory, with
        finite-difference velocities at the interior waypoints so the motion
        flows smoothly through them instead of stopping + settling at each.
        cfgs[0] must be the CURRENT config; the arm starts and ends at rest."""
        seg = self.move_time if seg_time is None else seg_time
        if len(cfgs) < 2:
            return self.move_arm(cfgs[-1]) if cfgs else True
        if not self.arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("arm action server unavailable")
            return False
        arrs = [np.array(ik.to_arm_array(c), dtype=float) for c in cfgs]
        n = len(arrs)
        traj = JointTrajectory()
        traj.joint_names = list(ik.ARM_JOINT_ORDER)
        for i, a in enumerate(arrs):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in a]
            if 0 < i < n - 1:                       # interior: pass through
                vel = (arrs[i + 1] - arrs[i - 1]) / (2.0 * seg)
            else:                                   # ends: start/finish at rest
                vel = np.zeros_like(a)
            pt.velocities = [float(v) for v in vel]
            pt.time_from_start = _duration(0.001 + i * seg)
            traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        gh = self._wait(self.arm_action.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            self.get_logger().error("arm goal rejected / timed out")
            return False
        self._wait(gh.get_result_async(), seg * n + 10.0)
        time.sleep(self.settle_time)
        return True

    def set_gripper(self, positions: List[float], duration: float = 0.6) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(GRIPPER_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = _duration(duration)
        traj.points = [pt]
        self.grip_pub.publish(traj)
        time.sleep(0.1)
        self.grip_pub.publish(traj)
        time.sleep(duration + 0.2)

    # ----- world queries -----
    def truth_xyz(self, model: str) -> Optional[np.ndarray]:
        with self._lock:
            ms = self.latest_models
        if ms is None or model not in ms.name:
            return None
        p = ms.pose[ms.name.index(model)].position
        return np.array([p.x, p.y, p.z])

    def perceived_xy(self, label: str) -> Optional[np.ndarray]:
        """Detection nearest the object's nominal XY (robust to array order)."""
        with self._lock:
            det = self.latest_detections
        if det is None or not det.poses:
            return None
        nominal = np.array(OBJECTS[label]["nominal"])
        best, best_d = None, 0.12   # reject matches > 12 cm from nominal
        for ps in det.poses:
            xy = np.array([ps.position.x, ps.position.y])
            d = np.linalg.norm(xy - nominal)
            if d < best_d:
                best, best_d = xy, d
        return best

    def _perceived_nearest(self, truth_xy: np.ndarray) -> Optional[np.ndarray]:
        """Detection nearest the CURRENT true XY -- used by vary_placement, where
        the object is teleported around so the fixed-nominal matcher no longer
        applies. Only the test object is on the table, so this is effectively the
        lone detection; a gross miss (>15 cm) returns None (counts as a failure)."""
        with self._lock:
            det = self.latest_detections
        if det is None or not det.poses:
            return None
        best, best_d = None, 0.15
        for ps in det.poses:
            xy = np.array([ps.position.x, ps.position.y])
            dd = float(np.linalg.norm(xy - truth_xy))
            if dd < best_d:
                best, best_d = xy, dd
        return best

    def grasp_point_xy(self, label: str) -> Optional[np.ndarray]:
        """On-body grasp point (from /grasp_points) nearest the object's nominal
        XY. This is where the gripper should actually close -- for the curved
        banana it sits on the body, unlike the centroid. None if unavailable."""
        with self._lock:
            gp = self.latest_grasps
        if gp is None or not gp.poses:
            return None
        nominal = np.array(OBJECTS[label]["nominal"])
        best, best_d = None, 0.12
        for ps in gp.poses:
            xy = np.array([ps.position.x, ps.position.y])
            d = np.linalg.norm(xy - nominal)
            if d < best_d:
                best, best_d = xy, d
        return best

    # ----- contact-triggered grasp (xarm1s_grasp_plugin) -----
    def _grasp_attach(self, model: str) -> None:
        """Arm the plugin to weld <model> to the gripper on two-finger contact."""
        msg = String(data=model)
        self.attach_pub.publish(msg)
        time.sleep(0.05)
        self.attach_pub.publish(msg)   # publish twice (robust delivery)

    def _grasp_detach(self, model: str) -> None:
        """Command the plugin to release the object from the gripper."""
        msg = String(data=model)
        self.detach_pub.publish(msg)
        time.sleep(0.05)
        self.detach_pub.publish(msg)

    def _place_at_rest(self, label: str) -> None:
        """Set the released object down on its plate spot via a few
        set_entity_state teleports (zero velocity). This is a plain pose set --
        NO joint -- so it does not perturb the physics solver; it just lands the
        object cleanly and absorbs any release motion (e.g. the banana mesh)."""
        d = OBJECTS[label]
        model = d["model"]
        pose = Pose()
        pose.position.x = float(d["place"][0])
        pose.position.y = float(d["place"][1])
        pose.position.z = 0.75 + d["half_h"]
        if label == "coke_can":          # lying cylinder (low profile): roll 90 deg
            pose.orientation.x = 0.70710678
            pose.orientation.w = 0.70710678
        else:
            pose.orientation.w = 1.0
        for _ in range(6):               # ~0.3 s of zero-velocity placement
            if self.set_state.service_is_ready():
                req = SetEntityState.Request()
                req.state.name = model
                req.state.pose = pose
                req.state.reference_frame = "world"
                self.set_state.call_async(req)
            time.sleep(0.05)

    # ----- varied-placement success-rate sweep -----
    def _set_object_pose(self, model: str, x, y, z, roll, pitch, yaw) -> None:
        """Teleport an object to a pose at zero velocity via set_entity_state."""
        qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(x), float(y), float(z))
        pose.orientation.x, pose.orientation.y = float(qx), float(qy)
        pose.orientation.z, pose.orientation.w = float(qz), float(qw)
        for _ in range(4):
            if self.set_state.service_is_ready():
                req = SetEntityState.Request()
                req.state.name = model
                req.state.pose = pose
                req.state.reference_frame = "world"
                self.set_state.call_async(req)
            time.sleep(0.05)

    def _park_others(self, keep_label: str) -> None:
        """Teleport every object except keep_label far off the table (out of the
        overhead-camera FOV) so only the test object is detected."""
        i = 0
        for lbl, d in OBJECTS.items():
            if lbl == keep_label:
                continue
            self._set_object_pose(d["model"], 2.0 + 0.3 * i, 2.0,
                                  0.75 + d["half_h"], 0.0, 0.0, 0.0)
            i += 1
        time.sleep(0.3)

    def _reachable_placements(self, label: str, n: int, seed: int):
        """n reproducible (x,y,z,roll,pitch,yaw) poses inside the REACHABLE
        workspace (r in [0.12, 0.30], each validated by ik.feasible_approach),
        with varied yaw. r>=0.12 also keeps objects clear of the home arm column
        at the centre. coke lies on its side (roll=pi/2); others upright/flat."""
        d = OBJECTS[label]
        z = 0.75 + d["half_h"]
        roll = math.pi / 2.0 if label == "coke_can" else 0.0
        rng = np.random.RandomState(
            (seed * 131 + sum(ord(c) for c in label)) % (2 ** 31 - 1))
        out, tries = [], 0
        while len(out) < n and tries < 5000:
            tries += 1
            r = rng.uniform(0.12, 0.30)
            th = rng.uniform(-math.pi, math.pi)
            x, y = r * math.cos(th), r * math.sin(th)
            if ik.feasible_approach((x, y, d["grasp_z"]), min_clearance=0.75) is None:
                continue                       # unreachable -> skip this sample
            yaw = rng.uniform(-math.pi, math.pi)
            out.append((x, y, z, roll, 0.0, yaw))
        return out

    def run_vary_placement(self) -> None:
        """Success-rate sweep: each object, one at a time, teleported to n_trials
        DIFFERENT reachable poses; record the analytic capture test at each (raw
        error when bias_correction is off). Gives full reachable-workspace coverage."""
        if not self.set_state.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "set_entity_state unavailable; cannot vary placements")
            return
        for label in self.objects:
            if label not in OBJECTS:
                self.get_logger().warn(f"unknown object '{label}', skipping")
                continue
            d = OBJECTS[label]
            model = d["model"]
            self._park_others(label)
            samples = self._reachable_placements(
                label, self.n_trials, self.placement_seed)
            self.get_logger().info(
                f"--- {label}: {len(samples)} reachable placements "
                f"(tag={self.run_tag}, bias_corr={self.bias_correction}) ---")
            for trial, pose in enumerate(samples):
                self._set_object_pose(model, *pose)
                time.sleep(0.7)               # settle at the new pose
                time.sleep(0.3)               # let a fresh /detected_objects arrive
                truth = self.truth_xyz(model)
                if truth is None:
                    continue
                xy = self._perceived_nearest(truth[:2])
                if xy is not None and self.bias_correction:
                    xy = xy - np.array(d["bias"])
                if xy is None:                  # gross perception miss -> failure
                    captured, place_err, grasp = False, None, None
                else:
                    capture_err = float(np.linalg.norm(xy - truth[:2]))
                    captured = capture_err <= d["capture_tol"]
                    place_err = capture_err if captured else None
                    grasp = [float(xy[0]), float(xy[1]), d["grasp_z"]]
                rec = self._record(label, trial, xy, grasp, truth, captured, place_err)
                self._append_record(rec)

    # ----- geometry -----
    @staticmethod
    def _approach_dir(xy: np.ndarray, tilt: float) -> np.ndarray:
        r = float(np.linalg.norm(xy))
        if r < 1e-6:
            return np.array([0.0, 0.0, -1.0])
        u = xy / r
        return np.array([u[0] * math.cos(tilt), u[1] * math.cos(tilt),
                         -math.sin(tilt)])

    def _ik_waypoint(self, xyz: np.ndarray, tilt: float,
                     seed: Optional[Dict[str, float]] = None
                     ) -> Optional[Dict[str, float]]:
        cfg = ik.inverse_kinematics(tuple(xyz), tilt, min_clearance=0.75, seed=seed)
        if cfg is not None:
            return cfg
        res = ik.feasible_approach(tuple(xyz), min_clearance=0.75, seed=seed)
        return res[1] if res else None

    # ----- one pick-and-place trial -----
    def run_trial(self, label: str, trial: int) -> Optional[dict]:
        d = OBJECTS[label]
        model = d["model"]
        truth = self.truth_xyz(model)
        if truth is None:
            self.get_logger().error(f"no ground truth for {model}; skipping")
            return None

        # target XY from the chosen source
        if self.target_source == "perception":
            xy = self.perceived_xy(label)
            if xy is None:
                self.get_logger().warn(f"{label} trial {trial}: no detection")
                rec = self._record(label, trial, None, None, truth, False, None)
                return rec
            perceived = xy.copy()
            if self.bias_correction:
                xy = xy - np.array(d["bias"])
        else:  # ground_truth
            perceived = truth[:2].copy()
            xy = truth[:2].copy()

        tilt = self.tilt[label]
        # capture / placement metric = perception accuracy of the object CENTRE
        capture_err = float(np.linalg.norm(xy - truth[:2]))
        captured = capture_err <= d["capture_tol"]
        # the gripper closes at the perceived ON-BODY grasp point when available
        # (e.g. the banana's body, not its off-body centroid); else the target
        # XY. This only changes WHERE the jaws close, not the centroid metric.
        gxy = self.grasp_point_xy(label)
        if gxy is None:
            gxy = xy
        grasp = np.array([gxy[0], gxy[1], d["grasp_z"]])
        self.get_logger().info(
            f"--- {label} trial {trial}: capture_err={capture_err*1000:.1f}mm "
            f"tol={d['capture_tol']*1000:.0f}mm -> "
            f"{'CAPTURE' if captured else 'MISS'} | grasp@"
            f"({grasp[0]:.3f},{grasp[1]:.3f}) ---")

        if self.execute_motion:
            a_dir = self._approach_dir(grasp[:2], tilt)
            pre = grasp - a_dir * PRE_DIST
            # this object's own spot on the green drop_plate (objects cluster,
            # not pile); release above the table so it drops the last cm on.
            place_xy = np.array(d["place"])
            place = np.array([place_xy[0], place_xy[1], PLACE_Z])
            place_dir = self._approach_dir(place_xy, tilt)
            place_pre = place - place_dir * PRE_DIST

            # Solve the grasp first (natural highest-elbow pose), then SEED every
            # other waypoint from it so the whole pick-carry-place stays in ONE
            # IK branch -- this kills the mid-carry arm flip ("spin").
            cfg_grasp = self._ik_waypoint(grasp, tilt)
            cfg_pre = self._ik_waypoint(pre, tilt, seed=cfg_grasp)
            cfg_place_pre = self._ik_waypoint(place_pre, tilt, seed=cfg_grasp)
            cfg_place = self._ik_waypoint(place, tilt, seed=cfg_place_pre or cfg_grasp)
            if None in (cfg_pre, cfg_grasp, cfg_place_pre, cfg_place):
                self.get_logger().error(
                    f"{label} trial {trial}: IK failed for a waypoint")
            else:
                # Rotate the gripper (wrist roll J2) so the jaws close ACROSS
                # the object's minor axis. J2 is about the approach axis through
                # the TCP, so it doesn't move the fingertip -> positions stay
                # valid. Hold the same roll through carry/place so the gripped
                # object doesn't spin.
                minor = d.get("minor_axis")
                if minor is not None:
                    j2 = ik.wrist_roll_for_axis(cfg_grasp, minor)
                    for c in (cfg_pre, cfg_grasp, cfg_place_pre, cfg_place):
                        c["xarm_2_joint"] = j2
                self.set_gripper(GRIP_OPEN)       # open the jaws first
                # smooth approach: flow HOME -> (through pre) -> grasp in one move
                self.move_through([HOME_CFG, cfg_pre, cfg_grasp])
                # Close the EFFORT-controlled jaws ONTO the object (GRIP_SQUEEZE
                # below its width): the fingers press it with a bounded soft
                # force and stall at the surface.
                self.set_gripper(
                    grip_positions(max(0.006, d["grasp_width"] - GRIP_SQUEEZE)))
                # Arm the contact-triggered grasp AFTER the jaws have closed, so
                # the plugin welds on the genuine CLOSED-jaw two-finger contact
                # (arming during the open approach could weld on a transient
                # open-jaw touch -> grip-on-air). A perception MISS (not captured)
                # is never armed -> the object is genuinely left behind.
                if captured:
                    self._grasp_attach(model)
                    time.sleep(1.2)             # let the plugin confirm contact +
                                                # weld (and cover the fallback) before lifting
                # smooth lift + carry as one motion (stops at the place pose)
                self.move_through([cfg_grasp, cfg_pre, cfg_place_pre, cfg_place])
                # Open the jaws (effort relaxes -> fingers lift off the object)
                # and release the joint: the object drops the last cm onto its
                # plate spot and settles under the surface friction -- a real
                # release, not a teleport. A size-limit object (place_settle, the
                # 52mm coke) is additionally set down on its spot, because the
                # pads cannot clear it and a free release springs it off the table.
                use_settle = self.settle_place or d.get("place_settle", False)
                self.set_gripper(GRIP_OPEN)
                if captured:
                    self._grasp_detach(model)
                    if use_settle:
                        self._place_at_rest(label)
                    time.sleep(self.settle_time)  # let it settle before retreat
                self.move_through([cfg_place, cfg_place_pre, HOME_CFG])  # retreat

        # placement error: rigid carry preserves the grasp-time offset, so the
        # object lands at place + (truth - grasp); error vs target = capture_err.
        place_err = capture_err if captured else None
        return self._record(label, trial, perceived, grasp, truth, captured, place_err)

    def _record(self, label, trial, perceived, grasp, truth, captured, place_err) -> dict:
        rec = {
            "tag": self.run_tag,
            "object": label,
            "trial": trial,
            "perceived_x": "" if perceived is None else round(float(perceived[0]), 5),
            "perceived_y": "" if perceived is None else round(float(perceived[1]), 5),
            "target_x": "" if grasp is None else round(float(grasp[0]), 5),
            "target_y": "" if grasp is None else round(float(grasp[1]), 5),
            "true_x": round(float(truth[0]), 5),
            "true_y": round(float(truth[1]), 5),
            "captured": int(bool(captured)),
            # grasp_feasible: the PERCEPTION/geometry success -- the perceived
            # centre is within the jaw's capture tolerance (the jaws would
            # straddle the object). It is NOT a physical-grasp check (Gazebo
            # Classic can't reliably simulate parallel-jaw contact), so it is
            # named honestly rather than "pick_success".
            "grasp_feasible": int(bool(captured)),
            "place_err_mm": "" if place_err is None else round(place_err * 1000, 2),
        }
        self.records.append(rec)
        return rec

    # ----- top level -----
    def wait_for_inputs(self, timeout: float = 30.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                have_models = self.latest_models is not None
                have_det = self.latest_detections is not None
            need_det = self.target_source == "perception"
            if have_models and (have_det or not need_det):
                return True
            time.sleep(0.2)
        self.get_logger().error("timed out waiting for /model_states"
                                + (" + /detected_objects" if self.target_source
                                   == "perception" else ""))
        return False

    def run(self) -> None:
        if self.vary_placement:               # success-rate sweep over the workspace
            self.run_vary_placement()
            return
        if self.execute_motion and not self.arm_action.wait_for_server(
                timeout_sec=20.0):
            self.get_logger().error(
                "arm action /arm_controller/follow_joint_trajectory not available "
                "after 20 s. Is the sim (Terminal 1) up with controllers ACTIVE? "
                "Check with: ros2 control list_controllers")
            return
        for label in self.objects:
            if label not in OBJECTS:
                self.get_logger().warn(f"unknown object '{label}', skipping")
                continue
            for trial in range(self.n_trials):
                rec = self.run_trial(label, trial)
                if rec is not None:
                    self._append_record(rec)
                if not self.execute_motion:
                    time.sleep(0.3)   # let a fresh /detected_objects frame arrive

    def _append_record(self, rec: dict) -> None:
        """Append one row; write the header only when creating the file. Rows
        accumulate across conditions/runs (like the pose evaluator's run.csv)."""
        new_file = not os.path.exists(self.csv_path)
        d = os.path.dirname(self.csv_path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(self.csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rec.keys()))
            if new_file:
                w.writeheader()
            w.writerow(rec)

    def summarise(self) -> None:
        print("\n" + "=" * 60)
        print(f"manipulation summary  (tag={self.run_tag}, "
              f"source={self.target_source})")
        print("=" * 60)
        for label in self.objects:
            recs = [r for r in self.records if r["object"] == label]
            if not recs:
                continue
            n = len(recs)
            k = sum(r["grasp_feasible"] for r in recs)
            p = k / n
            ci = 1.96 * math.sqrt(max(p * (1 - p), 0) / n) if n else 0.0
            errs = [r["place_err_mm"] for r in recs
                    if r["grasp_feasible"] and r["place_err_mm"] != ""]
            if errs:
                m = sum(errs) / len(errs)
                sem = (np.std(errs, ddof=1) / math.sqrt(len(errs))) if len(errs) > 1 else 0.0
                place = f"place_err {m:5.1f} +-{1.96*sem:4.1f} mm"
            else:
                place = "place_err   --"
            print(f"  {label:<10} grasp-feasible {k:3d}/{n:<3d} = {p*100:5.1f}% "
                  f"+-{ci*100:4.1f}   {place}")
        print(f"\nwrote {len(self.records)} rows -> {self.csv_path}")
        print("=" * 60)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GraspExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        if node.wait_for_inputs():
            node.run()
            node.summarise()
    except KeyboardInterrupt:
        node.summarise()
    finally:
        for fn in (executor.shutdown, node.destroy_node):
            try:
                fn()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
