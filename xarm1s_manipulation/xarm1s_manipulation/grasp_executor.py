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

Grasp realisation is a fixed-joint attach (proposal Sec 4.4 asks only for a
"stable and repeatable predefined grasp"), not Gazebo friction contact -- this
isolates the perception signal from Gazebo Classic's unreliable parallel-jaw
contact solver. On gripper-close the object is carried rigidly by the
``xarm1s_link_attacher`` world plugin (a zero-range revolute joint between the
gripper link and the object, created on ``/grasp/attach`` and removed on
``/grasp/detach``); on release it is settled at the drop point via the
gazebo_ros_state ``/set_entity_state`` service. Success and placement error are
computed analytically from the commanded-vs-true offset at grasp time and do not
depend on the physical carry.

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
#                  (it looks enveloped) rather than hovering above it. The hold is
#                  a soft-attach, so the depth only has to look right.
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
    #   place        : this object's own spot ON the green drop_plate (centre
    #                  PLATE_XY). Four distinct spots in a diamond so the objects
    #                  CLUSTER on the plate instead of piling onto one point
    #                  (stacking them at one spot makes the soft-attach teleport a
    #                  new object INTO the previous one -> physics explosion). All
    #                  four spots verified reachable + same-branch (no flip).
    #   half_h       : object half-height -> resting z on the table (0.75+half_h)
    #                  when pinned on the plate after placement.
    "coke_can":  dict(model="coke_can",  nominal=(0.14, 0.11), grasp_z=0.755,
                      capture_tol=0.020, bias=(0.0, 0.0), minor_axis=(1.0, 0.0),
                      grasp_width=0.052, place=(0.27, 0.07), half_h=0.026),
    "banana":    dict(model="banana",    nominal=(0.14, -0.11), grasp_z=0.755,
                      capture_tol=0.022, bias=(-0.011, -0.016), minor_axis=(0.0, 1.0),
                      grasp_width=0.035, place=(0.27, -0.07), half_h=0.015),
    "golf_ball": dict(model="golf_ball", nominal=(-0.14, 0.11), grasp_z=0.755,
                      capture_tol=0.020, bias=(0.0, 0.0), minor_axis=None,
                      grasp_width=0.043, place=(0.19, 0.05), half_h=0.0214),
    "blue_box":  dict(model="blue_box",  nominal=(-0.14, -0.11), grasp_z=0.755,
                      capture_tol=0.022, bias=(0.0, 0.0), minor_axis=(0.0, 1.0),
                      grasp_width=0.040, place=(0.19, -0.05), half_h=0.015),
}

GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
# Jaw kinematics (fingers rotate about Y): the gap between the finger inner-face
# fingertips is gap(lj) = 2*PX*cos(lj) - 2*PZ*sin(lj) = R*cos(lj + delta), where
# (PX, PZ) is the inner-face fingertip in the gripper frame (measured from the
# meshes). This lets us close to a TARGET WIDTH (grip-to-contact) instead of
# slamming to the crossed "closed" pose.
GRIPPER_PX, GRIPPER_PZ = 0.0015, 0.059
JAW_OPEN = -0.65               # left-finger angle for ~72 mm (widened URDF limit)
GRIP_SQUEEZE = 0.010           # (legacy) close this much tighter than object width
# The object is held by the fixed joint (xarm1s_grasp_plugin), not the fingers,
# so the close is purely visual: a uniform GENTLE framing that closes the jaws
# noticeably from open WITHOUT jamming into any object's collision (jamming a
# rigidly-joined object makes the solver whip it out -- the banana symptom).
GRIP_FRAME = 0.042
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
# joined (the table-contact force then fights the joint and explodes -- verified
# at 0.755). It is released here and set onto the spot by place_at_rest.
PLATE_XY = (0.23, 0.0)
PLACE_Z = 0.79


def _duration(t: float) -> Duration:
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


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
        # fixed-joint grasp: command the xarm1s_grasp_plugin to rigidly attach /
        # release the object (smooth carry, no teleport jitter).
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

    # ----- fixed-joint grasp (xarm1s_grasp_plugin) -----
    def _grasp_attach(self, model: str) -> None:
        """Command the plugin to form a rigid fixed joint gripper<->object."""
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
        if label == "coke_can":          # lying cylinder: roll 90 deg (axis = Y)
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
                # Close to a grip level (jaws stop near the object's width, not
                # fully shut). Gazebo Classic parallel-jaw contact is too
                # unreliable to hold these objects (verified: the fingers close
                # through them), so the HOLD is a soft-attach gated on the
                # geometric capture test -- a real miss -> no attach -> left behind.
                self.set_gripper(grip_positions(GRIP_FRAME))
                if captured:
                    self._grasp_attach(model)    # rigid fixed joint -> no jitter
                    time.sleep(0.4)              # let the plugin form it before lifting
                # smooth lift + carry + place as one motion (stops only at place)
                self.move_through([cfg_grasp, cfg_pre, cfg_place_pre, cfg_place])
                # Open the (cosmetic) jaws so the fingers are clear before the
                # object is set down -- otherwise place_at_rest teleports it into
                # the still-closed finger collisions and it is flung off the table
                # (verified). The object stays put meanwhile (held by the joint,
                # not the fingers).
                self.set_gripper(GRIP_OPEN)
                if captured:
                    self._grasp_detach(model)    # release the grasp joint
                    self._place_at_rest(label)   # set it down on its plate spot
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
            "pick_success": int(bool(captured)),
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
            k = sum(r["pick_success"] for r in recs)
            p = k / n
            ci = 1.96 * math.sqrt(max(p * (1 - p), 0) / n) if n else 0.0
            errs = [r["place_err_mm"] for r in recs
                    if r["pick_success"] and r["place_err_mm"] != ""]
            if errs:
                m = sum(errs) / len(errs)
                sem = (np.std(errs, ddof=1) / math.sqrt(len(errs))) if len(errs) > 1 else 0.0
                place = f"place_err {m:5.1f} +-{1.96*sem:4.1f} mm"
            else:
                place = "place_err   --"
            print(f"  {label:<10} pick {k:3d}/{n:<3d} = {p*100:5.1f}% "
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
