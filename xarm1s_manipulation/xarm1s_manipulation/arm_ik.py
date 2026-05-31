#!/usr/bin/env python3
"""Closed-form inverse kinematics for the xArm 1S (5-DoF) in the thesis sim.

The arm is a base-yaw + planar-3R + wrist-roll chain (see ``xarm.urdf``):

    xarm_base_link
      └─ J6 (axis Z, base yaw)        origin (0, 0,        0.03603)
         └─ J5 (axis X, pitch)        origin (0, -0.0022,  0.03195)
            └─ J4 (axis X, pitch)     origin (0, 0,        0.09765)
               └─ J3 (axis X, pitch)  origin (0, 0,        0.09825)
                  └─ J2 (axis Z, roll) origin (0, -0.001205, 0.05306)
                     └─ gripper_base  (fixed, coincident)
                        └─ gripper_tip (fixed, +0.09 Z)

Because the three pitch joints (J5/J4/J3) share a parallel axis, they form a
**planar 3R arm** in the vertical plane selected by the base yaw J6. The wrist
roll J2 rotates the gripper about the tool axis and does not move the fingertip
in that plane, so the fingertip position is a function of (J6, J5, J4, J3) and a
chosen *approach angle* only. This lets us solve position in closed form
(Craig, *Introduction to Robotics*, 3rd ed., ch. 4; Spong et al., *Robot
Modeling and Control*, ch. 3) rather than running a numerical IK solver — every
grasp error is then attributable to perception, not to solver convergence.

Key geometric fact for this setup: the arm is mounted *on* the table, so the
shoulder pivot sits only ~3 cm above the objects while the "hand" segment
(J3->fingertip = 0.143 m) is long relative to the upper/forearm (0.098 m each).
A purely vertical top-down grasp would need the wrist ~0.31 m from the shoulder,
but the upper+forearm span only 0.196 m -> infeasible. ``feasible_approach``
therefore searches for the *steepest reachable* (most top-down-like) approach
angle within joint limits; in practice this is a slanted approach.

Pure-numpy, no ROS / KDL dependency (PyKDL is used only as an optional FK
cross-check in the tests).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# URDF-derived constants (metres, radians). Keep in sync with xarm.urdf.
# ---------------------------------------------------------------------------
BASE_Z = 0.76            # world -> xarm_base_link fixed joint
J6_DZ = 0.03603          # base_link -> J6
J5_DY, J5_DZ = -0.0022, 0.03195   # J6 -> J5  (tiny lateral offset)
J4_DZ = 0.09765          # J5 -> J4
J3_DZ = 0.09825          # J4 -> J3
J2_DY, J2_DZ = -0.001205, 0.05306  # J3 -> J2 (tiny lateral offset)
# Grasp TCP = the MEASURED finger-contact point. The finger meshes extend to
# +0.059 m from gripper_base (mesh bounding box), which is where the closed
# fingertips actually meet -- NOT the nominal gripper_tip_link at +0.090 m.
# Driving IK to gripper_tip_link left the real fingers ~31 mm ABOVE the object
# ("closes above / grasps air"). We drive the fingertip TCP instead.
FINGERTIP_DZ = 0.059     # gripper_base -> finger-contact point (measured)

# Planar 3R link lengths (the small lateral offsets are ignored in the planar
# solver and show up only as a few-mm residual against the full FK; verified in
# the tests).
A1 = J4_DZ                    # 0.09765  J5 -> J4
A2 = J3_DZ                    # 0.09825  J4 -> J3
A3 = J2_DZ + FINGERTIP_DZ     # 0.11206  J3 -> finger-contact point (grasp TCP)
Z_PIVOT = BASE_Z + J6_DZ + J5_DZ   # 0.82798  shoulder (J5) pivot height

# Controller joint order (controllers.yaml arm_controller.joints).
ARM_JOINT_ORDER = [
    "xarm_2_joint", "xarm_3_joint", "xarm_4_joint", "xarm_5_joint", "xarm_6_joint",
]

# Revolute limits from xarm.urdf.
LIMITS: Dict[str, Tuple[float, float]] = {
    "xarm_6_joint": (-1.737238322, 1.79965407),
    "xarm_5_joint": (-1.592023304, 1.86797401),
    "xarm_4_joint": (-1.910427965, 1.910427965),
    "xarm_3_joint": (-1.740612146, 1.86797401),
    "xarm_2_joint": (-2.17494876, 1.63121157),
}


# ---------------------------------------------------------------------------
# Small homogeneous-transform helpers
# ---------------------------------------------------------------------------
def _trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def _rot_x(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]])


def _rot_z(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def _wrap(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Forward kinematics (full URDF chain, including the mm lateral offsets)
# ---------------------------------------------------------------------------
def forward_kinematics(cfg: Dict[str, float]) -> np.ndarray:
    """Return the 4x4 world pose of ``gripper_tip_link`` for a joint config."""
    t2 = cfg["xarm_2_joint"]
    t3 = cfg["xarm_3_joint"]
    t4 = cfg["xarm_4_joint"]
    t5 = cfg["xarm_5_joint"]
    t6 = cfg["xarm_6_joint"]
    T = _trans(0, 0, BASE_Z)
    T = T @ _trans(0, 0, J6_DZ) @ _rot_z(t6)
    T = T @ _trans(0, J5_DY, J5_DZ) @ _rot_x(t5)
    T = T @ _trans(0, 0, J4_DZ) @ _rot_x(t4)
    T = T @ _trans(0, 0, J3_DZ) @ _rot_x(t3)
    T = T @ _trans(0, J2_DY, J2_DZ) @ _rot_z(t2)
    T = T @ _trans(0, 0, FINGERTIP_DZ)
    return T


def fk_position(cfg: Dict[str, float]) -> np.ndarray:
    """Convenience: fingertip xyz in world."""
    return forward_kinematics(cfg)[:3, 3]


def link_world_positions(cfg: Dict[str, float]) -> Dict[str, np.ndarray]:
    """World xyz of each frame along the chain (for clearance checks / debug)."""
    t2, t3, t4 = cfg["xarm_2_joint"], cfg["xarm_3_joint"], cfg["xarm_4_joint"]
    t5, t6 = cfg["xarm_5_joint"], cfg["xarm_6_joint"]
    out: Dict[str, np.ndarray] = {}
    T = _trans(0, 0, BASE_Z); out["base"] = T[:3, 3].copy()
    T = T @ _trans(0, 0, J6_DZ) @ _rot_z(t6); out["J6"] = T[:3, 3].copy()
    T = T @ _trans(0, J5_DY, J5_DZ) @ _rot_x(t5); out["shoulder"] = T[:3, 3].copy()
    T = T @ _trans(0, 0, J4_DZ) @ _rot_x(t4); out["elbow"] = T[:3, 3].copy()
    T = T @ _trans(0, 0, J3_DZ) @ _rot_x(t3); out["wrist"] = T[:3, 3].copy()
    T = T @ _trans(0, J2_DY, J2_DZ) @ _rot_z(t2)
    T = T @ _trans(0, 0, FINGERTIP_DZ); out["tip"] = T[:3, 3].copy()
    return out


def wrist_roll_for_axis(cfg: Dict[str, float], axis_world,
                        limit_margin: float = 0.02) -> float:
    """Wrist-roll J2 that aligns the gripper's closing axis (gripper-frame X --
    where the parallel jaws open/close) ACROSS the horizontal world direction
    ``axis_world`` (the object's minor/grasp axis), so the jaws close across it.

    J2 rotates the gripper about its approach (Z) axis, which passes through the
    fingertip TCP, so changing it does NOT move the TCP -- the position solution
    stays valid. The closing axis is bidirectional, so the +-pi mirror is tried
    if the direct angle is outside the J2 limits. Returns a J2 within limits.
    """
    R = forward_kinematics(cfg)[:3, :3]
    x0, y0 = R[:, 0], R[:, 1]
    m = np.array([float(axis_world[0]), float(axis_world[1]),
                  float(axis_world[2]) if len(axis_world) > 2 else 0.0])
    n = float(np.linalg.norm(m))
    if n < 1e-9:
        return cfg["xarm_2_joint"]
    m = m / n
    dtheta = math.atan2(float(m @ y0), float(m @ x0))
    base = cfg["xarm_2_joint"]
    lo, hi = LIMITS["xarm_2_joint"]
    for cand in (_wrap(base + dtheta), _wrap(base + dtheta + math.pi),
                 _wrap(base + dtheta - math.pi)):
        if lo + limit_margin <= cand <= hi - limit_margin:
            return cand
    return base   # no in-limit roll found; keep current


def in_limits(cfg: Dict[str, float], margin: float = 0.0) -> bool:
    for name, val in cfg.items():
        lo, hi = LIMITS[name]
        if val < lo + margin or val > hi - margin:
            return False
    return True


def to_arm_array(cfg: Dict[str, float]) -> List[float]:
    """Joint positions in controllers.yaml order (for FollowJointTrajectory)."""
    return [cfg[n] for n in ARM_JOINT_ORDER]


# ---------------------------------------------------------------------------
# Inverse kinematics
# ---------------------------------------------------------------------------
def inverse_kinematics(target: Tuple[float, float, float],
                       tilt: float,
                       wrist_roll: float = 0.0,
                       limit_margin: float = 0.02,
                       min_clearance: Optional[float] = None,
                       seed: Optional[Dict[str, float]] = None,
                       ) -> Optional[Dict[str, float]]:
    """Solve for a fingertip at ``target`` with the given downward tilt.

    ``tilt`` is the approach angle measured *below horizontal*, in [0, pi/2]:
    ``0`` is a horizontal side approach, ``pi/2`` is a perfectly vertical
    top-down tool. The hand link points toward the object and downward by
    ``tilt`` regardless of which side of the shoulder the object is on.

    Both base-yaw branches are tried. Because J6's range is limited and
    asymmetric, some objects are only reachable with the arm folded *backward*
    over the base (the reach distance ``q`` along the yawed plane is negative);
    this is allowed. Both elbow solutions are tried and the configuration with
    the highest elbow (arm up, clear of the table) is preferred. If
    ``min_clearance`` is given, solutions whose elbow or wrist drop below that
    world-z are rejected (crude table-collision guard).

    Returns a joint dict (J2..J6) respecting all limits, or ``None``.
    """
    x, y, z = target
    psi = math.atan2(y, x)              # target azimuth
    candidates: List[Tuple[float, Dict[str, float]]] = []

    for t6 in (_wrap(psi - math.pi / 2), _wrap(psi + math.pi / 2)):
        lo6, hi6 = LIMITS["xarm_6_joint"]
        if t6 < lo6 + limit_margin or t6 > hi6 - limit_margin:
            continue
        # Reach axis (frame +Y after yaw) azimuth; q = signed reach distance.
        reach_az = t6 + math.pi / 2
        q = x * math.cos(reach_az) + y * math.sin(reach_az)
        if abs(q) < 1e-6:
            continue
        # Hand-link direction (wrist->tip): toward the object side, tilted down.
        dir_s = math.copysign(math.cos(tilt), q)
        dir_z = -math.sin(tilt)
        phi3 = math.atan2(dir_z, dir_s)
        qw = q - A3 * dir_s
        zw = (z - A3 * dir_z) - Z_PIVOT
        L2 = qw * qw + zw * zw
        L = math.sqrt(L2)
        if L < 1e-9:
            continue
        cos_d = (L2 - A1 * A1 - A2 * A2) / (2 * A1 * A2)
        if abs(cos_d) > 1.0:            # wrist out of 2R reach
            continue

        gamma = math.atan2(zw, qw)
        cos_a1 = (L2 + A1 * A1 - A2 * A2) / (2 * A1 * L)
        a1ang = math.acos(max(-1.0, min(1.0, cos_a1)))
        for sign in (+1.0, -1.0):       # elbow-up / elbow-down
            phi1 = gamma + sign * a1ang
            ex, ez = A1 * math.cos(phi1), A1 * math.sin(phi1)
            phi2 = math.atan2(zw - ez, qw - ex)
            cfg = {
                "xarm_6_joint": t6,
                "xarm_5_joint": _wrap(phi1 - math.pi / 2),
                "xarm_4_joint": _wrap(phi2 - phi1),
                "xarm_3_joint": _wrap(phi3 - phi2),
                "xarm_2_joint": _wrap(wrist_roll),
            }
            if not in_limits(cfg, limit_margin):
                continue
            elbow_world_z = Z_PIVOT + ez   # height of the J4 (elbow) point
            wrist_world_z = Z_PIVOT + zw   # height of the J3 (wrist) point
            if min_clearance is not None and (
                    elbow_world_z < min_clearance or wrist_world_z < min_clearance):
                continue
            candidates.append((elbow_world_z, cfg))

    if not candidates:
        return None
    if seed is not None:
        # Continuity: pick the solution closest in joint space to ``seed`` so a
        # multi-waypoint motion stays in ONE branch (no mid-carry arm flip).
        def jdist(cfg: Dict[str, float]) -> float:
            return sum(_wrap(cfg[n] - seed[n]) ** 2
                       for n in ARM_JOINT_ORDER if n in seed)
        candidates.sort(key=lambda c: jdist(c[1]))
    else:
        candidates.sort(key=lambda c: c[0], reverse=True)   # prefer highest elbow
    return candidates[0][1]


def feasible_approach(target: Tuple[float, float, float],
                      tilt_max: float = math.radians(88.0),
                      tilt_min: float = math.radians(0.0),
                      step: float = math.radians(1.0),
                      wrist_roll: float = 0.0,
                      min_clearance: Optional[float] = None,
                      seed: Optional[Dict[str, float]] = None,
                      ) -> Optional[Tuple[float, Dict[str, float]]]:
    """Find the steepest (most top-down) reachable downward tilt for ``target``.

    Sweeps the tilt from ``tilt_max`` (near-vertical) downward toward horizontal
    and returns the first feasible ``(tilt, joint-config)`` pair, i.e. the most
    top-down grasp the arm can actually reach within joint limits (and optional
    table clearance). Returns ``None`` if nothing in the range works.
    """
    tilt = tilt_max
    while tilt >= tilt_min - 1e-9:
        cfg = inverse_kinematics(target, tilt, wrist_roll=wrist_roll,
                                 min_clearance=min_clearance, seed=seed)
        if cfg is not None:
            return tilt, cfg
        tilt -= step
    return None
