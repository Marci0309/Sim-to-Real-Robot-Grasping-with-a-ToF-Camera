#!/usr/bin/env python3
"""Phase 4C real pick-and-place executor for the Hiwonder xArm 1S.

Runs on the **Windows** real-arm host (plain CPython, no ROS). For one object +
placement + repeat it:

  1. reads the PERCEIVED object pose in the TABLE frame (from
     ``perceive_scene_4c.py`` -> ``captures_4c/<scene>_perceived.json``);
  2. transforms it to the ARM base frame via ``config/arm_base_extrinsics.json``
     (table->arm rigid transform from ``solve_arm_base_extrinsics.py``);
  3. plans the pick-and-place waypoints with the SAME geometry as the sim
     (``grasp_common.plan_pick_place`` + ``arm_ik``);
  4. drives the real arm through HOME -> pre -> grasp -> close -> lift -> place ->
     open -> retreat via ``RealArm`` (open-loop, no live feedback);
  5. prompts the operator for the PHYSICAL outcome (lift-and-hold y/n) and appends
     a row to ``evaluation/results/real_manip_results.csv``.

Always validate with ``--dry-run`` first (no device, no prompts):
    python real_grasp_executor.py --object golf_ball --scene scene01 --dry-run
Real run (after calibration + arm-base extrinsic exist):
    python real_grasp_executor.py --object golf_ball --scene scene01 --placement-id 1 --repeat 1
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
from typing import Dict, Optional, Tuple

import numpy as np

# Make the package importable whether run standalone (Windows) or in-tree.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, "..", "xarm1s_manipulation")
sys.path.insert(0, os.path.abspath(_PKG))
try:
    from xarm1s_manipulation import grasp_common as gc, arm_ik as ik, visual_servo as vs
    from xarm1s_manipulation.real_arm_driver import RealArm
except ImportError:
    import grasp_common as gc            # type: ignore
    import arm_ik as ik                  # type: ignore
    import visual_servo as vs            # type: ignore
    from real_arm_driver import RealArm  # type: ignore

_CFG = os.path.abspath(os.path.join(_HERE, "..", "config"))
_RES = os.path.abspath(os.path.join(_HERE, "..", "..", "evaluation", "results"))
_CAP = os.path.abspath(os.path.join(_HERE, "..", "..", "tof_perception", "captures_4c"))
CSV_PATH = os.path.join(_RES, "real_manip_results.csv")
CSV_FIELDS = ["tag", "object", "placement_id", "repeat", "perceived_x",
              "perceived_y", "arm_x", "arm_y", "tilt_deg", "ik_feasible",
              "gripper_closed", "lift_success", "place_success", "note"]

# Real-bench fingertip grasp/place height. Last session's table-height finding:
# the closing fingers swing down ~4 cm, so commanding the fingertip to ~0.82 m
# grips the body with the tips clear of the (higher-than-model) real table. The
# sim/model uses 0.755. The APPROACH TILT is searched at the model seed height
# (TILT_SEED_Z) then applied at this height -- the validated demo recipe. Tunable
# per run via --grasp-z.
DEFAULT_GRASP_Z = 0.82
TILT_SEED_Z = 0.755


def load_arm_base(path: Optional[str]) -> Tuple[np.ndarray, np.ndarray, float, bool]:
    """Return (R 3x3, t 3, scale, calibrated?). Falls back to identity (dry-run only).

    ``scale`` (default 1.0 = rigid) absorbs the real-arm FK reach-scale error
    measured in Phase 4C: the real xArm 1S reaches ~28% farther than its URDF/
    arm_ik model, so the table->arm map is a similarity, not a rigid transform
    (see fit_arm_base_extrinsics.py). Applied BEFORE IK; because the same FK/IK
    runs end to end, the real fingertip still lands at the true target.
    """
    p = path or os.path.join(_CFG, "arm_base_extrinsics.json")
    if os.path.exists(p):
        d = json.load(open(p))
        return (np.array(d["R_arm_from_table"], float),
                np.array(d["t_arm_from_table"], float),
                float(d.get("scale", 1.0)), True)
    return np.eye(3), np.zeros(3), 1.0, False


def table_to_arm(xy_table, R: np.ndarray, t: np.ndarray, scale: float = 1.0) -> np.ndarray:
    p = scale * (R @ np.array([float(xy_table[0]), float(xy_table[1]), 0.0])) + t
    return p[:2]


def load_perceived(scene: str) -> Dict[str, dict]:
    p = os.path.join(_CAP, f"{scene}_perceived.json")
    if not os.path.exists(p):
        raise SystemExit(f"perceived poses not found: {p}\n"
                         f"  run perceive_scene_4c.py --name {scene} first.")
    return json.load(open(p))["objects"]


def append_row(row: dict, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


def _score(dry_run: bool) -> Tuple[str, str, str]:
    """Operator-scored physical outcome. Skipped in dry-run."""
    if dry_run or not sys.stdin.isatty():
        return "", "", "dry-run"
    lift = input("  lift-and-hold success? [y/N]: ").strip().lower().startswith("y")
    place = input("  placed on target? [y/N]: ").strip().lower().startswith("y")
    note = input("  note (optional): ").strip()
    return str(int(lift)), str(int(place)), note


# --------------------------------------------------------------------------- #
#  Closed-loop visual nudge (Phase 4C) -- see visual_servo.py + the plan doc.    #
#  Optional + additive: servo=None leaves the open-loop path byte-identical. The  #
#  loop measures the gripper marker vs the object through the SAME overhead       #
#  camera, so the wrong FK cancels as common-mode bias (accuracy = camera-bound). #
# --------------------------------------------------------------------------- #
def _plan_for_target(label, tgt_table, R, t, scale, grasp_z, minor_arm):
    """(arm_xy, tilt, wps) for a TABLE-frame target, or None if unreachable."""
    a_xy = table_to_arm(tgt_table, R, t, scale)
    res = ik.feasible_approach((a_xy[0], a_xy[1], TILT_SEED_Z), min_clearance=0.75)
    if res is None:
        return None
    tilt, _ = res
    w = gc.plan_pick_place(label, a_xy, tilt, grasp_z=grasp_z, minor_axis=minor_arm)
    if w is None:
        return None
    return a_xy, tilt, w


def _synthetic_observer(state, tip):
    """Dry-run only: the gripper MARKER 'lands' at a constant ~14 mm biased transform
    of the last commanded target (mimicking the position-dependent FK error), so the
    closed-loop executor path runs end-to-end with no camera/arm.
    marker = cmd + bias - tip  ->  err = obj - (marker + tip) = obj - (cmd + bias)."""
    def observe():
        cx, cy = state["cmd"]
        bx, by = state["bias"]
        return (cx + bx - tip[0], cy + by - tip[1])
    return observe


def _real_gripper_observer(servo):
    """BENCH path (hardware): capture an RGB via the depthai py-3.14 script (Windows) then
    detect the gripper marker via detect_gripper_marker.py (WSL cv2). Returns the marker XY
    in the table frame, or **None** on a miss (capture failed / marker not found). Returning
    None makes the loop HOLD AIM and retry -- it never fakes convergence. FRESHNESS-GUARDED:
    the RGB is deleted before capture and re-checked (returncode + mtime), so a failed capture
    can't be re-detected as a stale frame. The OAK-D can drop X_LINK on rapid re-opens, so a
    failed CAPTURE is retried with a short delay (lets the device reboot); a clean capture with
    no marker is a real miss (no retry)."""
    import time
    time.sleep(servo.get("settle", 1.2))          # let a wobbling/floppy marker mount stop moving
    rgb = servo["rgb_path"]
    for attempt in range(servo.get("cap_retries", 3)):
        try:
            try:
                os.remove(rgb)
            except FileNotFoundError:
                pass
            t0 = time.time()
            cp = subprocess.run(servo["capture_cmd"] + ["--name", servo["cap_name"]],
                                check=False, timeout=90)
            if cp.returncode == 0 and os.path.exists(rgb) and os.path.getmtime(rgb) >= t0:
                out = subprocess.run(servo["detect_cmd"], capture_output=True,
                                     text=True, timeout=90)
                d = json.loads(out.stdout.strip().splitlines()[-1])
                if d.get("found"):
                    return (float(d["table_xy"][0]), float(d["table_xy"][1]))
                print("  [servo] gripper marker not detected this step -- holding aim.")
                return None
            print(f"  [servo] capture failed (rc={cp.returncode}); retry {attempt + 1}...")
        except Exception as e:                              # noqa: BLE001
            print(f"  [servo] observe error ({e}); retry {attempt + 1}...")
        time.sleep(servo.get("cap_delay", 2.0))             # let the OAK-D reboot before re-open
    print("  [servo] capture failed after retries -- holding aim.")
    return None


def _run_servo(label, obj_xy, R, t, scale, grasp_z, minor_arm, arm, servo, dry_run):
    """Closed-loop nudge at the hover pose. Returns (wps_to_descend, row_fields);
    wps is None if a nudged target became unreachable (recorded as a miss)."""
    tip = (servo["tip_dx"] / 1000.0, servo["tip_dy"] / 1000.0)
    state = {"cmd": (float(obj_xy[0]), float(obj_xy[1])), "wps": None,
             "unreachable": False, "bias": servo.get("bias", (0.012, -0.008))}

    def command(tgt):
        p = _plan_for_target(label, tgt, R, t, scale, grasp_z, minor_arm)
        if p is None:
            state["unreachable"] = True
            return
        state["cmd"] = (float(tgt[0]), float(tgt[1]))
        state["wps"] = p[2]
        arm.move_joints(p[2]["pre"])                         # hover (dry-run just prints)

    if dry_run:
        observe = _synthetic_observer(state, tip)
    else:
        def observe():
            return _real_gripper_observer(servo)
    res = vs.servo_align(obj_xy, observe, command, tip_offset=tip, gain=servo["gain"],
                         thresh_m=servo["thresh"], max_iter=servo["iters"])
    real = [e for e in res["history"] if e.get("err_mm") is not None]   # steps that measured
    s0 = real[0]["err_mm"] if real else None
    s1 = real[-1]["err_mm"] if real else None
    meas, misses = res.get("measured", len(real)), res.get("misses", 0)
    # A real run where the marker was NEVER detected is an OPEN-LOOP landing, not a
    # convergence -- and a miss-induced break can never be logged as 'conv'.
    if not dry_run and meas == 0:
        status = "UNMEASURED-openloop"
    else:
        status = "conv" if res["converged"] else "NOCONV"
    miss = f" miss{misses}" if misses else ""
    print(f"  [servo] {meas} meas / {res['iters']} steps, err {s0}->{s1} mm, {status}{miss}")
    note = f"servo {meas}meas {s0}->{s1}mm {status}{miss}"
    if state["unreachable"] or state["wps"] is None:
        return None, {"note": note + " UNREACHABLE"}
    return state["wps"], {"note": note}


def run_one(label: str, scene: str, placement_id: int, repeat: int,
            arm: RealArm, R: np.ndarray, t: np.ndarray, scale: float,
            dry_run: bool, grasp_z: float = DEFAULT_GRASP_Z,
            pick_only: bool = False, close_mm: Optional[float] = None,
            off_x: float = 0.0, off_y: float = 0.0,
            servo: Optional[dict] = None) -> dict:
    d = gc.OBJECTS[label]
    perceived = load_perceived(scene)
    if label not in perceived:
        print(f"  {label} not detected in {scene}; recording as a miss.")
        row = dict(tag="real", object=label, placement_id=placement_id,
                   repeat=repeat, perceived_x="", perceived_y="", arm_x="",
                   arm_y="", tilt_deg="", ik_feasible=0, gripper_closed=0,
                   lift_success=0, place_success=0, note="not detected")
        return row

    xy_table = perceived[label]["table_xy"]
    # Constant aim nudge in the TABLE frame (mm->m) for a fixed base/camera drift,
    # applied BEFORE the table->arm transform so it shifts the physical landing by the
    # SAME vector in the table/camera frame the operator sees. (The calibration R is
    # ~85 deg, so an arm-frame nudge would move almost perpendicular to the table axes.)
    # Same value for every object; default 0 leaves the aim unchanged. Under --servo the
    # closed loop supersedes the manual nudge, so we DON'T apply it -- it would only corrupt
    # the recorded perceived_x/y provenance (the servo target reads the raw perceived pose).
    if servo is None:
        xy_table = (float(xy_table[0]) + off_x / 1000.0, float(xy_table[1]) + off_y / 1000.0)
    arm_xy = table_to_arm(xy_table, R, t, scale)

    # tilt from the steepest feasible approach, searched at the model SEED height
    # then applied at the real grasp_z below (matches the validated demo recipe)
    res = ik.feasible_approach((arm_xy[0], arm_xy[1], TILT_SEED_Z), min_clearance=0.75)
    row = dict(tag="real", object=label, placement_id=placement_id, repeat=repeat,
               perceived_x=round(float(xy_table[0]), 5),
               perceived_y=round(float(xy_table[1]), 5),
               arm_x=round(float(arm_xy[0]), 5), arm_y=round(float(arm_xy[1]), 5),
               tilt_deg="", ik_feasible=0, gripper_closed=0,
               lift_success=0, place_success=0, note="")
    if res is None:
        row["note"] = "IK infeasible at this placement"
        print(f"  {label}: IK infeasible at arm XY {arm_xy.round(3)} -- recorded.")
        return row
    tilt, _ = res
    row["tilt_deg"] = round(math.degrees(tilt), 1)

    # Use the object's PREDETERMINED nominal minor axis (same grasp recipe as sim),
    # then rotate it into the ARM frame via R (the yaw half of the transform). Ignoring
    # the perceived PCA axis keeps real and sim grasp planning identical, so the measured
    # gap is pure execution, not a perception confound. Assumes each object is placed at
    # its nominal orientation. Symmetric/missing axes (None) leave the wrist roll untouched.
    minor_table = d.get("minor_axis")
    minor_arm = None
    if minor_table is not None:
        v = R @ np.array([float(minor_table[0]), float(minor_table[1]), 0.0])
        minor_arm = (float(v[0]), float(v[1]), float(v[2]))

    wps = gc.plan_pick_place(label, arm_xy, tilt, grasp_z=grasp_z, minor_axis=minor_arm)
    if wps is None:
        row["note"] = "waypoint IK failed"
        print(f"  {label}: a waypoint was unreachable -- recorded.")
        return row
    row["ik_feasible"] = 1

    # Grip force = how far PAST the object width the jaws are commanded (they stall
    # on the object). The nominal grasp_width was too wide here, so allow a tighter
    # absolute close via --close-mm for a firm pinch.
    close_w = (close_mm / 1000.0) if close_mm else max(0.006, d["grasp_width"] - gc.GRIP_SQUEEZE)
    print(f"  {label}: table{np.round(xy_table,3).tolist()} -> arm"
          f"{arm_xy.round(3).tolist()}, tilt {row['tilt_deg']} deg, "
          f"grasp_z {grasp_z:.3f}, close {close_w*1000:.0f}mm")

    # --- the pick-and-place motion (mirrors grasp_executor.run_trial) ---
    arm.set_gripper(0.055)                              # open
    arm.move_joints(gc.HOME_CFG)
    if servo is not None:
        # CLOSED-LOOP: hover, measure the gripper marker vs the object, nudge the
        # table-frame target until the gripper is on it, THEN descend (FK-independent;
        # supersedes the open-loop --off-x/--off-y aim).
        obj_xy = (float(perceived[label]["table_xy"][0]),
                  float(perceived[label]["table_xy"][1]))
        wps, srow = _run_servo(label, obj_xy, R, t, scale, grasp_z, minor_arm,
                               arm, servo, dry_run)
        row["note"] = srow["note"]
        if wps is None:
            print(f"  {label}: servo target became unreachable -- recorded.")
            return row
    else:
        arm.move_joints(wps["pre"])
    arm.move_joints(wps["grasp"])                       # descend at the corrected target
    arm.set_gripper(close_w)                            # close ONTO the object
    row["gripper_closed"] = 1
    arm.move_joints(wps["grasp"]); arm.move_joints(wps["pre"])      # lift
    if pick_only:
        print("  [pick-only] holding the lifted object at the pre-grasp pose "
              "(skipping carry/place/retreat).")
    else:
        arm.move_joints(wps["place_pre"]); arm.move_joints(wps["place"])  # carry
        arm.set_gripper(0.055)                          # release
        arm.move_joints(wps["place"]); arm.move_joints(wps["place_pre"]); arm.move_joints(gc.HOME_CFG)

    servo_note = row.get("note", "") or ""
    lift, place, note = _score(dry_run)
    row["lift_success"], row["place_success"] = lift, place
    row["note"] = f"{servo_note} | {note}" if servo_note else note
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description="Phase 4C real pick-and-place")
    ap.add_argument("--object", required=True, choices=list(gc.OBJECTS))
    ap.add_argument("--scene", required=True, help="perceived scene name (e.g. scene01)")
    ap.add_argument("--placement-id", type=int, default=0)
    ap.add_argument("--repeat", type=int, default=0)
    ap.add_argument("--calib", default=os.path.join(_CFG, "servo_calib.json"),
                    help="servo_calib.json path (defaults to config/servo_calib.json "
                         "so the real run never falls back to the NOMINAL map)")
    ap.add_argument("--extrinsics", default=None, help="arm_base_extrinsics.json path")
    ap.add_argument("--grasp-z", type=float, default=DEFAULT_GRASP_Z,
                    help="real fingertip grasp/place height (m); baked from the "
                         "validated phase4c demo (real table sits higher than the model)")
    ap.add_argument("--pick-only", action="store_true",
                    help="stop after the lift (skip carry/place) -- safe for first grasps")
    ap.add_argument("--close-mm", type=float, default=None,
                    help="absolute jaw width to command at the grasp (mm); command "
                         "well past the object width for a firm pinch (overrides grasp_width)")
    ap.add_argument("--off-x", type=float, default=0.0,
                    help="constant aim nudge in X (mm, TABLE frame) for a fixed base/camera "
                         "drift; +x shifts the grasp along the table's +X; same value every object")
    ap.add_argument("--off-y", type=float, default=0.0,
                    help="constant aim nudge in Y (mm, TABLE frame); see --off-x")
    ap.add_argument("--place-x", type=float, default=None,
                    help="override the drop-spot X (ARM frame, m); e.g. the blue_box spot 0.19. "
                         "Smaller magnitude = closer to the base (safer on-table).")
    ap.add_argument("--place-y", type=float, default=None,
                    help="override the drop-spot Y (ARM frame, m); e.g. the blue_box spot -0.05")
    ap.add_argument("--servo", action="store_true",
                    help="CLOSED-LOOP visual nudge: hover, measure the gripper ArUco marker "
                         "vs the object via the overhead camera, correct the aim before "
                         "descending (FK-independent; supersedes --off-x/--off-y)")
    ap.add_argument("--servo-id", type=int, default=4, help="gripper marker id (default 4)")
    ap.add_argument("--servo-len", type=float, default=0.05, help="gripper marker side (m)")
    ap.add_argument("--servo-gain", type=float, default=0.6, help="nudge gain K (0<K<2)")
    ap.add_argument("--servo-thresh", type=float, default=3.0, help="convergence radius (mm)")
    ap.add_argument("--servo-iters", type=int, default=4, help="max nudge iterations")
    ap.add_argument("--servo-settle", type=float, default=1.2,
                    help="seconds to wait after the hover move before each servo capture, so a "
                         "wobbling/floppy marker mount stops moving (raise if the marker blurs to white)")
    ap.add_argument("--tip-dx", type=float, default=0.0,
                    help="gripper marker->fingertip offset X (mm, table frame)")
    ap.add_argument("--tip-dy", type=float, default=0.0, help="marker->fingertip offset Y (mm)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    # Optional drop-spot override (ARM frame, m). The per-object place spots in
    # grasp_common are defined in the SIM frame (table == arm there); on the real
    # bench coke's far (0.27, 0.07) spot lands off the table. This overrides it
    # IN-MEMORY only -- grasp_common.py stays byte-identical and the sim is unaffected.
    if args.place_x is not None and args.place_y is not None:
        gc.OBJECTS[args.object]["place"] = (float(args.place_x), float(args.place_y))
        print(f"[exec] drop-spot override -> arm-frame "
              f"({args.place_x:.3f}, {args.place_y:.3f}) (in-memory; grasp_common untouched)")

    servo = None
    if args.servo:
        _tof = os.path.normpath(os.path.join(_HERE, "..", "..", "tof_perception", "scripts"))
        capture_cmd = ["py", "-3.14", os.path.join(_tof, "capture_rgb_4b.py"),
                       "--out-dir", _CAP, "--n-frames", "10"]
        detect_cmd = ["wsl.exe", "-d", "Ubuntu-22.04", "--", "bash", "-lc",
                      "cd ~/thesis_ws/src/tof_perception/scripts && python3 "
                      "detect_gripper_marker.py --rgb ~/thesis_ws/src/tof_perception/"
                      "captures_4c/_servo_rgb.png --id %d --len %g"
                      % (args.servo_id, args.servo_len)]
        servo = dict(gain=args.servo_gain, thresh=args.servo_thresh / 1000.0,
                     iters=args.servo_iters, tip_dx=args.tip_dx, tip_dy=args.tip_dy,
                     id=args.servo_id, len=args.servo_len, cap_name="_servo",
                     settle=args.servo_settle, rgb_path=os.path.join(_CAP, "_servo_rgb.png"),
                     capture_cmd=capture_cmd, detect_cmd=detect_cmd)
        print("[exec] CLOSED-LOOP servo ON (id=%d len=%.0fmm K=%g thresh=%gmm iters=%d); "
              "--off-x/--off-y ignored under servo."
              % (args.servo_id, args.servo_len * 1000, args.servo_gain,
                 args.servo_thresh, args.servo_iters))

    R, t, scale, calibrated = load_arm_base(args.extrinsics)
    if not calibrated:
        print("[exec] WARNING: no arm_base_extrinsics.json -- using IDENTITY "
              "table->arm transform (dry-run only; not a real grasp frame).")
    # Dry-runs go to a separate CSV so they never pollute the real results.
    out_csv = CSV_PATH.replace(".csv", ".dryrun.csv") if args.dry_run else CSV_PATH
    arm = RealArm(calib_path=args.calib, dry_run=args.dry_run)
    try:
        row = run_one(args.object, args.scene, args.placement_id, args.repeat,
                      arm, R, t, scale, args.dry_run, grasp_z=args.grasp_z,
                      pick_only=args.pick_only, close_mm=args.close_mm,
                      off_x=args.off_x, off_y=args.off_y, servo=servo)
        append_row(row, out_csv)
        print(f"[exec] recorded -> {out_csv}\n        {row}")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
