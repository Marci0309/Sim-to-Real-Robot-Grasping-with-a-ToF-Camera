#!/usr/bin/env python3
"""Phase 4C perception -- perceived object pose (TABLE frame) for a real grasp.

Reuses the EXACT Phase-4B depth-segmentation (``ingest_real_scene.perceived_blobs_table``
+ ``board_4b/table_extrinsics.json``), so the perceived XY is identical to 4B by
construction. The difference from 4B: there is NO ArUco marker on the object
during a 4C grasp (it would block the gripper), so blobs cannot be labelled by
marker. The 4C experiment therefore captures **one object at a time** (place the
object, capture depth, grasp it), and this script assigns the dominant blob to
the named ``--object``. An optional ``--hint-x/-y`` (table-frame, m) disambiguates
if stray blobs are present.

Writes ``captures_4c/<name>_perceived.json``:
    {"scene": "...", "objects": {"golf_ball": {"table_xy": [x, y], "z": z}}}

Validate (no hardware) against an existing 4B scene -- ``--all`` dumps every blob,
which must match what ingest_real_scene perceives:
    python3 perceive_scene_4c.py --name scene01 --captures-dir ./captures_4b --all
Real use (single object on the table):
    python3 perceive_scene_4c.py --name p1_golf --object golf_ball
"""
import argparse
import json
import math
import os

import cv2
import numpy as np

from ingest_real_scene import (perceived_blobs_table, roi_filter_above, ROI_TABLE_XY,
                                OBJECT_HEIGHT_MIN_M, MIN_OBJECT_AREA_PX, MIN_OBJECT_POINTS)

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, ".."))                 # tof_perception
BOARD_DIR = os.path.join(PKG, "board_4b")
OUT_DIR = os.path.join(PKG, "captures_4c")


def load_extrinsics():
    ext = json.load(open(os.path.join(BOARD_DIR, "table_extrinsics.json")))
    return (np.array(ext["R_table_from_camera"]),
            np.array(ext["t_table_from_camera"]),
            ext["tof_intrinsics"])


def pick_blob(blobs, hint=None):
    """Choose the target blob: nearest --hint if given, else the blob nearest the
    mean of all blobs (the central/dominant one for a single-object scene)."""
    if not blobs:
        return None
    if hint is not None:
        return min(blobs, key=lambda b: math.hypot(b[0] - hint[0], b[1] - hint[1]))
    if len(blobs) == 1:
        return blobs[0]
    mean = np.mean(np.array(blobs)[:, :2], axis=0)
    return min(blobs, key=lambda b: math.hypot(b[0] - mean[0], b[1] - mean[1]))


def blob_minor_axis(depth_m, K_d, R_tc, t_tc, cxy, roi=None, max_dist=0.05):
    """PCA minor axis (table-frame XY unit vector) of the depth blob nearest
    ``cxy`` -- the direction the gripper jaws should close ACROSS. Uses the SAME
    segmentation (and ``roi``) as ``perceived_blobs_table``. Returns
    ``(minor_axis, elongation)`` or ``(None, None)`` if no blob matches.
    ``elongation`` ~1 means near-circular (axis ill-defined)."""
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not valid.any():
        return None, None
    above = valid & (depth_m < float(np.median(depth_m[valid])) - OBJECT_HEIGHT_MIN_M)
    above = roi_filter_above(above, depth_m, K_d, R_tc, t_tc, roi)
    n_lab, lab, stats, _ = cv2.connectedComponentsWithStats(above.astype(np.uint8), 8)
    fx, fy, cx, cy = K_d["fx"], K_d["fy"], K_d["cx"], K_d["cy"]
    best = None
    for comp in range(1, n_lab):
        if stats[comp, cv2.CC_STAT_AREA] < MIN_OBJECT_AREA_PX:
            continue
        ys, xs = np.where((lab == comp) & valid)
        if len(xs) < MIN_OBJECT_POINTS:
            continue
        z = depth_m[ys, xs].astype(np.float64)
        X = (xs - cx) * z / fx
        Y = (ys - cy) * z / fy
        pts = (R_tc @ np.stack([X, Y, z], 1).T).T + t_tc
        c = np.median(pts, 0)
        d = math.hypot(c[0] - cxy[0], c[1] - cxy[1])
        if d <= max_dist and (best is None or d < best[0]):
            best = (d, pts)
    if best is None:
        return None, None
    xy = best[1][:, :2] - best[1][:, :2].mean(0)
    ev, evec = np.linalg.eigh(xy.T @ xy / len(xy))    # ascending eigenvalues
    minor = evec[:, 0]
    elong = float(math.sqrt(ev[1] / max(ev[0], 1e-12)))
    return (float(minor[0]), float(minor[1])), elong


def main():
    ap = argparse.ArgumentParser(description="Phase 4C perceived pose (table frame)")
    ap.add_argument("--name", required=True, help="scene name (e.g. p1_golf or scene01)")
    ap.add_argument("--object", help="object label to assign the dominant blob to")
    ap.add_argument("--captures-dir", default=os.path.join(PKG, "captures_4c"))
    ap.add_argument("--hint-x", type=float, default=None)
    ap.add_argument("--hint-y", type=float, default=None)
    ap.add_argument("--all", action="store_true", help="dump every blob (validation)")
    ap.add_argument("--no-roi", action="store_true",
                    help="disable the reachable-table ROI (segment the whole frame)")
    args = ap.parse_args()
    roi = None if args.no_roi else ROI_TABLE_XY

    depth_path = os.path.join(args.captures_dir, args.name + "_depth.npy")
    if not os.path.exists(depth_path):
        raise SystemExit(f"missing {depth_path} (run capture_scene_4b.py --name {args.name})")
    R_tc, t_tc, K_d = load_extrinsics()
    depth_m = np.load(depth_path).astype(np.float32) / 1000.0
    blobs = perceived_blobs_table(depth_m, K_d, R_tc, t_tc, roi=roi)

    print(f"scene '{args.name}': {len(blobs)} depth blob(s) (table frame):")
    for i, b in enumerate(blobs):
        print(f"  blob {i}: x={b[0]*1000:7.1f}  y={b[1]*1000:7.1f}  z={b[2]*1000:7.1f} mm")

    if args.all:
        return

    if not args.object:
        raise SystemExit("--object is required unless --all is given.")
    hint = (args.hint_x, args.hint_y) if args.hint_x is not None else None
    b = pick_blob(blobs, hint)
    if b is None:
        raise SystemExit(f"no blob found for {args.object} in '{args.name}'.")
    if len(blobs) > 1 and hint is None:
        print(f"[note] {len(blobs)} blobs present and no --hint; assigned the central "
              f"one to {args.object}. Prefer single-object scenes for 4C.")

    minor, elong = blob_minor_axis(depth_m, K_d, R_tc, t_tc, (b[0], b[1]), roi=roi)
    obj = {"table_xy": [float(b[0]), float(b[1])], "z": float(b[2])}
    if minor is not None:
        obj["minor_axis"] = [minor[0], minor[1]]
        obj["elongation"] = round(elong, 2)
        print(f"  PCA minor axis (jaws close across this): ({minor[0]:+.3f}, "
              f"{minor[1]:+.3f})  elongation {elong:.2f}"
              + ("  [near-circular: axis weak]" if elong < 1.15 else ""))
    out = {"scene": args.name, "objects": {args.object: obj}}
    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, args.name + "_perceived.json")
    json.dump(out, open(out_path, "w"), indent=2)
    print(f"\n{args.object} -> table_xy "
          f"[{b[0]*1000:.1f}, {b[1]*1000:.1f}] mm   wrote {out_path}")


if __name__ == "__main__":
    main()
