#!/usr/bin/env python3
"""
Phase 4B / step 7 -- ingest a captured REAL scene into evaluation/results/run.csv.

For one captured scene (captures_4b/<name>_depth.npy + <name>_rgb.png):

  PERCEIVED (what the perception sees): depth-segment the ToF depth with the
    SAME pipeline as the sim pose_estimator -- table plane = median depth, pixels
    >= 5 mm closer are "object", connected components -> one blob per object,
    per-axis MEDIAN back-projected centroid -- then map the ToF centroid into the
    table frame via board_4b/table_extrinsics.json (p_table = R @ p_tof + t).

  TRUTH (ground truth): detect the per-object ArUco markers in the sharp RGB
    (markers_4b/markers.json), solve each marker's pose, map it into the table
    frame via the saved board pose (table_extrinsics.json: rvec/tvec_board_in_rgb),
    and apply the marker->object offset -> the object's true table-frame position.

Each depth blob is matched to the nearest object-truth (greedy nearest-neighbour,
exactly like the sim pose_evaluator), the position error is computed, and ONE
row is appended to run.csv with the SAME schema as the sim rows, tagged `real`
(override with --tag), so the sim and real conditions sit side by side for the
step-8 sim-to-real gap.

METRIC NOTE: the flat ArUco markers give a reliable PLANAR (table-XY) ground
truth; they do NOT give a clean visible-top Z truth (the sim aligns Z to the
visible top per Hodan/BOP, which a flat marker can't reproduce). So the
comparable metric here is the PLANAR error: `<obj>_dx`, `<obj>_dy`, and
`<obj>_norm = sqrt(dx^2+dy^2)`. `<obj>_dz` is written as the perceived centroid
height above the marker plane (informational only). The step-8 comparison should
use the PLANAR error for BOTH sim and real (recompute the sim's planar norm from
its dx,dy).

Run (WSL):  python3 ingest_real_scene.py --name scene01            # tag=real
            python3 ingest_real_scene.py --name scene01 --tag real_noisy
"""
import argparse
import csv
import json
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, ".."))                       # tof_perception
BOARD_DIR = os.path.join(PKG, "board_4b")
MARKERS_JSON = os.path.join(PKG, "markers_4b", "markers.json")
CALIB = os.path.normpath(os.path.join(PKG, "..", "..", "oakd_calib.json"))
RUN_CSV = os.path.normpath(os.path.join(PKG, "..", "evaluation", "results", "run.csv"))

# Row order MUST match run.csv's header (pose_evaluator.OBJECTS order).
OBJECT_ORDER = ["coke_can", "banana", "golf_ball", "blue_box"]

# Depth segmentation -- identical constants to the sim pose_estimator.
OBJECT_HEIGHT_MIN_M = 0.005     # 5 mm above the table -> object
MIN_OBJECT_AREA_PX = 100        # smaller depth blobs are noise
MIN_OBJECT_POINTS = 30          # need this many valid 3D points for a centroid
MATCH_TOL_M = 0.04              # blob <-> truth nearest-neighbour match radius.
                               # Valid real perception errors are <~16 mm; a junk
                               # blob grabbed when the object is unsegmented (edge of
                               # the ToF zone) sits >70 mm away. 40 mm cleanly accepts
                               # the former and rejects the latter -> an unsegmented
                               # object becomes an honest no-detection, not a fake error.


def get_dictionary(name):
    code = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(code)
    return cv2.aruco.Dictionary_get(code)


def detect_markers(gray, dictionary):
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    return corners, ids


def marker_pose(corner, marker_len, K, dist):
    """Pose of one marker (corner shape (1,4,2)) in the camera frame.
    estimatePoseSingleMarkers if present, else solvePnP (version-robust)."""
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corner], marker_len, K, dist)
        return np.asarray(rvecs[0]).reshape(3), np.asarray(tvecs[0]).reshape(3)
    h = marker_len / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, corner.reshape(-1, 2), K, dist)
    return rvec.reshape(3), tvec.reshape(3)


def homog(rvec, tvec):
    T = np.eye(4)
    T[:3, :3], _ = cv2.Rodrigues(np.asarray(rvec, float))
    T[:3, 3] = np.asarray(tvec, float).reshape(3)
    return T


def rgb_intrinsics(calib, socket, W, H):
    """CAM_B intrinsics + distortion scaled to the captured resolution
    (mirrors solve_table_extrinsics.py)."""
    cam = calib["cameras"][socket]
    nat = cam["intrinsics_native"]
    sx, sy = W / nat["width"], H / nat["height"]
    K = np.array([[nat["fx"] * sx, 0, nat["cx"] * sx],
                  [0, nat["fy"] * sy, nat["cy"] * sy],
                  [0, 0, 1.0]])
    dist = np.array(cam.get("distortion", []), dtype=np.float64).ravel()
    if dist.size == 0:
        dist = np.zeros(5)
    return K, dist


def perceived_blobs_table(depth_m, K_d, R_tc, t_tc):
    """Sim-style depth segmentation -> list of object-blob centroids in the
    table frame. Returns [(x,y,z), ...]."""
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not valid.any():
        return []
    table_depth = float(np.median(depth_m[valid]))
    above = valid & (depth_m < table_depth - OBJECT_HEIGHT_MIN_M)
    n_lab, lab, stats, _ = cv2.connectedComponentsWithStats(
        above.astype(np.uint8), 8)
    fx, fy, cx, cy = K_d["fx"], K_d["fy"], K_d["cx"], K_d["cy"]
    out = []
    for comp in range(1, n_lab):
        if stats[comp, cv2.CC_STAT_AREA] < MIN_OBJECT_AREA_PX:
            continue
        ys, xs = np.where((lab == comp) & valid)
        if len(xs) < MIN_OBJECT_POINTS:
            continue
        z = depth_m[ys, xs].astype(np.float64)
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        c_tof = np.array([np.median(x), np.median(y), np.median(z)])
        out.append(R_tc @ c_tof + t_tc)            # -> table frame
    return out


def greedy_match(truth, blobs):
    """Greedy nearest-neighbour (planar XY): {object: blob_xyz or None}.
    Each blob/truth used at most once; only within MATCH_TOL_M."""
    result = {o: None for o in truth}
    pairs = []
    for o, t in truth.items():
        for bi, b in enumerate(blobs):
            d = math.hypot(b[0] - t[0], b[1] - t[1])
            if d <= MATCH_TOL_M:
                pairs.append((d, o, bi))
    pairs.sort()
    used_obj, used_blob = set(), set()
    for d, o, bi in pairs:
        if o in used_obj or bi in used_blob:
            continue
        result[o] = blobs[bi]
        used_obj.add(o)
        used_blob.add(bi)
    return result


def next_t_sec(csv_path, tag):
    """Per-tag sample index so each ingested scene gets a unique t_sec."""
    if not os.path.exists(csv_path):
        return 0
    n = 0
    with open(csv_path) as fh:
        for row in csv.reader(fh):
            if row and row[0] == tag:
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True, help="scene tag, e.g. scene01")
    ap.add_argument("--captures-dir", default=os.path.join(PKG, "captures_4b"))
    ap.add_argument("--tag", default="real", help="run.csv tag (default: real)")
    ap.add_argument("--run-csv", default=RUN_CSV)
    args = ap.parse_args()

    cap = os.path.join(args.captures_dir, args.name)
    ext = json.load(open(os.path.join(BOARD_DIR, "table_extrinsics.json")))
    mk = json.load(open(MARKERS_JSON))
    calib = json.load(open(CALIB))

    R_tc = np.array(ext["R_table_from_camera"])
    t_tc = np.array(ext["t_table_from_camera"])
    K_d = ext["tof_intrinsics"]
    socket = ext.get("rgb_socket", "CAM_B")
    # T_table<-rgb = inv(T_rgb<-board); board frame IS the table frame.
    T_rgb_board = homog(ext["rvec_board_in_rgb"], ext["tvec_board_in_rgb"])
    T_table_rgb = np.linalg.inv(T_rgb_board)

    # ---- TRUTH: object markers in RGB -> table frame ------------------------
    rgb = cv2.imread(cap + "_rgb.png")
    if rgb is None:
        raise SystemExit("missing %s_rgb.png (run capture_rgb_4b.py)" % cap)
    H, W = rgb.shape[:2]
    K_rgb, dist_rgb = rgb_intrinsics(calib, socket, W, H)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    dictionary = get_dictionary(mk["dictionary"])
    corners, ids = detect_markers(gray, dictionary)

    id_to_obj = {int(k): v for k, v in mk["id_to_object"].items()}
    offsets = mk.get("marker_to_object_offset_m", {})
    marker_len = float(mk["marker_length_m"])

    truth = {}
    seen_ids = []
    if ids is not None:
        for c, i in zip(corners, ids.flatten()):
            i = int(i)
            if i not in id_to_obj:
                continue
            obj = id_to_obj[i]
            seen_ids.append(i)
            rvec, tvec = marker_pose(c, marker_len, K_rgb, dist_rgb)
            T_table_marker = T_table_rgb @ homog(rvec, tvec)
            off = offsets.get(obj, [0.0, 0.0])
            off3 = np.array([off[0], off[1], 0.0])
            truth[obj] = T_table_marker[:3, 3] + T_table_marker[:3, :3] @ off3

    # ---- PERCEIVED: ToF depth segmentation -> table frame ------------------
    if not os.path.exists(cap + "_depth.npy"):
        raise SystemExit("missing %s_depth.npy (run capture_scene_4b.py)" % cap)
    depth_m = np.load(cap + "_depth.npy").astype(np.float32) / 1000.0
    blobs = perceived_blobs_table(depth_m, K_d, R_tc, t_tc)

    matched = greedy_match(truth, blobs)

    # ---- build + append the run.csv row ------------------------------------
    tag = args.tag
    t_sec = next_t_sec(args.run_csv, tag)
    row = [tag, t_sec, 0]
    report = []
    for obj in OBJECT_ORDER:
        t = truth.get(obj)
        p = matched.get(obj)
        if t is None or p is None:
            row += ["", "", "", ""]
            why = "no marker" if t is None else "no depth blob"
            report.append("%-10s --       (%s)" % (obj, why))
            continue
        d = np.asarray(p) - np.asarray(t)
        norm_xy = float(math.hypot(d[0], d[1]))
        row += [float(d[0]), float(d[1]), float(d[2]), norm_xy]
        report.append("%-10s dx=%+6.1f dy=%+6.1f mm  planar=%5.1f mm  (dz=%+6.1f)"
                      % (obj, d[0] * 1000, d[1] * 1000, norm_xy * 1000, d[2] * 1000))

    new_file = not os.path.exists(args.run_csv)
    os.makedirs(os.path.dirname(args.run_csv), exist_ok=True)
    with open(args.run_csv, "a", newline="") as fh:
        w = csv.writer(fh)
        if new_file:
            hdr = ["tag", "t_sec", "t_nsec"]
            for o in OBJECT_ORDER:
                hdr += [o + "_dx", o + "_dy", o + "_dz", o + "_norm"]
            w.writerow(hdr)
        w.writerow(row)

    # ---- report ------------------------------------------------------------
    print("scene '%s'  (tag=%s)" % (args.name, tag))
    print("OpenCV", cv2.__version__)
    print("markers detected: %s   depth blobs: %d"
          % (sorted(id_to_obj[i] for i in seen_ids) or "none", len(blobs)))
    print("planar (table-XY) perception error vs ArUco ground truth:")
    for line in report:
        print("  " + line)
    print("\nappended 1 '%s' row -> %s" % (tag, args.run_csv))
    if len(seen_ids) < 4:
        print("[note] only %d/4 object markers detected -- check they are flat, "
              "in frame, and unobstructed in the RGB." % len(seen_ids))


if __name__ == "__main__":
    main()
