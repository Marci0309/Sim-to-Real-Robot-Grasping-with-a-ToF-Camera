#!/usr/bin/env python3
"""Phase 4C -- detect the gripper ArUco marker in an RGB frame and return its position
in the TABLE frame (the MEASUREMENT half of the closed-loop visual nudge; see
xarm1s_manipulation/xarm1s_manipulation/visual_servo.py + PHASE4C_VISUAL_SERVO_PLAN.md).

Reuses the PROVEN object-marker truth path from ingest_real_scene.py:
``detect_markers -> marker_pose(solvePnP) -> T_table_rgb @ pose``. Same overhead CAM_B
colour image + same board-in-RGB extrinsic that already locates the object markers to
~1-4 mm. The gripper marker uses the SAME ArUco dictionary as the object markers but a
DISTINCT id (default 4), so it is found in the same detection pass without colliding.

WSL python3 (needs cv2 + the OAK-D calib/extrinsic JSONs). The Windows arm host has no
cv2, so the executor calls this as a subprocess each servo step. Standalone / debug:
    python3 detect_gripper_marker.py --rgb ../captures_4c/g_servo_rgb.png --id 4 --len 0.05
    python3 detect_gripper_marker.py --rgb ../captures_4b/scene01_rgb.png --all   # dump every id
Prints one JSON line: {"found": true, "id": 4, "table_xy": [x, y], "z": z}
"""
from __future__ import annotations
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cv2  # noqa: E402
from ingest_real_scene import (  # noqa: E402
    detect_markers, marker_pose, homog, rgb_intrinsics, get_dictionary,
    BOARD_DIR, CALIB, MARKERS_JSON,
)


def _load_frame_context():
    ext = json.load(open(os.path.join(BOARD_DIR, "table_extrinsics.json")))
    calib = json.load(open(CALIB))
    socket = ext.get("rgb_socket", "CAM_B")
    # board frame IS the table frame: T_table<-rgb = inv(T_rgb<-board)
    T_rgb_board = homog(ext["rvec_board_in_rgb"], ext["tvec_board_in_rgb"])
    return calib, socket, np.linalg.inv(T_rgb_board)


def _markers_table_xy(rgb_path, marker_len, dict_name):
    """Return {id: (table_xy np.ndarray (2,), z float)} for every detected marker,
    or None if the image cannot be read."""
    calib, socket, T_table_rgb = _load_frame_context()
    rgb = cv2.imread(rgb_path)
    if rgb is None:
        return None
    h, w = rgb.shape[:2]
    K_rgb, dist_rgb = rgb_intrinsics(calib, socket, w, h)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    corners, ids = detect_markers(gray, get_dictionary(dict_name))
    out = {}
    if ids is None:
        return out
    for c, i in zip(corners, ids.flatten()):
        rvec, tvec = marker_pose(c, float(marker_len), K_rgb, dist_rgb)
        T = T_table_rgb @ homog(rvec, tvec)
        out[int(i)] = (T[:3, 3][:2].astype(float), float(T[2, 3]))
    return out


def detect_gripper_table_xy(rgb_path, marker_id=4, marker_len=0.05,
                            dict_name="DICT_4X4_50"):
    """Return (table_xy np.ndarray (2,), z, ok) for the gripper marker."""
    found = _markers_table_xy(rgb_path, marker_len, dict_name)
    if not found or int(marker_id) not in found:
        return None, None, False
    xy, z = found[int(marker_id)]
    return xy, z, True


def main():
    ap = argparse.ArgumentParser(description="Detect the gripper ArUco marker -> table XY")
    ap.add_argument("--rgb", required=True, help="path to the CAM_B RGB frame (..._rgb.png)")
    ap.add_argument("--id", type=int, default=4, help="gripper marker id (default 4)")
    ap.add_argument("--len", type=float, default=0.05, help="gripper marker side length (m)")
    ap.add_argument("--dict", default="DICT_4X4_50", help="ArUco dictionary name")
    ap.add_argument("--all", action="store_true",
                    help="debug: dump table XY of EVERY marker using the OBJECT markers.json "
                         "dictionary/length (sanity-checks the RGB->table reuse path on real data)")
    args = ap.parse_args()

    if args.all:
        mk = json.load(open(MARKERS_JSON))
        found = _markers_table_xy(args.rgb, float(mk["marker_length_m"]), mk["dictionary"])
        if found is None:
            print(json.dumps({"error": "could not read %s" % args.rgb})); sys.exit(1)
        dump = {str(i): {"table_xy": [round(float(xy[0]), 4), round(float(xy[1]), 4)],
                         "z": round(z, 4)} for i, (xy, z) in sorted(found.items())}
        print(json.dumps({"dict": mk["dictionary"], "marker_len": mk["marker_length_m"],
                          "markers": dump}, indent=2))
        return

    xy, z, ok = detect_gripper_table_xy(args.rgb, args.id, args.len, args.dict)
    if not ok:
        print(json.dumps({"found": False, "id": args.id})); sys.exit(1)
    print(json.dumps({"found": True, "id": args.id,
                      "table_xy": [round(float(xy[0]), 5), round(float(xy[1]), 5)],
                      "z": round(z, 5)}))


if __name__ == "__main__":
    main()
