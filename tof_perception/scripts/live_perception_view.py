#!/usr/bin/env python3
"""Live perception viewer (Phase 4C) -- watch how the depth segmentation "thinks".

Streams the overhead OAK-D ToF in real time and runs the EXACT same segmentation
the grasp pipeline uses (ingest_real_scene.perceived_blobs_table): table plane =
median depth, pixels >= OBJECT_HEIGHT_MIN_M above it are "object", connected
components -> one blob per object, median back-projected centroid -> table frame.

Two windows (both drawn over the ToF amplitude image so you see the real scene):
  (1) FILTER  -- red = the pixels classified as "object" (what the table-plane
                 subtraction keeps). This is "the filtering".
  (2) CENTERS -- a green X at each blob's perceived centroid (the point the grasp
                 aims at), labelled with its TABLE-frame XY in mm, plus a magenta
                 line = the PCA minor axis (the direction the jaws close across).

A short running median (N frames) stabilises the live depth; the real grasp uses a
100-frame median, so the live mask is a bit noisier than what a capture sees.

RUN ON WINDOWS (DepthAI is Windows-side):
    py -3.14 live_perception_view.py
    py -3.14 live_perception_view.py --median 12 --camera-ip 169.254.1.222
Press q (with a window focused) to quit.
"""
import argparse
import json
import os
import sys
from collections import deque

import warnings

import cv2
import depthai as dai
import numpy as np

warnings.filterwarnings("ignore", message="All-NaN slice encountered")  # empty-buffer pixels

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# Reuse the EXACT segmentation constants + ROI as the grasp perception, so the live
# view and the real grasp filter identically.
from ingest_real_scene import (OBJECT_HEIGHT_MIN_M, MIN_OBJECT_AREA_PX,   # noqa: E402
                               MIN_OBJECT_POINTS, roi_filter_above, ROI_TABLE_XY)

EXT = json.load(open(os.path.join(HERE, "..", "board_4b", "table_extrinsics.json")))
R_TC = np.array(EXT["R_table_from_camera"])
T_TC = np.array(EXT["t_table_from_camera"])
K = EXT["tof_intrinsics"]


def table_to_pixel(xy):
    """Project a table-frame (x, y, z=0) point to a ToF image pixel (for drawing)."""
    p = R_TC.T @ (np.array([xy[0], xy[1], 0.0]) - T_TC)
    if p[2] <= 1e-6:
        return None
    return (int(K["fx"] * p[0] / p[2] + K["cx"]), int(K["fy"] * p[1] / p[2] + K["cy"]))


def segment(depth_m, roi=None):
    """Return (object_mask, table_depth_m, [blob, ...]) with blob =
    (u, v, table_xyz, minor_axis_img, n_px). Mirrors perceived_blobs_table."""
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not valid.any():
        return None, 0.0, []
    table_depth = float(np.median(depth_m[valid]))
    above = valid & (depth_m < table_depth - OBJECT_HEIGHT_MIN_M)
    above = roi_filter_above(above, depth_m, K, R_TC, T_TC, roi)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(above.astype(np.uint8), 8)
    fx, fy, cx, cy = K["fx"], K["fy"], K["cx"], K["cy"]
    blobs = []
    for c in range(1, n):
        if stats[c, cv2.CC_STAT_AREA] < MIN_OBJECT_AREA_PX:
            continue
        ys, xs = np.where((lab == c) & valid)
        if len(xs) < MIN_OBJECT_POINTS:
            continue
        z = depth_m[ys, xs].astype(np.float64)
        X = (xs - cx) * z / fx
        Y = (ys - cy) * z / fy
        ctr_tab = np.median((R_TC @ np.stack([X, Y, z], 1).T).T + T_TC, 0)
        u, v = int(np.median(xs)), int(np.median(ys))
        pix = np.stack([xs, ys], 1).astype(np.float64)
        pix -= pix.mean(0)
        _, evec = np.linalg.eigh(pix.T @ pix / len(pix))   # ascending
        blobs.append((u, v, ctr_tab, evec[:, 0], len(xs)))   # evec[:,0] = short axis
    return above, table_depth, blobs


def main():
    ap = argparse.ArgumentParser(description="Live ToF perception viewer (Phase 4C)")
    ap.add_argument("--camera-ip", default="169.254.1.222")
    ap.add_argument("--median", type=int, default=15, help="running-median frames (stability)")
    ap.add_argument("--scale", type=float, default=1.4, help="display upscale factor")
    ap.add_argument("--no-roi", action="store_true", help="disable the reachable-table ROI")
    ap.add_argument("--name", default="live_grasp",
                    help="scene name for the depth frame saved with the 'S' key (for a grasp)")
    args = ap.parse_args()
    roi = None if args.no_roi else ROI_TABLE_XY
    save_dir = os.path.join(HERE, "..", "captures_4c")

    print(f"Connecting to OAK-D at {args.camera_ip} ...")
    device = dai.Device(dai.DeviceInfo(args.camera_ip))
    print("Connected. Live view -- press S to save this depth for a grasp, Q to quit.")
    buf = deque(maxlen=max(1, args.median))

    with dai.Pipeline(device) as pipe:
        tof = pipe.create(dai.node.ToF).build(dai.CameraBoardSocket.CAM_A)
        cfg = tof.getInitialConfig()
        cfg.enableFPPNCorrection = True
        cfg.enableOpticalCorrection = True
        cfg.enableWiggleCorrection = True
        cfg.enablePhaseUnwrapping = True
        cfg.phaseUnwrappingLevel = 4
        cfg.median = dai.MedianFilter.MEDIAN_OFF
        depth_q = tof.depth.createOutputQueue()
        amp_node = getattr(tof, "amplitude", None) or getattr(tof, "intensity", None)
        amp_q = amp_node.createOutputQueue() if amp_node is not None else None
        pipe.start()

        last_amp = None
        while pipe.isRunning():
            f = depth_q.tryGet()
            if f is None:
                continue
            buf.append(f.getFrame())
            if amp_q is not None:
                a = amp_q.tryGet()
                if a is not None:
                    last_amp = a.getFrame()

            stack = np.stack(buf, 0).astype(np.float32)
            depth_m = np.nan_to_num(np.nanmedian(np.where(stack == 0, np.nan, stack), 0)) / 1000.0
            above, td, blobs = segment(depth_m, roi)

            if last_amp is not None:
                g = cv2.normalize(last_amp, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                base = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            else:
                d8 = np.interp(depth_m, (0, (td or 1) + 0.2), (255, 0)).astype(np.uint8)
                base = cv2.applyColorMap(d8, cv2.COLORMAP_JET)

            # (1) FILTER
            v1 = base.copy()
            if above is not None:
                v1[above] = (0, 0, 255)
            cv2.putText(v1, f"table={td*1000:.0f}mm  object=>{OBJECT_HEIGHT_MIN_M*1000:.0f}mm above  blobs:{len(blobs)}",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(v1, "[S] save depth for a grasp   [Q] quit", (8, base.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # (2) CENTERS
            v2 = base.copy()
            for u, v, ctr, minor, npx in blobs:
                p1 = (int(u + minor[0] * 28), int(v + minor[1] * 28))
                p2 = (int(u - minor[0] * 28), int(v - minor[1] * 28))
                cv2.line(v2, p1, p2, (255, 0, 255), 2)               # jaw-close axis
                cv2.drawMarker(v2, (u, v), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 20, 2)
                cv2.putText(v2, f"({ctr[0]*1000:.0f},{ctr[1]*1000:.0f})mm",
                            (u + 10, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
            cv2.putText(v2, "green X = perceived center (table mm)  |  magenta = jaw-close axis",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            if roi is not None:   # draw the reachable-table ROI boundary (cyan)
                corners = [table_to_pixel((roi[0], roi[2])), table_to_pixel((roi[1], roi[2])),
                           table_to_pixel((roi[1], roi[3])), table_to_pixel((roi[0], roi[3]))]
                if all(c is not None for c in corners):
                    poly = np.array(corners, np.int32).reshape(-1, 1, 2)
                    cv2.polylines(v1, [poly], True, (255, 255, 0), 1)
                    cv2.polylines(v2, [poly], True, (255, 255, 0), 1)

            if args.scale != 1.0:
                v1 = cv2.resize(v1, None, fx=args.scale, fy=args.scale)
                v2 = cv2.resize(v2, None, fx=args.scale, fy=args.scale)
            cv2.imshow("1) FILTER  (red = segmented object pixels)", v1)
            cv2.imshow("2) CENTERS  (X = where the grasp aims)", v2)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                os.makedirs(save_dir, exist_ok=True)
                np.save(os.path.join(save_dir, f"{args.name}_depth.npy"),
                        (np.nan_to_num(depth_m) * 1000.0).astype(np.uint16))
                print(f"[saved] {args.name}_depth.npy  ({len(blobs)} blobs). In another terminal:\n"
                      f"   (WSL)    python3 perceive_scene_4c.py --name {args.name} --object <OBJ>\n"
                      f"   (py-3.11) real_grasp_executor.py --object <OBJ> --scene {args.name} --pick-only --close-mm 15")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
