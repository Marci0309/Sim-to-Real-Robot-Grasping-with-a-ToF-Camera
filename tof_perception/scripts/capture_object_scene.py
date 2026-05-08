#!/usr/bin/env python3
"""
Capture N depth frames of a tabletop scene containing one (or more) objects.

Setup:
  - OAK-D SR PoE mounted top-down at the working height (~0.68 m above the
    table, matching the sim).
  - Robot arm parked OUTSIDE the camera FOV.
  - Object placed roughly under the camera center.
  - Scene static, stable lighting.

Pipeline matches capture_tof_calibration.py: bias-correction filters ON
(FPPN/optical/wiggle), temporal/spatial filters OFF, so we measure the
raw per-pixel temporal noise.

Usage:
    python3 capture_object_scene.py --name coke_can --n-frames 200 \\
        --table-distance 0.68 --out-dir ./object_captures

Output:
    object_captures/<name>.npy        # uint16 stack (N, H, W), depth in mm
    object_captures/<name>.json       # metadata (name, table distance, ...)
"""
import argparse
import json
import os
import time
from datetime import datetime

import cv2
import depthai as dai
import numpy as np

CAMERA_IP = '169.254.1.222'
WARMUP_FRAMES = 30


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', type=str, required=True,
                   help='object scene tag, e.g. coke_can / banana / golf_ball / blue_box / empty_table')
    p.add_argument('--table-distance', type=float, required=True,
                   help='measured camera-to-table distance in meters (used by analyzer to segment)')
    p.add_argument('--n-frames', type=int, default=200)
    p.add_argument('--out-dir', type=str, default='./object_captures')
    p.add_argument('--camera-ip', type=str, default=CAMERA_IP)
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    npy_path = os.path.join(args.out_dir, args.name + '.npy')
    json_path = os.path.join(args.out_dir, args.name + '.json')

    cmap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    cmap[0] = [0, 0, 0]

    print(f"Connecting to OAK-D SR PoE at {args.camera_ip}...")
    device = dai.Device(dai.DeviceInfo(args.camera_ip))
    print("Connected.")

    frames = []
    with dai.Pipeline(device) as pipeline:
        tof = pipeline.create(dai.node.ToF).build(dai.CameraBoardSocket.CAM_A)
        cfg = tof.getInitialConfig()
        cfg.enablePhaseShuffleTemporalFilter = False
        cfg.enableFPPNCorrection = True
        cfg.enableOpticalCorrection = True
        cfg.enableWiggleCorrection = True
        cfg.enableTemperatureCorrection = False
        cfg.enablePhaseUnwrapping = True
        cfg.phaseUnwrappingLevel = 4
        cfg.median = dai.MedianFilter.MEDIAN_OFF
        depth_q = tof.depth.createOutputQueue()
        pipeline.start()

        print(f"Warming up ({WARMUP_FRAMES} frames)...")
        warm = 0
        while pipeline.isRunning() and warm < WARMUP_FRAMES:
            if depth_q.tryGet() is not None:
                warm += 1

        print(f"Capturing {args.n_frames} frames of '{args.name}'.")
        print(f"Table distance: {args.table_distance:.3f} m. Press 'q' to abort.")
        t0 = time.time()
        max_disp = int(args.table_distance * 1000) + 200
        while pipeline.isRunning() and len(frames) < args.n_frames:
            f = depth_q.tryGet()
            if f is None:
                continue
            d = f.getFrame()
            frames.append(d.copy())

            vis = np.interp(d, (0, max_disp), (255, 0)).astype(np.uint8)
            vis = cv2.applyColorMap(vis, cmap)
            h, w = d.shape
            cv2.drawMarker(vis, (w // 2, h // 2), (255, 255, 255),
                           cv2.MARKER_CROSS, 20, 1)
            cx_mm = int(d[h // 2, w // 2])
            cv2.putText(vis, f"center: {cx_mm} mm  (table {int(args.table_distance*1000)} mm)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, f"frame {len(frames)}/{args.n_frames}  [{args.name}]",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("ToF object capture", vis)
            if cv2.waitKey(1) == ord('q'):
                print("Aborted by user.")
                cv2.destroyAllWindows()
                return

        cv2.destroyAllWindows()
        dt = time.time() - t0

    if len(frames) < args.n_frames:
        print(f"Only captured {len(frames)} frames. Aborting save.")
        return

    stack = np.stack(frames, axis=0).astype(np.uint16)
    np.save(npy_path, stack)
    meta = {
        'name': args.name,
        'table_distance_m': args.table_distance,
        'n_frames': int(stack.shape[0]),
        'height': int(stack.shape[1]),
        'width': int(stack.shape[2]),
        'depth_units': 'mm',
        'dtype': 'uint16',
        'camera_ip': args.camera_ip,
        'capture_seconds': round(dt, 2),
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'phase_shuffle_temporal_filter': False,
        'median_filter': 'OFF',
        'fppn_correction': True,
        'optical_correction': True,
        'wiggle_correction': True,
    }
    with open(json_path, 'w') as fh:
        json.dump(meta, fh, indent=2)

    print(f"Saved {npy_path}  shape={stack.shape}  ({dt:.1f} s)")
    print(f"Saved {json_path}")


if __name__ == '__main__':
    main()
