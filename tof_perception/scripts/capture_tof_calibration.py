#!/usr/bin/env python3
"""
Capture N depth frames from the OAK-D SR PoE ToF for noise calibration.

Protocol (Nguyen 2012, Khoshelham & Elberink 2012):
  - fronto-parallel matte planar target
  - fixed distance, ~200 frames
  - repeat at multiple distances spanning the working range (0.4-0.8 m)

Usage (Windows PowerShell):
    python capture_tof_calibration.py --distance 0.50 --n-frames 200 --out-dir .\captures

Output:
    <out-dir>/dist_0.50m.npy        # uint16 stack, shape (N, H, W), depth in mm
    <out-dir>/dist_0.50m.json       # metadata
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
    p.add_argument('--distance', type=float, required=True,
                   help='true distance from camera to target plane, in meters')
    p.add_argument('--n-frames', type=int, default=200,
                   help='number of frames to save (Nguyen 2012 used 100; we use 200)')
    p.add_argument('--out-dir', type=str, default='./captures')
    p.add_argument('--camera-ip', type=str, default=CAMERA_IP)
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    tag = f'dist_{args.distance:.2f}m'
    npy_path = os.path.join(args.out_dir, tag + '.npy')
    json_path = os.path.join(args.out_dir, tag + '.json')

    cmap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    cmap[0] = [0, 0, 0]

    print(f"Connecting to OAK-D SR PoE at {args.camera_ip}...")
    device = dai.Device(dai.DeviceInfo(args.camera_ip))
    print("Connected.")

    frames = []
    with dai.Pipeline(device) as pipeline:
        # Build a Camera + ToF graph manually (no ImageFiltersPreset) so the
        # filter config we set actually sticks. Presets like TOF_MID_RANGE
        # bake in temporal/spatial filtering that hides raw sensor noise.
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

        print(f"Capturing {args.n_frames} frames at distance {args.distance:.2f} m...")
        print("Verify aim in preview window. Press 'q' to abort.")
        t0 = time.time()
        while pipeline.isRunning() and len(frames) < args.n_frames:
            f = depth_q.tryGet()
            if f is None:
                continue
            d = f.getFrame()  # uint16, mm
            frames.append(d.copy())

            vis = np.interp(d, (0, 2000), (255, 0)).astype(np.uint8)
            vis = cv2.applyColorMap(vis, cmap)
            h, w = d.shape
            cv2.drawMarker(vis, (w // 2, h // 2), (255, 255, 255),
                           cv2.MARKER_CROSS, 20, 1)
            cx_mm = int(d[h // 2, w // 2])
            cv2.putText(vis, f"center: {cx_mm} mm  (target {int(args.distance*1000)} mm)",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, f"frame {len(frames)}/{args.n_frames}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("ToF calibration capture", vis)
            if cv2.waitKey(1) == ord('q'):
                print("Aborted by user.")
                cv2.destroyAllWindows()
                return

        cv2.destroyAllWindows()
        dt = time.time() - t0

    if len(frames) < args.n_frames:
        print(f"Only captured {len(frames)} frames (wanted {args.n_frames}). Aborting save.")
        return

    stack = np.stack(frames, axis=0).astype(np.uint16)
    np.save(npy_path, stack)
    meta = {
        'distance_m': args.distance,
        'n_frames': int(stack.shape[0]),
        'height': int(stack.shape[1]),
        'width': int(stack.shape[2]),
        'depth_units': 'mm',
        'dtype': 'uint16',
        'camera_ip': args.camera_ip,
        'capture_seconds': round(dt, 2),
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        'preset': 'TOF_MID_RANGE',
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
