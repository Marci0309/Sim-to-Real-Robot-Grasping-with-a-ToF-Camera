#!/usr/bin/env python3
"""
Phase 4B capture: ToF DEPTH (median over N frames, stable) + the ToF AMPLITUDE
image (averaged over N frames, for ArUco). Both are 640x480 and perfectly aligned
(same ToF sensor), so markers detected in the amplitude image index straight
into the depth map -- no RGB<->depth alignment needed.

Use it for BOTH:
  * step 0.3  -- put the ChArUco board flat on the table, capture -> solves the
                 camera->world extrinsics offline.
  * step 4B.2 -- place the 4 objects + their ArUco markers, capture -> perceived
                 positions (depth) + ground truth (markers), offline.

RUN IN WINDOWS (DepthAI is Windows-side here):
    copy to your DepthAI folder, then
    python capture_scene_4b.py --name board   --table-distance 0.68
    python capture_scene_4b.py --name scene01 --table-distance 0.68

Output (per <name>):
    <name>_depth.npy   uint16 (480,640) median depth in mm
    <name>_amp.npy     float32 (480,640) raw ToF amplitude
    <name>_amp.png     uint8 (480,640) normalised amplitude (what ArUco runs on)
    <name>.json        metadata (intrinsics ref, table distance, ...)

NB matches capture_object_scene.py's depthai-v3 ToF setup (CAM_A, bias filters
ON). The amplitude output attribute is `amplitude` on most v3 builds; this
script falls back to `intensity` if needed and prints which it used.
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
    p.add_argument('--name', required=True, help='output tag, e.g. board / scene01')
    p.add_argument('--table-distance', type=float, required=True,
                   help='measured camera-to-table distance in meters')
    p.add_argument('--n-frames', type=int, default=100)
    p.add_argument('--out-dir', default='./captures_4b')
    p.add_argument('--camera-ip', default=CAMERA_IP)
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    base = os.path.join(args.out_dir, args.name)

    print(f"Connecting to OAK-D at {args.camera_ip} ...")
    device = dai.Device(dai.DeviceInfo(args.camera_ip))
    print("Connected.")

    depth_frames, last_amp = [], None
    amp_sum, amp_count = None, 0
    amp_attr_used = None
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
        # amplitude/IR output (perfectly aligned with depth)
        amp_node = getattr(tof, 'amplitude', None)
        if amp_node is None:
            amp_node = getattr(tof, 'intensity', None)
            amp_attr_used = 'intensity' if amp_node is not None else None
        else:
            amp_attr_used = 'amplitude'
        amp_q = amp_node.createOutputQueue() if amp_node is not None else None
        print(f"amplitude output: {amp_attr_used or 'NONE FOUND -- depth only'}")
        pipeline.start()

        print(f"Warming up ({WARMUP_FRAMES} frames) ...")
        warm = 0
        while pipeline.isRunning() and warm < WARMUP_FRAMES:
            if depth_q.tryGet() is not None:
                warm += 1

        print(f"Capturing {args.n_frames} frames of '{args.name}'. Press 'q' to abort.")
        while pipeline.isRunning() and len(depth_frames) < args.n_frames:
            f = depth_q.tryGet()
            if f is None:
                continue
            depth_frames.append(f.getFrame().copy())
            if amp_q is not None:
                a = amp_q.tryGet()
                if a is not None:
                    last_amp = a.getFrame().copy()
                    af = last_amp.astype(np.float64)
                    amp_sum = af if amp_sum is None else amp_sum + af
                    amp_count += 1
            vis = np.interp(depth_frames[-1],
                            (0, int(args.table_distance * 1000) + 200),
                            (255, 0)).astype(np.uint8)
            cv2.putText(vis, f"frame {len(depth_frames)}/{args.n_frames} [{args.name}]",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 2)
            cv2.imshow("4B capture (depth)", vis)
            if last_amp is not None:
                amp_vis = cv2.normalize(last_amp, None, 0, 255, cv2.NORM_MINMAX)
                cv2.imshow("4B capture (amplitude -> ArUco)", amp_vis.astype(np.uint8))
            if cv2.waitKey(1) == ord('q'):
                print("Aborted."); cv2.destroyAllWindows(); return
        cv2.destroyAllWindows()

    if len(depth_frames) < args.n_frames:
        print(f"Only {len(depth_frames)} frames; aborting save."); return

    stack = np.stack(depth_frames, 0).astype(np.uint16)
    # median over time -> stable per-pixel depth (0 = invalid; median ignores via masking)
    med = np.median(np.where(stack == 0, np.nan, stack), axis=0)
    med = np.nan_to_num(med, nan=0.0).astype(np.uint16)
    np.save(base + '_depth.npy', med)
    if amp_count > 0:
        # Average amplitude over ALL frames (the board/scene is static): ~sqrt(N)
        # less noise than a single frame. A single ToF amplitude frame is too
        # noisy for ArUco/ChArUco to decode; the averaged image is what the
        # solver detects on.
        amp_mean = (amp_sum / amp_count).astype(np.float32)
        np.save(base + '_amp.npy', amp_mean)
        amp8 = cv2.normalize(amp_mean, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imwrite(base + '_amp.png', amp8)
        print(f"  amplitude averaged over {amp_count} frames")

    meta = {
        'name': args.name, 'table_distance_m': args.table_distance,
        'n_frames': int(stack.shape[0]), 'height': int(med.shape[0]),
        'width': int(med.shape[1]), 'depth_units': 'mm',
        'amplitude_attr': amp_attr_used, 'camera_ip': args.camera_ip,
        'tof_intrinsics_640x480': {'fx': 472.01025390625, 'fy': 471.7865295410156,
                                   'cx': 317.9997863769531, 'cy': 247.08209228515625},
        'timestamp': datetime.now().isoformat(timespec='seconds'),
    }
    with open(base + '.json', 'w') as fh:
        json.dump(meta, fh, indent=2)
    print(f"Saved {base}_depth.npy  {base}_amp.npy  {base}_amp.png  {base}.json")
    print("Send me <name>_amp.png and I'll test ArUco detection on it.")


if __name__ == '__main__':
    main()
