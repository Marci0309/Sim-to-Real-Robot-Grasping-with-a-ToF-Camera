#!/usr/bin/env python3
"""
Phase 4B RGB capture (CAM_B colour) -- for reliable ArUco / ChArUco detection.

The ToF AMPLITUDE image is too low-resolution to decode fine ArUco markers
(~7 px/bit at 0.68 m, and the ToF optics are soft). The OAK-D's colour camera
CAM_B (OV9782, 1280x800 global shutter) is SHARP, so we detect the board (and
later the object ground-truth markers) there, then map the result into the depth
(ToF / CAM_A) frame using the factory CAM_A<->CAM_B extrinsic in oakd_calib.json.

Pairs with the existing ToF capture:
  * capture_scene_4b.py  -> <name>_depth.npy   (perceived geometry, ToF frame)
  * capture_rgb_4b.py    -> <name>_rgb.png     (fiducials, CAM_B frame)
Both view the same static scene from the fixed mount, so they are captured
separately (no need to be simultaneous).

RUN IN WINDOWS (DepthAI is Windows-side here):
    python capture_rgb_4b.py --name board --out-dir ..\\captures_4b

Output (per <name>):
    <name>_rgb.png    uint8 BGR, averaged over N frames (sharp, low-noise)
    <name>_rgb.json   metadata (socket, resolution, N frames)

NB: if your depthai build rejects the Camera node call below, paste the error --
the v3 Camera API has minor variants and it's a one-line fix.
"""
import argparse
import json
import os
from datetime import datetime

import cv2
import depthai as dai
import numpy as np

CAMERA_IP = '169.254.1.222'
WARMUP_FRAMES = 15


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', required=True, help='output tag, e.g. board / scene01')
    p.add_argument('--n-frames', type=int, default=30,
                   help='frames to average (RGB is sharp; 30 is plenty)')
    p.add_argument('--out-dir', default='./captures_4b')
    p.add_argument('--camera-ip', default=CAMERA_IP)
    p.add_argument('--socket', default='CAM_B', choices=['CAM_B', 'CAM_C'],
                   help='colour socket to use (CAM_B default; CAM_C is the other)')
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    base = os.path.join(args.out_dir, args.name)
    socket = getattr(dai.CameraBoardSocket, args.socket)

    print(f"Connecting to OAK-D at {args.camera_ip} ...")
    device = dai.Device(dai.DeviceInfo(args.camera_ip))
    print("Connected.")

    acc, count, last = None, 0, None
    with dai.Pipeline(device) as pipeline:
        cam = pipeline.create(dai.node.Camera).build(socket)
        # Native OV9782 resolution -> biggest, sharpest markers.
        rgb_out = cam.requestOutput((1280, 800), dai.ImgFrame.Type.BGR888i)
        rgb_q = rgb_out.createOutputQueue()
        pipeline.start()

        print(f"Warming up ({WARMUP_FRAMES} frames) ...")
        warm = 0
        while pipeline.isRunning() and warm < WARMUP_FRAMES:
            if rgb_q.tryGet() is not None:
                warm += 1

        print(f"Capturing {args.n_frames} frames of '{args.name}' from {args.socket}. "
              f"Press 'q' to abort.")
        while pipeline.isRunning() and count < args.n_frames:
            f = rgb_q.tryGet()
            if f is None:
                continue
            img = f.getCvFrame()                 # BGR uint8 (H, W, 3)
            last = img
            acc = img.astype(np.float64) if acc is None else acc + img
            count += 1
            h, w = img.shape[:2]
            vis = cv2.resize(img, (640, int(640 * h / w)))
            cv2.putText(vis, f"frame {count}/{args.n_frames} [{args.name}]",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("4B RGB capture (CAM_B -> ArUco)", vis)
            if cv2.waitKey(1) == ord('q'):
                print("Aborted."); cv2.destroyAllWindows(); return
        cv2.destroyAllWindows()

    if count == 0:
        print("No RGB frames captured -- aborting save."); return

    mean = (acc / count).round().astype(np.uint8)
    cv2.imwrite(base + '_rgb.png', mean)
    meta = {
        'name': args.name, 'socket': args.socket,
        'width': int(mean.shape[1]), 'height': int(mean.shape[0]),
        'n_frames': int(count), 'camera_ip': args.camera_ip,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
    }
    with open(base + '_rgb.json', 'w') as fh:
        json.dump(meta, fh, indent=2)
    print(f"  averaged {count} frames -> {mean.shape[1]}x{mean.shape[0]}")
    print(f"Saved {base}_rgb.png  {base}_rgb.json")
    print("Next (WSL): python3 solve_table_extrinsics.py --name board")


if __name__ == '__main__':
    main()
