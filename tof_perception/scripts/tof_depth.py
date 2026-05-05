#!/usr/bin/env python3
"""
ToF depth-only stream from OAK-D SR PoE — depthai v3
Run from Windows PowerShell:
    python tof_depth.py

Controls:
    Q = quit
    1-5 = change phase unwrapping level (range)
    + / - = increase / decrease max depth scale
"""
import cv2
import depthai as dai
import numpy as np

CAMERA_IP = '169.254.1.222'
MAX_DEPTH_MM = 6000

cvColorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
cvColorMap[0] = [0, 0, 0]

print(f"Connecting to OAK-D SR PoE at {CAMERA_IP}...")
device = dai.Device(dai.DeviceInfo(CAMERA_IP))
print("Connected!")

with dai.Pipeline(device) as pipeline:
    tof = pipeline.create(dai.node.ToF).build(
        dai.CameraBoardSocket.CAM_A,
        dai.ImageFiltersPresetMode.TOF_MID_RANGE
    )
    depth_q = tof.depth.createOutputQueue()
    pipeline.start()

    print("Streaming ToF depth!")
    print("Controls: Q=quit | 1-5=range level | +/-=scale depth")

    max_depth = MAX_DEPTH_MM
    frame_count = 0

    while pipeline.isRunning():
        frame = depth_q.tryGet()

        if frame is not None:
            depth = frame.getFrame()
            frame_count += 1

            # Colorize: red=close, blue=far
            vis = np.interp(depth, (0, max_depth), (255, 0)).astype(np.uint8)
            colored = cv2.applyColorMap(vis, cvColorMap)

            # Overlay stats
            h, w = depth.shape
            center_d = depth[h//2, w//2]
            min_d = int(np.min(depth[depth > 0])) if np.any(depth > 0) else 0
            max_d = int(np.max(depth))

            cv2.putText(colored, f"Center: {center_d}mm",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(colored, f"Min: {min_d}mm  Max: {max_d}mm",
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
            cv2.putText(colored, f"Scale: 0-{max_depth}mm | Frame: {frame_count}",
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,180,180), 1)

            # Crosshair at center
            cv2.drawMarker(colored, (w//2, h//2),
                (255,255,255), cv2.MARKER_CROSS, 20, 1)

            cv2.imshow("ToF Depth - OAK-D SR PoE", colored)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
            level = key - ord('0')
            max_depth = (level + 1) * 1500
            print(f"Phase unwrapping level {level} — max range ~{max_depth}mm")
        elif key == ord('+') or key == ord('='):
            max_depth = min(max_depth + 500, 10000)
            print(f"Max depth scale: {max_depth}mm")
        elif key == ord('-'):
            max_depth = max(max_depth - 500, 500)
            print(f"Max depth scale: {max_depth}mm")

    cv2.destroyAllWindows()
    print(f"Done. {frame_count} frames captured.")