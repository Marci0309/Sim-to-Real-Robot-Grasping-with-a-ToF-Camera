#!/usr/bin/env python3
"""
Phase 4B / step 0.2 -- dump the OAK-D SR PoE factory calibration.

Reads, per connected camera socket: intrinsics (fx, fy, cx, cy + 3x3),
distortion coefficients, and sensor resolution; plus the extrinsics between
every camera pair (esp. ToF CAM_A <-> the color socket, needed to map
ArUco/colour detections into the depth frame for back-projection).

RUN IN WINDOWS (DepthAI is Windows-side in this project):
    copy this file to C:\\Users\\user\\thesis_ws\\...  then
    python dump_oakd_intrinsics.py --camera-ip 169.254.1.222 --out oakd_calib.json

Then send me oakd_calib.json (or paste the printout) and we finalise 4B.

depthai v3 API (matches capture_object_scene.py). If a call raises on your
depthai version the script keeps going and records the error per field, so the
output still tells us what the device exposes.
"""
import argparse
import json

import depthai as dai


def _to_list(m):
    return [[float(x) for x in row] for row in m]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--camera-ip', default='169.254.1.222')
    p.add_argument('--out', default='oakd_calib.json')
    # ToF depth capture resolution (from capture_object_scene.py: 640x480).
    p.add_argument('--depth-width', type=int, default=640)
    p.add_argument('--depth-height', type=int, default=480)
    args = p.parse_args()

    print(f"Connecting to OAK-D at {args.camera_ip} ...")
    device = dai.Device(dai.DeviceInfo(args.camera_ip))
    print("Connected.")
    calib = device.readCalibration()

    out = {'camera_ip': args.camera_ip, 'cameras': {}, 'extrinsics': {}}
    try:
        out['device_name'] = str(device.getDeviceName())
    except Exception as e:
        out['device_name_error'] = str(e)

    feats = device.getConnectedCameraFeatures()
    sockets = {}
    for f in feats:
        name = f.socket.name                      # e.g. 'CAM_A'
        sockets[name] = f.socket
        # native sensor resolution (largest supported config)
        try:
            cfgs = list(f.configs)
            best = max(cfgs, key=lambda c: c.width * c.height)
            w_nat, h_nat = int(best.width), int(best.height)
        except Exception:
            w_nat = getattr(f, 'width', None)
            h_nat = getattr(f, 'height', None)
        entry = {
            'socket': name,
            'sensorName': getattr(f, 'sensorName', ''),
            'native_width': w_nat,
            'native_height': h_nat,
            'supportedTypes': [str(t) for t in getattr(f, 'supportedTypes', [])],
        }
        # intrinsics at native res AND at the ToF depth res (640x480) so we have
        # exactly what back-projection needs without rescaling guesswork.
        for tag, (w, h) in (('native', (w_nat, h_nat)),
                            ('depth_res', (args.depth_width, args.depth_height))):
            if not w or not h:
                continue
            try:
                K = calib.getCameraIntrinsics(f.socket, w, h)
                entry[f'intrinsics_{tag}'] = {
                    'width': w, 'height': h, 'K': _to_list(K),
                    'fx': float(K[0][0]), 'fy': float(K[1][1]),
                    'cx': float(K[0][2]), 'cy': float(K[1][2]),
                }
            except Exception as e:
                entry[f'intrinsics_{tag}_error'] = str(e)
        try:
            entry['distortion'] = [float(x) for x in
                                   calib.getDistortionCoefficients(f.socket)]
        except Exception as e:
            entry['distortion_error'] = str(e)
        out['cameras'][name] = entry

    names = list(sockets)
    for a in names:
        for b in names:
            if a == b:
                continue
            try:
                E = calib.getCameraExtrinsics(sockets[a], sockets[b])
                out['extrinsics'][f'{a}->{b}'] = _to_list(E)
            except Exception as e:
                out['extrinsics'][f'{a}->{b}'] = {'error': str(e)}

    with open(args.out, 'w') as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps(out, indent=2))
    print(f"\nSaved -> {args.out}")


if __name__ == '__main__':
    main()
