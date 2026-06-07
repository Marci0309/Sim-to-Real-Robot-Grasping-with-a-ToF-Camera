#!/usr/bin/env python3
"""
Phase 4B / step 3 (solve) -- recover the camera(ToF)->table extrinsic.

Detection runs on the SHARP RGB camera (CAM_B), not the soft ToF amplitude
(which can't decode the fine markers). The board pose is solved in the RGB
frame, then mapped into the ToF/depth (CAM_A) frame via the factory
CAM_A<->CAM_B extrinsic in oakd_calib.json. Two independent sanity checks
(RGB reprojection error, and ToF-depth table-plane height) confirm the result.

Inputs:
  captures_4b/<name>_rgb.png    sharp colour image from CAM_B (capture_rgb_4b.py)
  captures_4b/<name>_depth.npy  ToF median depth, mm (table-plane cross-check)
  captures_4b/<name>.json       ToF metadata (table_distance_m, tof intrinsics)
  board_4b/board_params.json    board definition (measured square size!)
  oakd_calib.json (repo root)   CAM_B intrinsics+distortion, CAM_A->CAM_B extrinsic

Outputs:
  board_4b/table_extrinsics.json   R/t (ToF camera -> table) + checks  <- step 7 reads
  board_4b/extrinsics_overlay.png  RGB with detected board + solved axes

Frames: estimatePoseCharucoBoard gives T_RGB<-board (board == table frame).
DepthAI 'CAM_A->CAM_B' maps a ToF point into the RGB frame (T_RGB<-ToF). So
  T_table<-ToF = inv(T_RGB<-board) @ T_RGB<-ToF.
DepthAI extrinsic translation is in CENTIMETRES -> /100.

Run (WSL): python3 solve_table_extrinsics.py --name board
"""
import argparse
import json
import os

import cv2
import numpy as np

HERE      = os.path.dirname(os.path.abspath(__file__))
PKG       = os.path.normpath(os.path.join(HERE, ".."))
BOARD_DIR = os.path.join(PKG, "board_4b")
CALIB     = os.path.normpath(os.path.join(PKG, "..", "..", "oakd_calib.json"))


def get_dictionary(name):
    code = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(code)
    return cv2.aruco.Dictionary_get(code)


def make_board(p, d):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            p["squares_x"], p["squares_y"], p["square_length_m"], p["marker_length_m"], d)
    return cv2.aruco.CharucoBoard(
        (p["squares_x"], p["squares_y"]), p["square_length_m"], p["marker_length_m"], d)


def fit_plane_ransac(pts, thresh=0.005, iters=300):
    best_in, best = None, None
    n = len(pts)
    rs = np.random.RandomState(0)
    for _ in range(iters):
        i = rs.randint(0, n, 3)
        a, b, c = pts[i]
        nrm = np.cross(b - a, c - a)
        m = np.linalg.norm(nrm)
        if m < 1e-9:
            continue
        nrm = nrm / m
        dd = np.abs((pts - a) @ nrm)
        inl = dd < thresh
        if best_in is None or inl.sum() > best_in.sum():
            best_in, best = inl, (nrm, a)
    P = pts[best_in]
    c = P.mean(0)
    _, _, vh = np.linalg.svd(P - c)
    return vh[2] / np.linalg.norm(vh[2]), c


def homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def pose_charuco(ch_c, ch_id, board, K, dist):
    """Solve board pose; estimatePoseCharucoBoard if present, else solvePnP."""
    if hasattr(cv2.aruco, "estimatePoseCharucoBoard"):
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_c, ch_id, board, K, dist, None, None)
        return ok, rvec, tvec
    obj = board.chessboardCorners[ch_id.flatten()]
    ok, rvec, tvec = cv2.solvePnP(obj, ch_c.reshape(-1, 2), K, dist)
    return ok, rvec, tvec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="board")
    ap.add_argument("--captures-dir", default=os.path.join(PKG, "captures_4b"))
    ap.add_argument("--rgb-socket", default="CAM_B")
    ap.add_argument("--axis-len", type=float, default=0.15)
    args = ap.parse_args()

    params = json.load(open(os.path.join(BOARD_DIR, "board_params.json")))
    cap = os.path.join(args.captures_dir, args.name)
    calib = json.load(open(CALIB))

    # --- RGB image + CAM_B intrinsics/distortion (scaled to captured res) -----
    rgb = cv2.imread(cap + "_rgb.png")
    if rgb is None:
        raise SystemExit("missing %s_rgb.png -- run capture_rgb_4b.py first" % cap)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    H, W = gray.shape

    camB = calib["cameras"][args.rgb_socket]
    nat = camB["intrinsics_native"]
    sx, sy = W / nat["width"], H / nat["height"]
    K_rgb = np.array([[nat["fx"] * sx, 0, nat["cx"] * sx],
                      [0, nat["fy"] * sy, nat["cy"] * sy],
                      [0, 0, 1.0]])
    dist_rgb = np.array(camB.get("distortion", []), dtype=np.float64).ravel()
    if dist_rgb.size == 0:
        dist_rgb = np.zeros(5)

    d = get_dictionary(params["dictionary"])
    board = make_board(params, d)

    corners, ids, _ = cv2.aruco.detectMarkers(gray, d)
    if ids is None or len(ids) == 0:                 # CLAHE fallback (rarely needed)
        g2 = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        corners, ids, _ = cv2.aruco.detectMarkers(g2, d)
        if ids is not None:
            gray = g2
    if ids is None or len(ids) == 0:
        raise SystemExit("No markers in RGB -- check board fully in frame / focus.")
    _, ch_c, ch_id = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    n_ch = 0 if ch_id is None else len(ch_id)
    if n_ch < 6:
        raise SystemExit("Only %d ChArUco corners (need >=6)." % n_ch)

    ok, rvec, tvec = pose_charuco(ch_c, ch_id, board, K_rgb, dist_rgb)
    if not ok:
        raise SystemExit("pose estimation failed.")
    R_rb, _ = cv2.Rodrigues(rvec)
    T_rgb_board = homog(R_rb, tvec)

    obj = board.chessboardCorners[ch_id.flatten()]
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K_rgb, dist_rgb)
    reproj = float(np.sqrt(((proj.reshape(-1, 2) - ch_c.reshape(-1, 2)) ** 2).sum(1).mean()))

    # --- factory extrinsic CAM_A(ToF) -> CAM_B(RGB), cm -> m ------------------
    E = np.array(calib["extrinsics"]["CAM_A->" + args.rgb_socket], dtype=np.float64)
    E[:3, 3] /= 100.0                      # depthai translation is in centimetres
    T_rgb_tof = E                           # maps a ToF point into the RGB frame

    T_table_tof = np.linalg.inv(T_rgb_board) @ T_rgb_tof
    R_tc = T_table_tof[:3, :3]
    t_tc = T_table_tof[:3, 3]
    cam_height = float(abs(t_tc[2]))        # ToF camera height above the table

    # --- cross-check: table-plane height straight from the ToF depth map ------
    meta = json.load(open(cap + ".json"))
    K_d = meta["tof_intrinsics_640x480"]
    depth_height = None
    if os.path.exists(cap + "_depth.npy"):
        Z = np.load(cap + "_depth.npy").astype(np.float32) / 1000.0
        v, u = np.nonzero(Z > 0)
        Zf = Z[v, u]
        X = (u - K_d["cx"]) * Zf / K_d["fx"]
        Y = (v - K_d["cy"]) * Zf / K_d["fy"]
        pts = np.stack([X, Y, Zf], 1)
        if len(pts) > 8000:
            pts = pts[np.linspace(0, len(pts) - 1, 8000).astype(int)]
        nrm, p0 = fit_plane_ransac(pts)
        depth_height = float(abs(nrm @ p0))

    table_dist = meta.get("table_distance_m")

    out = {
        "name": args.name,
        "detection": "RGB %s -> composed into ToF/CAM_A frame" % args.rgb_socket,
        "R_table_from_camera": R_tc.tolist(),     # camera == ToF / depth camera
        "t_table_from_camera": t_tc.tolist(),
        "rvec_board_in_rgb": np.asarray(rvec).reshape(3).tolist(),
        "tvec_board_in_rgb": np.asarray(tvec).reshape(3).tolist(),
        "tof_intrinsics": K_d,
        "rgb_socket": args.rgb_socket,
        "n_charuco_corners": int(n_ch),
        "reproj_err_px": reproj,
        "camera_height_extrinsic_m": cam_height,
        "camera_height_depthplane_m": depth_height,
        "measured_table_distance_m": table_dist,
        "square_length_m_used": params["square_length_m"],
        "note": "p_table = R_table_from_camera @ p_tof + t_table_from_camera",
    }
    with open(os.path.join(BOARD_DIR, "table_extrinsics.json"), "w") as fh:
        json.dump(out, fh, indent=2)

    # --- overlay (on RGB) -----------------------------------------------------
    vis = rgb.copy()
    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
    if ch_c is not None:
        cv2.aruco.drawDetectedCornersCharuco(vis, ch_c, ch_id, (0, 0, 255))
    try:
        cv2.drawFrameAxes(vis, K_rgb, dist_rgb, rvec, tvec, args.axis_len, 3)
    except Exception:
        cv2.aruco.drawAxis(vis, K_rgb, dist_rgb, rvec, tvec, args.axis_len)
    cv2.imwrite(os.path.join(BOARD_DIR, "extrinsics_overlay.png"), vis)

    # --- report ---------------------------------------------------------------
    print("OpenCV", cv2.__version__)
    print("detected on RGB %s : %d markers, %d/%d ChArUco corners"
          % (args.rgb_socket, len(ids), n_ch, params["interior_corners"]))
    print("RGB reprojection err : %.3f px   (want < ~1.0)" % reproj)
    print("camera height (extr) : %.4f m" % cam_height)
    if depth_height is not None:
        print("camera height (depth): %.4f m   (independent ToF cross-check)" % depth_height)
        print("  extr vs depth diff : %.1f mm" % (abs(cam_height - depth_height) * 1000))
    if table_dist is not None:
        print("measured mount dist  : %.4f m" % table_dist)
    print("ToF camera in table frame (m): [%.3f, %.3f, %.3f]" % (t_tc[0], t_tc[1], t_tc[2]))
    print("\nwrote board_4b/table_extrinsics.json + extrinsics_overlay.png")

    ref = depth_height if depth_height is not None else table_dist
    bad = (reproj > 2.0) or (ref is not None and abs(cam_height - ref) > 0.02)
    print("[%s] extrinsic looks %s"
          % ("WARN" if bad else "OK", "SUSPECT -- recheck" if bad else "good"))


if __name__ == "__main__":
    main()
