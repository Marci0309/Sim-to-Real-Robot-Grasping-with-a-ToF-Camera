#!/usr/bin/env python3
"""Fit the table->arm extrinsic from logged touch data (config/arm_base_touch_raw.json).

Unlike the rigid Kabsch in solve_arm_base_extrinsics.py, this fits a 2D SIMILARITY
(scale + rotation + translation, Umeyama) with iterative outlier rejection. The
scale absorbs a systematic discrepancy discovered in Phase 4C: the real xArm 1S
reaches ~28% farther than its URDF/arm_ik model predicts (the model link lengths
do not match this arm), so the FK output is a uniformly-scaled copy of the true
geometry. Because the SAME FK/IK runs during a grasp, applying
``arm = scale * R @ table + t`` before IK still drives the real fingertip to the
true target (the model error cancels end to end).

Writes config/arm_base_extrinsics.json:
    {R_arm_from_table 3x3, t_arm_from_table 3, scale, residual_mm, n, n_dropped}

    python3 fit_arm_base_extrinsics.py            # uses arm_base_touch_raw.json
"""
import json
import os

import numpy as np

CFG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config"))
RAW = os.path.join(CFG, "arm_base_touch_raw.json")
OUT = os.path.join(CFG, "arm_base_extrinsics.json")
RMS_TARGET_MM = 8.0     # drop outliers until the fit reaches this (or n == 4)


def umeyama(A, B):
    """2D similarity aligning A->B:  B ~= s * R @ A + t.  Returns s, R(2x2), t, per-pt err."""
    cA, cB = A.mean(0), B.mean(0)
    X, Y = A - cA, B - cB
    H = Y.T @ X / len(A)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U @ Vt))
    R = U @ np.diag([1.0, d]) @ Vt
    s = (S * np.array([1.0, d])).sum() / ((X ** 2).sum() / len(A))
    t = cB - s * R @ cA
    err = np.linalg.norm(B - (s * (A @ R.T) + t), axis=1)
    return s, R, t, err


def main():
    raw = json.load(open(RAW))
    pts = raw["points"]
    T = np.array([p["table"] for p in pts])     # camera-true table XY (metric)
    M = np.array([p["fk"][:2] for p in pts])     # arm_ik FK XY (model)

    keep = list(range(len(pts)))
    dropped = []
    while True:
        s, R, t, err = umeyama(T[keep], M[keep])
        rms = float(np.sqrt((err ** 2).mean()))
        if rms * 1000 <= RMS_TARGET_MM or len(keep) <= 4:
            break
        w = keep[int(np.argmax(err))]
        dropped.append((w, float(err.max())))
        keep.remove(w)
        print(f"  dropped touch #{w+1} (residual {err.max()*1000:.1f} mm)")

    R3 = np.eye(3)
    R3[:2, :2] = R
    out = {
        "R_arm_from_table": R3.tolist(),
        "t_arm_from_table": [float(t[0]), float(t[1]), 0.0],
        "scale": float(s),
        "residual_mm": round(rms * 1000, 2),
        "n": len(keep), "n_dropped": len(dropped),
        "method": "touch points -> 2D similarity (Umeyama scale+R+t) w/ outlier rejection",
        "note": "arm = scale * R_arm_from_table @ [table_x, table_y, 0] + t_arm_from_table; "
                "scale absorbs the real-arm FK reach-scale error (URDF links != real links)",
    }
    json.dump(out, open(OUT, "w"), indent=2)
    print(f"\nscale = {s:.4f}   rotation = {np.degrees(np.arctan2(R[1,0], R[0,0])):.1f} deg")
    print(f"residual = {rms*1000:.1f} mm   n = {len(keep)} (dropped {len(dropped)})")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
