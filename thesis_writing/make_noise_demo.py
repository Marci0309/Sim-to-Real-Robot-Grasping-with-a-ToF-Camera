#!/usr/bin/env python3
"""Clean-vs-noisy depth demo for the presentation.

A synthetic CLEAN top-down tabletop depth frame (table @ 0.68 m + the four
objects), then the EXACT measured-OAK-D noise model applied (the same four-term
pipeline as noise_injection/depth_noise_injector._on_depth, with the
oakd_measured.yaml parameters). Shows clean / +noise(x1) / +noise(x10) so the
edge flying-pixels — the real pose-relevant noise — are visible.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.expanduser("~/thesis_ws/src")
OUT = os.path.join(BASE, "evaluation", "results", "clean_vs_noisy_depth.png")

H, W = 360, 560
TABLE = 0.68                      # table distance from the overhead camera (m)
clean = np.full((H, W), TABLE, np.float32)
yy, xx = np.mgrid[0:H, 0:W]


def disk(cy, cx, r):
    return (yy - cy) ** 2 + (xx - cx) ** 2 <= r * r


def rect(cy, cx, hy, hx):
    return (np.abs(yy - cy) <= hy) & (np.abs(xx - cx) <= hx)


def ellipse(cy, cx, ry, rx):
    return ((yy - cy) / ry) ** 2 + ((xx - cx) / rx) ** 2 <= 1.0


# objects raise the surface toward the camera -> smaller depth (height in m)
clean[rect(110, 150, 34, 44)]    = TABLE - 0.0285   # blue_box  (sharp faces)
clean[ellipse(110, 410, 26, 70)] = TABLE - 0.052    # coke_can  (lying cylinder)
clean[disk(265, 175, 30)]        = TABLE - 0.043    # golf_ball (sphere)
clean[ellipse(255, 405, 22, 64)] = TABLE - 0.026    # banana    (elongated)


def inject(depth, axial_scale=1.0, seed=42):
    """Faithful copy of depth_noise_injector._on_depth with OAK-D params."""
    a, b, z0 = 0.0009, 0.0, 0.4          # oakd_measured.yaml: axial (b=0, flat)
    lat_sigma = 0.8                       # lateral jitter (px)
    edge_tau, p_edge = 0.02, 0.70         # edge mixed/flying pixels
    p_drop = 0.0                          # OAK-D: no dropout
    ss = np.random.SeedSequence(seed); s1, s2 = ss.spawn(2)
    rng_g, rng_m = np.random.default_rng(s1), np.random.default_rng(s2)

    valid = np.isfinite(depth) & (depth > 0)
    out = depth.copy()
    # 1) axial Gaussian
    sigma_z = np.clip(a + b * (depth - z0) ** 2, 0, None) * axial_scale
    out = out + rng_g.standard_normal(depth.shape).astype(np.float32) * sigma_z.astype(np.float32) * valid
    # 2) lateral jitter
    h, w = depth.shape
    vv, uu = np.mgrid[0:h, 0:w].astype(np.int32)
    du = rng_g.normal(0, lat_sigma, depth.shape).astype(np.int32)
    dv = rng_g.normal(0, lat_sigma, depth.shape).astype(np.int32)
    out = out[np.clip(vv + dv, 0, h - 1), np.clip(uu + du, 0, w - 1)]
    # 3) edge / flying pixels
    gx = np.abs(np.gradient(depth, axis=1)); gy = np.abs(np.gradient(depth, axis=0))
    sel = ((gx + gy) > edge_tau) & (rng_m.random(depth.shape) < p_edge) & valid
    if np.any(sel):
        sh = np.stack([np.roll(depth, 1, 0), np.roll(depth, -1, 0),
                       np.roll(depth, 1, 1), np.roll(depth, -1, 1), depth], 0)
        alpha = rng_m.random(depth.shape).astype(np.float32)
        mixed = alpha * sh.min(0) + (1 - alpha) * sh.max(0)
        out = np.where(sel, mixed, out)
    # 4) dropout
    if p_drop > 0:
        out[rng_m.random(depth.shape) < p_drop] = np.nan
    return out


noisy1 = inject(clean, 1.0)
noisy10 = inject(clean, 10.0)

vmin, vmax = TABLE - 0.058, TABLE + 0.004
panels = [("Clean simulated depth", clean),
          ("+ measured OAK-D noise  (×1)", noisy1),
          ("+ noise ×10  (robustness threshold)", noisy10)]
fig, axes = plt.subplots(1, 3, figsize=(15, 3.6), dpi=130)
for ax, (ttl, img) in zip(axes, panels):
    im = ax.imshow(img, cmap="turbo", vmin=vmin, vmax=vmax)
    ax.set_title(ttl, fontsize=13, fontweight="bold"); ax.axis("off")
fig.suptitle("ToF depth-noise model: realistic (×1) noise barely perturbs the body; "
             "edge flying-pixels appear at object boundaries", fontsize=12, y=1.02)
cb = fig.colorbar(im, ax=axes, fraction=0.025, pad=0.01); cb.set_label("depth (m)")
fig.savefig(OUT, dpi=130, bbox_inches="tight")
print("wrote", OUT)
