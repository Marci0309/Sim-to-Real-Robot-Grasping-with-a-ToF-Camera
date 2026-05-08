#!/usr/bin/env python3
"""
Analyze object-scene depth captures from capture_object_scene.py.

For each scene, segment object pixels (depth significantly above table) and
report the noise statistics that actually matter for tabletop pose estimation:

  - Body axial noise:   median temporal sigma over object-body pixels (mm)
  - Body bias:          median (depth - table_estimate) over object body
  - Dropout rate:       fraction of object pixels with >50% invalid frames
  - Edge flying-pixels: fraction of edge-ring pixels with depth strictly
                        between object and table planes (mixed pixels per
                        Foix 2011 / Sarbolandi 2015)

Usage:
    python3 analyze_object_noise.py --in-dir ./object_captures \\
        --out-dir ./object_results

Output:
    object_results/object_noise.json   # per-scene stats
    object_results/<name>_overlay.png  # diagnostic overlay (object mask + edge ring)
"""
import argparse
import glob
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import cv2

OBJECT_HEIGHT_MIN_MM = 15    # object must rise at least this much above the table
                             # (raised from 5 to reject ~5–10 mm spatial sensor bias)
INVALID_FRACTION_DROPOUT = 0.5
EDGE_DILATE_PX = 6           # ring width around object silhouette
ROI_FRAC = 0.6               # central fraction of FOV to search for object;
                             # corners often have housing/edge artifacts
MIN_OBJECT_AREA_PX = 100     # smaller blobs are noise specks


def per_pixel_stats(stack):
    s = stack.astype(np.float32)
    valid = s > 0
    n = stack.shape[0]
    n_valid = valid.sum(axis=0)
    invalid_frac = 1.0 - (n_valid / n)
    s_masked = np.where(valid, s, np.nan)
    with np.errstate(invalid='ignore'):
        mean = np.nanmean(s_masked, axis=0)
        std = np.nanstd(s_masked, axis=0)
    return mean, std, invalid_frac


def estimate_table_mm(mean_mm, table_distance_m):
    """Robust table-plane depth from a histogram peak near the nominal value."""
    target = table_distance_m * 1000.0
    band = (mean_mm > target - 50) & (mean_mm < target + 50) & np.isfinite(mean_mm)
    if band.sum() < 1000:
        return target
    return float(np.median(mean_mm[band]))


def analyze_one(npy_path):
    json_path = npy_path.replace('.npy', '.json')
    stack = np.load(npy_path)
    with open(json_path) as fh:
        meta = json.load(fh)
    table_d_m = meta['table_distance_m']
    name = meta['name']

    mean_mm, std_mm, invalid_frac = per_pixel_stats(stack)
    table_mm = estimate_table_mm(mean_mm, table_d_m)

    # Object body = pixels noticeably above the table plane (i.e. closer to
    # the camera). Top-down geometry: smaller depth = higher off table.
    H, W = mean_mm.shape
    y0, y1 = int(H * (0.5 - ROI_FRAC / 2)), int(H * (0.5 + ROI_FRAC / 2))
    x0, x1 = int(W * (0.5 - ROI_FRAC / 2)), int(W * (0.5 + ROI_FRAC / 2))
    roi_mask = np.zeros((H, W), dtype=bool)
    roi_mask[y0:y1, x0:x1] = True

    raw_above = (mean_mm < table_mm - OBJECT_HEIGHT_MIN_MM) & np.isfinite(mean_mm)
    object_mask = raw_above & roi_mask

    # Pick the connected component closest to image center (objects are placed
    # under the camera; nearest-to-center beats largest when artifacts dominate).
    n_lab, lab, cc_stats, centroids = cv2.connectedComponentsWithStats(
        object_mask.astype(np.uint8), 8)
    cy_img, cx_img = H / 2.0, W / 2.0
    best, best_d = None, float('inf')
    for i in range(1, n_lab):
        if cc_stats[i, cv2.CC_STAT_AREA] < MIN_OBJECT_AREA_PX:
            continue
        d2 = (centroids[i, 0] - cx_img) ** 2 + (centroids[i, 1] - cy_img) ** 2
        if d2 < best_d:
            best_d, best = d2, i
    if best is not None:
        object_mask = (lab == best)
    else:
        object_mask = np.zeros((H, W), dtype=bool)

    if object_mask.sum() < MIN_OBJECT_AREA_PX:
        print(f"  [{name}] WARNING: no object found in central ROI "
              f"(this is expected for empty_table; otherwise check placement / "
              f"--table-distance / OBJECT_HEIGHT_MIN_MM)")

    # Body stats over object-mask pixels with valid temporal data.
    body_valid = object_mask & np.isfinite(std_mm) & (std_mm > 0)
    body_sigma_mm = float(np.median(std_mm[body_valid])) if body_valid.any() else float('nan')
    body_depth_mm = float(np.median(mean_mm[body_valid])) if body_valid.any() else float('nan')
    body_height_mm = table_mm - body_depth_mm
    dropout_rate = float((invalid_frac[object_mask] > INVALID_FRACTION_DROPOUT).mean()) \
        if object_mask.any() else float('nan')

    # Edge ring = dilate(object) - erode(object). Use the *filtered* object mask
    # so the ring sits around the true object body, not whatever spurious blobs
    # also passed the depth threshold.
    object_mask_u8 = object_mask.astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (EDGE_DILATE_PX, EDGE_DILATE_PX))
    dil = cv2.dilate(object_mask_u8, kernel)
    ero = cv2.erode(object_mask_u8, kernel)
    edge_ring = (dil > 0) & (ero == 0)
    # Flying / mixed pixels: depth strictly between body and table.
    edge_valid = edge_ring & np.isfinite(mean_mm) & (mean_mm > 0)
    if body_depth_mm and not np.isnan(body_depth_mm):
        flying = edge_valid & (mean_mm > body_depth_mm + 2) & (mean_mm < table_mm - 2)
        edge_flying_rate = float(flying.sum() / max(1, edge_valid.sum()))
    else:
        edge_flying_rate = float('nan')

    stats = {
        'name': name,
        'table_distance_m_measured': round(table_mm / 1000.0, 4),
        'object_body_depth_m': round(body_depth_mm / 1000.0, 4) if not np.isnan(body_depth_mm) else None,
        'object_height_mm': round(body_height_mm, 1) if not np.isnan(body_height_mm) else None,
        'object_pixels': int(object_mask.sum()),
        'body_sigma_mm': round(body_sigma_mm, 3),
        'body_dropout_rate': round(dropout_rate, 4),
        'edge_ring_pixels': int(edge_valid.sum()),
        'edge_flying_pixel_rate': round(edge_flying_rate, 4),
    }
    return stats, mean_mm, object_mask, edge_ring, table_mm


def save_overlay(out_path, mean_mm, object_mask, edge_ring, table_mm, name):
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
    im0 = ax[0].imshow(mean_mm, cmap='viridis')
    ax[0].set_title(f'{name} — mean depth (mm), table≈{table_mm:.0f} mm')
    plt.colorbar(im0, ax=ax[0], fraction=0.046)
    overlay = np.zeros((*mean_mm.shape, 3), dtype=np.uint8)
    overlay[object_mask] = (0, 200, 0)
    overlay[edge_ring] = (255, 50, 0)
    ax[1].imshow(overlay)
    ax[1].set_title('object mask (green) + edge ring (red)')
    for a in ax:
        a.set_xticks([]); a.set_yticks([])
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--in-dir', required=True)
    p.add_argument('--out-dir', default='./object_results')
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    npy_files = sorted(glob.glob(os.path.join(args.in_dir, '*.npy')))
    if not npy_files:
        raise SystemExit(f'No *.npy in {args.in_dir}')

    all_stats = []
    print(f'{"object":<15s} {"sigma":>8s} {"drop":>7s} {"flying":>8s} {"height":>8s}')
    print('-' * 55)
    for npy in npy_files:
        s, mean_mm, om, er, table_mm = analyze_one(npy)
        all_stats.append(s)
        save_overlay(
            os.path.join(args.out_dir, f'{s["name"]}_overlay.png'),
            mean_mm, om, er, table_mm, s['name'],
        )
        def _fmt(v, suffix='', width=6, prec=1):
            if v is None or (isinstance(v, float) and np.isnan(v)):
                return f'{"--":>{width}s}{suffix}'
            return f'{v:>{width}.{prec}f}{suffix}'
        print(f'{s["name"]:<15s} '
              f'{_fmt(s["body_sigma_mm"], " mm", 6, 2)} '
              f'{_fmt((s["body_dropout_rate"] or float("nan"))*100, "%", 5, 1)} '
              f'{_fmt((s["edge_flying_pixel_rate"] or float("nan"))*100, "%", 6, 1)} '
              f'{_fmt(s["object_height_mm"], " mm", 6, 1)}')

    out_json = os.path.join(args.out_dir, 'object_noise.json')
    with open(out_json, 'w') as fh:
        json.dump(all_stats, fh, indent=2)
    print(f'\nWrote {out_json}')


if __name__ == '__main__':
    main()
