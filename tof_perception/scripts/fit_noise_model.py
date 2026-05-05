#!/usr/bin/env python3
"""
Fit a Nguyen-form axial noise model sigma_z(z) = a + b * z^2 to OAK-D ToF
captures produced by capture_tof_calibration.py.

Method (Nguyen 2012, Khoshelham & Elberink 2012, Sarbolandi 2015):
  - per-pixel temporal std over N frames (separates random noise from
    fixed-pattern / lens distortion structure)
  - bin pixels by mean depth, take median std per bin (robust to flying
    pixels, which are heavy-tailed outliers per Foix 2011)
  - least-squares fit of sigma_z = a + b * z^2 with z in meters,
    sigma_z in meters

Usage:
    python fit_noise_model.py --in-dir ./captures --out-dir ./fit_results

Output:
    fit_results/noise_params.json   # fitted a, b, plus bias curve
    fit_results/sigma_vs_z.png      # diagnostic plot
    fit_results/bias_vs_z.png       # systematic bias (reported, not injected)
"""
import argparse
import glob
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


BIN_WIDTH_M = 0.02       # 20 mm bins for sigma-vs-depth
MIN_PIXELS_PER_BIN = 50  # discard sparsely-sampled bins


def load_capture(npy_path):
    json_path = npy_path.replace('.npy', '.json')
    stack = np.load(npy_path)               # (N, H, W) uint16, mm
    with open(json_path) as fh:
        meta = json.load(fh)
    return stack, meta


def per_pixel_stats(stack):
    """Return (mean_mm, std_mm, valid_mask) ignoring zero (invalid) pixels."""
    s = stack.astype(np.float32)
    valid = s > 0
    n_valid = valid.sum(axis=0)
    keep = n_valid >= max(10, int(0.5 * stack.shape[0]))  # >=50% valid in time
    s_masked = np.where(valid, s, np.nan)
    with np.errstate(invalid='ignore'):
        mean = np.nanmean(s_masked, axis=0)
        std = np.nanstd(s_masked, axis=0)
    return mean, std, keep


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--in-dir', type=str, required=True)
    p.add_argument('--out-dir', type=str, default='./fit_results')
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    npy_files = sorted(glob.glob(os.path.join(args.in_dir, 'dist_*.npy')))
    if not npy_files:
        raise SystemExit(f'No dist_*.npy files in {args.in_dir}')

    # ---- aggregate per-pixel stats across all captures ----
    all_means_m = []
    all_stds_m = []
    per_distance = []  # for bias curve

    for npy in npy_files:
        stack, meta = load_capture(npy)
        d_true = meta['distance_m']
        mean_mm, std_mm, keep = per_pixel_stats(stack)

        # central ROI only — avoid lens vignetting / edge effects
        h, w = mean_mm.shape
        cy, cx = h // 2, w // 2
        roi = (slice(cy - h // 6, cy + h // 6), slice(cx - w // 6, cx + w // 6))
        m_roi = mean_mm[roi]
        s_roi = std_mm[roi]
        k_roi = keep[roi]

        m_flat = m_roi[k_roi] / 1000.0   # to meters
        s_flat = s_roi[k_roi] / 1000.0
        all_means_m.append(m_flat)
        all_stds_m.append(s_flat)

        per_distance.append({
            'distance_m_true': d_true,
            'mean_measured_m': float(np.nanmedian(m_flat)),
            'sigma_measured_m': float(np.nanmedian(s_flat)),
            'n_pixels': int(m_flat.size),
            'file': os.path.basename(npy),
        })
        print(f'{os.path.basename(npy):>20s}  '
              f'true={d_true:.3f} m  '
              f'meas={np.nanmedian(m_flat):.4f} m  '
              f'sigma={np.nanmedian(s_flat)*1000:.2f} mm  '
              f'pixels={m_flat.size}')

    means = np.concatenate(all_means_m)
    stds = np.concatenate(all_stds_m)
    finite = np.isfinite(means) & np.isfinite(stds) & (stds > 0)
    means = means[finite]
    stds = stds[finite]

    # ---- bin and take median std per bin (Sarbolandi 2015 robust practice) ----
    z_min, z_max = means.min(), means.max()
    bins = np.arange(z_min, z_max + BIN_WIDTH_M, BIN_WIDTH_M)
    idx = np.digitize(means, bins) - 1
    bin_z, bin_sigma = [], []
    for i in range(len(bins) - 1):
        mask = idx == i
        if mask.sum() < MIN_PIXELS_PER_BIN:
            continue
        bin_z.append(0.5 * (bins[i] + bins[i + 1]))
        bin_sigma.append(np.median(stds[mask]))
    bin_z = np.array(bin_z)
    bin_sigma = np.array(bin_sigma)

    # ---- least-squares fit sigma = a + b * z^2 ----
    X = np.stack([np.ones_like(bin_z), bin_z ** 2], axis=1)
    coeffs, *_ = np.linalg.lstsq(X, bin_sigma, rcond=None)
    a, b = float(coeffs[0]), float(coeffs[1])

    # ---- map to noise_injector parameters ----
    # Our injector currently uses a single "axial_scale" multiplier on top of
    # a Nguyen 2012 baseline (sigma_z = 0.0012 + 0.0019 * (z - 0.4)^2).
    # We expose both: the raw fitted (a, b) AND a recommended axial_scale at
    # the nominal working distance of 0.7 m.
    z_nom = 0.7
    sigma_fit_at_znom = a + b * z_nom ** 2
    sigma_nguyen_at_znom = 0.0012 + 0.0019 * (z_nom - 0.4) ** 2
    axial_scale_recommend = sigma_fit_at_znom / sigma_nguyen_at_znom

    params = {
        'model': 'sigma_z(z) = a + b * z^2  [meters, meters]',
        'fitted_a_m': a,
        'fitted_b_m_per_m2': b,
        'sigma_at_0.5m_mm': round(1000.0 * (a + b * 0.5 ** 2), 3),
        'sigma_at_0.7m_mm': round(1000.0 * (a + b * 0.7 ** 2), 3),
        'sigma_at_0.9m_mm': round(1000.0 * (a + b * 0.9 ** 2), 3),
        'axial_scale_recommend_at_0.7m': round(axial_scale_recommend, 3),
        'lateral_sigma_px_default': 0.8,  # Nguyen 2012 Eq. 1, perpendicular target
        'edge_probability_default': 0.30, # Foix 2011 / Sarbolandi 2015, scene-dependent — measure separately
        'dropout_probability_default': 0.02,
        'per_distance': per_distance,
        'n_bins_used': int(len(bin_z)),
        'bin_width_m': BIN_WIDTH_M,
        'citations': [
            'Nguyen, Izadi, Lovell 2012 — Modeling Kinect Sensor Noise',
            'Khoshelham & Elberink 2012 — Accuracy and Resolution of Kinect Depth Data',
            'Sarbolandi, Lefloch, Kolb 2015 — Kinect Range Sensing: SL vs ToF',
            'Foix, Alenya, Torras 2011 — Lost in Translation: A Survey of ToF',
        ],
    }
    with open(os.path.join(args.out_dir, 'noise_params.json'), 'w') as fh:
        json.dump(params, fh, indent=2)

    # ---- sigma-vs-z plot ----
    z_grid = np.linspace(bin_z.min(), bin_z.max(), 100)
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(bin_z, 1000 * bin_sigma, 'o', label='median per-bin (data)')
    ax.plot(z_grid, 1000 * (a + b * z_grid ** 2), '-',
            label=f'fit: $\\sigma$ = {1000*a:.2f} + {1000*b:.2f}·z² mm')
    ax.set_xlabel('depth z [m]')
    ax.set_ylabel('axial noise $\\sigma_z$ [mm]')
    ax.set_title('OAK-D SR PoE ToF axial noise (per-pixel temporal std)')
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(args.out_dir, 'sigma_vs_z.png'), dpi=150)

    # ---- bias-vs-z plot (reported, not injected) ----
    z_true = np.array([d['distance_m_true'] for d in per_distance])
    z_meas = np.array([d['mean_measured_m'] for d in per_distance])
    bias_mm = 1000.0 * (z_meas - z_true)
    fig2, ax2 = plt.subplots(figsize=(6, 4))
    ax2.plot(z_true, bias_mm, 'o-')
    ax2.axhline(0, color='k', linestyle=':', alpha=0.5)
    ax2.set_xlabel('true distance [m]')
    ax2.set_ylabel('measured − true [mm]')
    ax2.set_title('OAK-D SR PoE ToF systematic bias (reported, not injected)')
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(os.path.join(args.out_dir, 'bias_vs_z.png'), dpi=150)

    print()
    print('=' * 60)
    print(f'Fitted: sigma_z(z) = {1000*a:.3f} + {1000*b:.3f} * z^2  [mm, m]')
    print(f'  sigma at 0.5 m: {1000*(a+b*0.25):.2f} mm')
    print(f'  sigma at 0.7 m: {1000*(a+b*0.49):.2f} mm')
    print(f'  sigma at 0.9 m: {1000*(a+b*0.81):.2f} mm')
    print(f'Recommended axial_scale at z=0.7 m: {axial_scale_recommend:.3f}')
    print(f'Wrote: {args.out_dir}/noise_params.json')
    print(f'Wrote: {args.out_dir}/sigma_vs_z.png')
    print(f'Wrote: {args.out_dir}/bias_vs_z.png')


if __name__ == '__main__':
    main()
