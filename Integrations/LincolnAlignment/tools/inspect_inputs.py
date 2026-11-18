#!/usr/bin/env python3
"""Audit the supplied clouds in their native, unverified frames."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rotation_integration'))
from rotation_integration.data_io import load_xyz


def stats(path):
    raw = load_xyz(path)
    a = raw[np.isfinite(raw).all(axis=1)]
    if len(a) < 2:
        raise ValueError('Need at least two finite points for inspection')
    nn = cKDTree(a).query(a, k=2)[0][:, 1]
    return a, {
        'file': Path(path).name,
        'sha256': hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        'points': len(raw), 'finite_points': len(a),
        'duplicate_xyz_rows': len(a) - len(np.unique(a, axis=0)),
        'xyz_min': a.min(axis=0).tolist(), 'xyz_max': a.max(axis=0).tolist(),
        'radius_from_file_origin_m': {
            str(r): int((np.linalg.norm(a, axis=1) <= r).sum()) for r in (20, 30)},
        'nn_spacing_m': {'median': float(np.median(nn)), 'p90': float(np.quantile(nn, .9))},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan', type=Path, default=ROOT / 'current_alignment/lincoln_lidar_10s_30m.csv')
    parser.add_argument('--reference', type=Path, default=ROOT / 'current_alignment/pointcloud_uniform_50k.pcd')
    parser.add_argument('--output', type=Path, default=ROOT / 'reports')
    args = parser.parse_args()
    scan, scan_stats = stats(args.scan)
    reference, reference_stats = stats(args.reference)
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'scan': scan_stats, 'reference': reference_stats,
              'alignment_computed': False,
              'notes': [
                  'Reference is identified as a Unity sample by Jake; no Autoware map is included.',
                  'Original frame_id, exact timestamp, sensor extrinsic and map-to-Unity transform are missing.',
                  'Radius counts for the reference are around its file origin, not a verified vehicle location.',
                  'No common frame, initial vehicle pose, or registration is inferred from these numbers.']}
    (args.output / 'input_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.6), constrained_layout=True)
    for ax, cloud, title in zip(axes, (scan, reference),
                               ('Lincoln scan — 31,110 XYZ rows', 'Unity reference sample — 49,950 XYZ rows')):
        points = ax.scatter(cloud[:, 0], cloud[:, 1], c=cloud[:, 2], s=.7, cmap='viridis', rasterized=True)
        ax.set_title(title)
        ax.set_xlabel('Native file X (m)')
        ax.set_ylabel('Native file Y (m)')
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
        fig.colorbar(points, ax=ax, shrink=.75, label='Native file Z (m)')
    fig.suptitle('Input inspection only — different frames and plot extents; no alignment', fontsize=12)
    fig.savefig(args.output / 'native_frame_overview.png', dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
