#!/usr/bin/env python3
"""Fit an approximate map transform from manually matched XYZ landmarks."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rotation_integration'))
from rotation_integration.map_registration import transform


def landmark_seed(source, target):
    import open3d as o3d
    s, t = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if s.ndim != 2 or s.shape[1] != 3 or s.shape != t.shape or len(s) < 3:
        raise ValueError('Supply at least three corresponding XYZ landmarks in each map')
    if not np.isfinite(s).all() or not np.isfinite(t).all():
        raise ValueError('Landmarks must be finite')
    for pts in (s, t):
        values = np.linalg.svd(pts - pts.mean(0), compute_uv=False)
        if values[0] < 1e-6 or values[1] / values[0] < 1e-3:
            raise ValueError('Landmarks must not be collinear; choose widely separated features')
    src, dst = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    src.points, dst.points = o3d.utility.Vector3dVector(s), o3d.utility.Vector3dVector(t)
    pairs = o3d.utility.Vector2iVector(np.column_stack([np.arange(len(s))]*2))
    matrix = o3d.pipelines.registration.TransformationEstimationPointToPoint(False).compute_transformation(src, dst, pairs)
    errors = np.linalg.norm(transform(s, matrix) - t, axis=1)
    return matrix, errors


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--landmarks', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.landmarks.resolve() == a.output.resolve():
        p.error('Output must differ from input')
    data = json.loads(a.landmarks.read_text())
    matrix, errors = landmark_seed(data['unity_ros'], data['autoware'])
    config = json.loads((ROOT / 'config/map_alignment.template.json').read_text())
    config['initial_autoware_from_unity_ros'] = matrix.tolist()
    config['landmark_residuals_m'] = errors.tolist()
    config['landmark_rmse_m'] = float(np.sqrt(np.mean(errors**2)))
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(config, indent=2) + '\n')
    print('Wrote approximate transform. Inspect landmark residuals and confirm frames/units before setting confirmed=true.')


if __name__ == '__main__':
    main()
