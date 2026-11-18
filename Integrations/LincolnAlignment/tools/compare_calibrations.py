#!/usr/bin/env python3
"""Compare the reported and installed calibration candidates without publishing TF."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rotation_integration'))
from rotation_integration.frame_chain import FixedFrameChain, profile_urdf
from rotation_integration.data_io import load_xyz


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/calibration_candidates.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/calibration_comparison.json')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    reports, matrices = {}, {}
    for name, profile in config['profiles'].items():
        entry = {'source': profile['source'], 'confirmed_for_bag': profile['confirmed_for_bag'],
                 'combines_sources': profile.get('combines_sources', False),
                 'final_joint_description_model': profile.get('final_joint_description_model'),
                 'description_model_matches_bag_diagnostics': profile.get('description_model_matches_bag_diagnostics')}
        try:
            matrix, path = FixedFrameChain(profile_urdf(profile)).lookup(profile['body_frame'], profile['scan_frame'])
            entry.update({'complete_scan_chain': True, 'chain': path,
                          'body_from_scan': matrix.tolist(), 'sensor_origin_in_body_m': matrix[:3, 3].tolist()})
            matrices[name] = matrix
        except ValueError as exc:
            entry.update({'complete_scan_chain': False, 'reason': str(exc)})
        reports[name] = entry
    source = load_xyz(ROOT / 'current_alignment/lincoln_lidar_10s_30m.csv')
    installed = matrices['installed_20260303']
    jake = matrices['jake_with_installed_scan_joint_hypothesis']
    a = source @ installed[:3, :3].T + installed[:3, 3]
    b = source @ jake[:3, :3].T + jake[:3, 3]
    differences = np.linalg.norm(a - b, axis=1)
    rotation = installed[:3, :3].T @ jake[:3, :3]
    comparison = {'recorded_lidar_model_from_bag_diagnostics': config.get('recorded_lidar_model_from_bag_diagnostics'),
                  'profiles': reports,
                  'real_scan_sensitivity': {'points': len(source),
                    'median_point_displacement_m': float(np.median(differences)),
                    'p95_point_displacement_m': float(np.quantile(differences, .95)),
                    'max_point_displacement_m': float(differences.max()),
                    'sensor_origin_difference_m': float(np.linalg.norm(installed[:3, 3] - jake[:3, 3])),
                    'relative_rotation_deg': float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))},
                  'selected_runtime_profile': None,
                  'meaning': 'Sensitivity comparison only. No candidate is confirmed for the recording and no TF or alignment is published.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(comparison, indent=2, allow_nan=False) + '\n')
    print(json.dumps(comparison, indent=2))


if __name__ == '__main__':
    main()
