#!/usr/bin/env python3
"""Inspect possible geographic relationships; emit an unconfirmed XY diagnostic, not a pose seed."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rotation_integration'))
from rotation_integration.georeference import inspect_lanelet, opendrive_georeference, project_xy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, default=Path('/home/devanng/host_data/maps/ub_autonomous_proving_grounds'))
    parser.add_argument('--opendrive', type=Path, default=REPO / 'Assets/UB_MR_Assets/RoadRunner/UB-Service-Center-Loop/ServiceCenterLoopMR.xodr')
    parser.add_argument('--bag-audit', type=Path, default=ROOT / 'reports/oakley_bag_audit.json')
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/local_georeference_audit.json')
    parser.add_argument('--plot', action='store_true', help='Write a standalone diagnostic PNG next to the JSON')
    args = parser.parse_args()
    projector_file = args.map_dir / 'map_projector_info.yaml'
    projector = yaml.safe_load(projector_file.read_text())
    lanelet_file = args.map_dir / 'lanelet2_map.osm'
    lanelet, local = inspect_lanelet(lanelet_file)
    projection = opendrive_georeference(args.opendrive)
    audit = json.loads(args.bag_audit.read_text())
    fix = audit['gnss_fix_at_requested_offset']
    ll = [[fix['longitude'], fix['latitude']]]
    utm = '+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs'
    wgs84 = '+proj=longlat +datum=WGS84 +no_defs'
    projected_fix = project_xy(ll, wgs84, projection)[0]
    projected_utm = project_xy([audit['gnss_odometry_at_requested_offset']['position'][:2]], utm, projection)[0]
    report = {'projector_file': str(projector_file), 'projector': projector,
              'lanelet_file': str(lanelet_file), 'lanelet_geographic_evidence': lanelet,
              'opendrive': str(args.opendrive), 'opendrive_sha256': hashlib.sha256(args.opendrive.read_bytes()).hexdigest(),
              'opendrive_projection': projection,
              'gnss_frame': fix['frame_id'], 'projection_of_gnss_fix_into_opendrive_xy_m': projected_fix.tolist(),
              'projection_of_utm_odometry_into_opendrive_xy_m': projected_utm.tolist(),
              'fix_vs_utm_projection_difference_m': float(np.linalg.norm(projected_fix - projected_utm)),
              'map_and_opendrive_frame_relationship_confirmed': False,
              'local_from_utm_confirmed': False, 'map_z': None, 'body_pose': None,
              'remaining_evidence': ['Verify this OpenDRIVE geographic reference describes the actual PCD local frame.',
                                     'Verify axis orientation and independent geographic/static landmarks.',
                                     'Establish the local vertical datum/origin and GNSS lever arm/orientation.'],
              'meaning': 'XY geographic hypothesis only. No 6-DOF map pose, initialization seed, TF or Unity correction is produced.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    if args.plot:
        # Headless diagnostic plots should not need a writable user configuration directory.
        cache = tempfile.TemporaryDirectory(prefix='lincoln-matplotlib-')
        os.environ.setdefault('MPLCONFIGDIR', cache.name)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 6))
        ax.scatter(local[:, 0], local[:, 1], s=2, color='slategray', label='Local Lanelet nodes')
        ax.scatter(*projected_fix, s=160, marker='x', color='crimson',
                   label='GNSS projected through Unity OpenDRIVE (unconfirmed for this map)')
        ax.set(xlabel='Local / projected X (m)', ylabel='Local / projected Y (m)',
               title='Geographic hypothesis only — PCD origin/axes and sensor pose unverified')
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(args.output.with_suffix('.png'), dpi=160)
        plt.close(fig)
        cache.cleanup()
    print(json.dumps({key: value for key, value in report.items() if key != 'lanelet_geographic_evidence'}, indent=2))


if __name__ == '__main__':
    main()
