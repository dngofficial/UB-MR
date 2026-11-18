#!/usr/bin/env python3
"""Crop a reference near an explicitly supplied center in that reference's frame."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rotation_integration'))
from rotation_integration.data_io import crop_xyz, load_xyz


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--center', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
    p.add_argument('--radius', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.resolve() == a.reference.resolve():
        p.error('Output must differ from input')
    cloud = load_xyz(a.reference)
    selected = crop_xyz(cloud, a.center, a.radius)
    if not len(selected):
        p.error('Crop contains no finite points; check the reference frame and center')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(a.output, selected, delimiter=',', header='x,y,z', comments='', fmt='%.12g')
    metadata = {'input': str(a.reference), 'center_in_reference_frame': a.center,
                'radius_m': a.radius, 'points': len(selected),
                'alignment_computed': False}
    a.output.with_suffix(a.output.suffix + '.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
