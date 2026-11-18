#!/usr/bin/env python3
"""Inspect the saved scene's assigned mesh and compare its vertices with Jake's PCD."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/rotation_integration'))
from rotation_integration.data_io import load_xyz


def mesh_xyz(path):
    text = path.read_text()
    count = int(re.search(r'm_VertexCount: (\d+)', text)[1])
    channels = re.findall(r'- stream: (\d+)\s+offset: (\d+)\s+format: (\d+)\s+dimension: (\d+)', text)
    if not channels or channels[0] != ('0', '0', '0', '3') or any(int(c[3]) for c in channels[1:]):
        raise ValueError('Audit supports the current uncompressed, position-only float32 mesh layout')
    data = bytes.fromhex(re.search(r'_typelessdata: ([0-9a-fA-F]+)', text)[1])
    if len(data) != count * 12:
        raise ValueError('Unexpected vertex-buffer size')
    return np.frombuffer(data, dtype='<f4').reshape(count, 3)


def reference_assignment(scene_text, selector_guid):
    docs = {key: value for key, value in re.findall(
        r'^--- !u!\d+ &(\d+)(?: stripped)?\n(.*?)(?=^--- !u!|\Z)', scene_text, re.M | re.S)}
    selectors = [doc for doc in docs.values() if f'guid: {selector_guid},' in doc]
    if len(selectors) != 1:
        raise ValueError('Expected exactly one saved NearbyPointCloudSelector in the scene')
    mesh_id = re.search(r'pointCloudMeshFilter: \{fileID: (\d+)\}', selectors[0])[1]
    mesh_filter = docs[mesh_id]
    mesh_guid = re.search(r'm_Mesh: \{fileID: \d+, guid: (\w+),', mesh_filter)[1]
    game_id = re.search(r'm_GameObject: \{fileID: (\d+)\}', mesh_filter)[1]
    game = docs[game_id]
    name = re.search(r'  m_Name: (.+)', game)[1]
    transform = next(doc for doc in docs.values() if doc.startswith('Transform:') and
                     f'm_GameObject: {{fileID: {game_id}}}' in doc)
    pose = {}
    for field in ('m_LocalPosition', 'm_LocalRotation', 'm_LocalScale'):
        values = re.search(field + r': \{([^}]+)\}', transform)[1]
        pose[field] = {key: float(value) for key, value in re.findall(r'(\w+): ([^,]+)', values)}
    pose['parent_file_id'] = re.search(r'm_Father: \{fileID: (\d+)\}', transform)[1]
    return name, mesh_guid, pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=REPO)
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/unity_scene_audit.json')
    args = parser.parse_args()
    repo = args.repo.resolve()
    scene = repo / 'Assets/UB_MR/Modules/UB-Service-Center-Loop/UB-Service-Center-Loop.unity'
    guid = re.search(r'guid: (\w+)', (repo / 'Assets/UB_MR/Scripts/PointCloud/NearbyPointCloudSelector.cs.meta').read_text())[1]
    name, assigned_guid, pose = reference_assignment(scene.read_text(), guid)
    pcd = load_xyz(ROOT / 'current_alignment/pointcloud_uniform_50k.pcd')
    variants, assigned = [], None
    for asset in sorted((repo / 'Assets/UB_MR/PointClouds/Imported').glob('*uniform*.asset')):
        asset_guid = re.search(r'guid: (\w+)', asset.with_suffix('.asset.meta').read_text())[1]
        vertices = mesh_xyz(asset)
        modes = {'RawXYZ': pcd, 'CurrentXZY': pcd[:, [0, 2, 1]],
                 'XZNegY': pcd[:, [0, 2, 1]] * [1, 1, -1],
                 'NegXZY': pcd[:, [0, 2, 1]] * [-1, 1, 1]}
        errors = {mode: float(np.max(np.abs(vertices - points))) for mode, points in modes.items()
                  if vertices.shape == points.shape}
        best = min(errors, key=errors.get) if errors else None
        row = {'asset': str(asset.relative_to(repo)), 'guid': asset_guid, 'points': len(vertices),
               'best_matching_import_mode': best,
               'max_coordinate_difference_m': errors.get(best), 'assigned_to_selector': asset_guid == assigned_guid}
        variants.append(row)
        if row['assigned_to_selector']:
            assigned = row
    if assigned is None or assigned['max_coordinate_difference_m'] > 1e-4:
        raise ValueError('Selected mesh does not match the supplied PCD under a known importer mode')
    report = {'scene': str(scene.relative_to(repo)), 'selected_object': name,
              'selected_mesh': assigned, 'cloud_transform_relative_to_parent': pose,
              'mesh_variants': variants,
              'source_pcd_sha256': hashlib.sha256((ROOT / 'current_alignment/pointcloud_uniform_50k.pcd').read_bytes()).hexdigest(),
              'published_frame': 'unity_map_ros',
              'published_coordinate_rule': 'mesh local -> TransformPoint (all parents) -> (world.z, -world.x, world.y)',
              'physical_map_alignment_verified': False,
              'notes': ['Mesh identity/import mode verified; the recording map provenance and map-to-Unity transform remain unverified.',
                        'Manual scene transform is an existing visual placement, not an Autoware calibration.',
                        'RawXYZ variants must not replace the XZY reference without reviewing axes/handedness.']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
