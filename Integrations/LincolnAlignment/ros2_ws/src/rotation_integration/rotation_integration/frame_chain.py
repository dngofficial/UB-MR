"""Resolve complete fixed URDF chains; candidate calibrations never imply bag provenance."""
from collections import deque
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def xyz_rpy_matrix(xyz, rpy):
    xyz, rpy = np.asarray(xyz, dtype=float), np.asarray(rpy, dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
        raise ValueError('Joint origin needs finite xyz[3] and roll/pitch/yaw[3] in radians')
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    # URDF fixed-axis roll/pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll).
    out = np.eye(4)
    out[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                   [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                   [-sp, cp*sr, cp*cr]]
    out[:3, 3] = xyz
    return out


def profile_urdf(profile):
    root = ET.Element('robot', name='lincoln_replay_sensor_frames')
    joints = profile['joints']
    frames = sorted({edge[key] for edge in joints for key in ('parent', 'child')})
    for frame in frames:
        ET.SubElement(root, 'link', name=frame)
    for i, edge in enumerate(joints):
        matrix = xyz_rpy_matrix(edge['xyz_m'], edge['rpy_rad'])
        if edge['parent'] == edge['child']:
            raise ValueError('A joint cannot parent its own link')
        joint = ET.SubElement(root, 'joint', name=f'calibration_joint_{i}', type='fixed')
        ET.SubElement(joint, 'parent', link=edge['parent'])
        ET.SubElement(joint, 'child', link=edge['child'])
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, matrix[:3, 3])),
                      rpy=' '.join(map(str, edge['rpy_rad'])))
    return ET.tostring(root, encoding='unicode')


class FixedFrameChain:
    def __init__(self, urdf):
        root = ET.fromstring(urdf)
        if root.tag != 'robot' or any('xacro' in element.tag for element in root.iter()):
            raise ValueError('Supply an expanded URDF, not a xacro source')
        names = [link.get('name') for link in root.findall('link')]
        if any(not name or name.strip() != name for name in names) or len(set(names)) != len(names):
            raise ValueError('URDF links must have unique, nonempty names')
        self.graph = {name: [] for name in names}
        parents = {}
        for joint in root.findall('joint'):
            p, c = joint.find('parent'), joint.find('child')
            if p is None or c is None:
                raise ValueError('Joint is missing parent/child')
            parent, child = p.get('link'), c.get('link')
            if parent not in self.graph or child not in self.graph or child in parents or parent == child:
                raise ValueError('Joint references a missing link, ambiguous parent or self-link')
            parents[child] = parent
            if joint.get('type') != 'fixed':
                continue
            origin = joint.find('origin')
            def vector(key):
                return [float(v) for v in (origin.get(key, '0 0 0') if origin is not None else '0 0 0').split()]
            matrix = xyz_rpy_matrix(vector('xyz'), vector('rpy'))
            self.graph[parent].append((child, matrix, joint.get('name')))
            self.graph[child].append((parent, np.linalg.inv(matrix), joint.get('name')))
        for child in parents:
            seen, cursor = set(), child
            while cursor in parents:
                if cursor in seen:
                    raise ValueError('URDF joint graph contains a cycle')
                seen.add(cursor)
                cursor = parents[cursor]

    def lookup(self, parent, child):
        if parent not in self.graph or child not in self.graph:
            raise ValueError(f'Incomplete frame chain: missing link {parent!r} or {child!r}')
        queue = deque([(parent, np.eye(4), [parent])])
        visited = {parent}
        while queue:
            current, matrix, path = queue.popleft()
            if current == child:
                return matrix, path
            for neighbor, edge, _ in self.graph[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, matrix @ edge, path + [neighbor]))
        raise ValueError(f'No complete fixed-joint path from {parent!r} to {child!r}')


def replay_description(config_file, profile_name):
    profiles = json.loads(Path(config_file).read_text())['profiles']
    if profile_name not in profiles:
        raise ValueError('Unknown calibration profile: ' + profile_name)
    profile = profiles[profile_name]
    if profile.get('confirmed_for_bag') is not True:
        raise ValueError('Calibration is unconfirmed for this bag; inspect the complete chain and recording provenance first')
    if profile['body_frame'] == profile['scan_frame']:
        raise ValueError('Body and scan frames must be distinct')
    if profile['body_frame'] in {edge['child'] for edge in profile['joints']}:
        raise ValueError('Sensor replay description must be rooted at body_frame; do not supply map localization here')
    description = profile_urdf(profile)
    graph = FixedFrameChain(description)
    graph.lookup(profile['body_frame'], profile['scan_frame'])
    for frame in graph.graph:
        graph.lookup(profile['body_frame'], frame)
    return description
