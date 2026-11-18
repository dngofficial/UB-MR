"""Audit map geographic anchors and project diagnostic XY hypotheses with local PROJ."""
import os
import shlex
import shutil
import subprocess
import xml.etree.ElementTree as ET
import numpy as np


def project_xy(points, source_projection, target_projection):
    executable = shutil.which('cs2cs')
    if executable is None:
        raise ValueError('Install PROJ command-line tools (cs2cs) to inspect geographic coordinates')
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Projection requires finite XY pairs')
    command = [executable, *shlex.split(source_projection), '+to', *shlex.split(target_projection), '-f', '%.12f']
    env = os.environ.copy()
    env['PROJ_NETWORK'] = 'OFF'
    text = ''.join(f'{x:.15f} {y:.15f}\n' for x, y in points)
    result = subprocess.run(command, input=text, text=True, capture_output=True, timeout=30, env=env)
    if result.returncode:
        raise ValueError('PROJ projection failed: ' + result.stderr.strip())
    try:
        projected = np.asarray([[float(value) for value in line.split()[:2]]
                                for line in result.stdout.splitlines()], dtype=float)
    except ValueError as exc:
        raise ValueError('PROJ could not project these coordinates') from exc
    if projected.shape != points.shape or not np.isfinite(projected).all():
        raise ValueError('PROJ returned invalid coordinates')
    return projected


def inspect_lanelet(path):
    nodes = ET.parse(path).getroot().findall('node')
    local, geographic = [], []
    for node in nodes:
        tags = {tag.get('k'): tag.get('v') for tag in node.findall('tag')}
        try:
            xy = [float(tags['local_x']), float(tags['local_y'])]
        except (KeyError, TypeError, ValueError):
            continue
        if not np.isfinite(xy).all():
            continue
        local.append(xy)
        try:
            ll = [float(node.get('lon')), float(node.get('lat'))]
        except (TypeError, ValueError):
            continue
        if np.isfinite(ll).all() and -180 <= ll[0] <= 180 and -90 <= ll[1] <= 90:
            geographic.append({'id': node.get('id'), 'longitude_latitude': ll, 'local_xy': xy})
    return {'node_count': len(nodes), 'local_coordinate_nodes': len(local),
            'paired_geographic_anchor_nodes': len(geographic),
            'anchors': geographic}, np.asarray(local).reshape(-1, 2)


def opendrive_georeference(path):
    header = ET.parse(path).getroot().find('header')
    if header is None or not header.findtext('geoReference', '').strip():
        raise ValueError('OpenDRIVE header has no geographic reference')
    offset = header.find('offset')
    if offset is not None and any(abs(float(offset.get(k, 0))) > 1e-12 for k in ('x', 'y', 'z', 'hdg')):
        raise ValueError('Nonzero OpenDRIVE offset needs explicit composition; refusing to ignore it')
    return header.findtext('geoReference').strip()
