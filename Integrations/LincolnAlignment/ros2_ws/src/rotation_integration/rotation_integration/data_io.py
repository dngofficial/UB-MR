"""Read Jake's headered XYZ CSV and ASCII PCD without ROS or Open3D."""
from pathlib import Path
import numpy as np


def load_xyz(path):
    path = Path(path)
    if path.suffix.lower() == '.csv':
        data = np.genfromtxt(path, delimiter=',', names=True, dtype=np.float64)
        if data.dtype.names is None or not {'x', 'y', 'z'} <= set(data.dtype.names):
            raise ValueError('CSV requires x,y,z column headers')
        points = np.column_stack([np.atleast_1d(data[k]) for k in ('x', 'y', 'z')])
    elif path.suffix.lower() == '.pcd':
        with path.open() as f:
            header = {}
            for line in f:
                values = line.strip().split()
                if not values or values[0].startswith('#'):
                    continue
                header[values[0].upper()] = values[1:]
                if values[0].upper() == 'DATA':
                    break
            if header.get('DATA') != ['ascii']:
                raise ValueError('This loader supports ASCII PCD only; convert binary PCD first')
            fields = header.get('FIELDS', [])
            counts = [int(n) for n in header.get('COUNT', ['1'] * len(fields))]
            if len(counts) != len(fields) or any(n < 1 for n in counts):
                raise ValueError('Invalid PCD field counts')
            if not {'x', 'y', 'z'} <= set(fields):
                raise ValueError('PCD requires x,y,z fields')
            offsets = np.cumsum([0] + counts[:-1])
            for name in ('x', 'y', 'z'):
                if counts[fields.index(name)] != 1:
                    raise ValueError('XYZ fields must have COUNT 1')
            data = np.loadtxt(f, ndmin=2)
            expected = int(header['POINTS'][0])
            if data.shape != (expected, sum(counts)):
                raise ValueError('PCD POINTS/COUNT does not match payload')
            points = data[:, [offsets[fields.index(k)] for k in ('x', 'y', 'z')]]
    else:
        raise ValueError('Expected .csv or .pcd')
    if points.shape[0] == 0:
        raise ValueError('Empty cloud')
    return np.asarray(points, dtype=np.float64)


def crop_xyz(points, center, radius):
    center = np.asarray(center, dtype=np.float64)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError('Center must have three finite coordinates in the cloud frame')
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError('Radius must be positive and finite')
    finite = points[np.isfinite(points).all(axis=1)]
    return finite[np.linalg.norm(finite - center, axis=1) <= radius]


def validate_seed(data, map_frame='map', body_frame='base_link'):
    if data.get('confirmed') is not True:
        raise ValueError('Seed is unconfirmed: supply a measured/verified map-frame pose first')
    if data.get('frame_id') != map_frame or data.get('body_frame') != body_frame:
        raise ValueError('Seed must describe the configured body frame in the configured map frame')
    p = np.asarray(data.get('position'), dtype=float)
    q = np.asarray(data.get('orientation_xyzw'), dtype=float)
    c = np.asarray(data.get('covariance'), dtype=float)
    if p.shape != (3,) or q.shape != (4,) or c.shape != (36,):
        raise ValueError('Seed requires position[3], orientation_xyzw[4], covariance[36]')
    if not all(np.isfinite(v).all() for v in (p, q, c)):
        raise ValueError('Seed contains non-finite values')
    if abs(np.linalg.norm(q) - 1.0) > 1e-3:
        raise ValueError('Seed quaternion must be normalized')
    matrix = c.reshape(6, 6)
    if not np.allclose(matrix, matrix.T, atol=1e-9) or np.linalg.eigvalsh(matrix).min() < -1e-9:
        raise ValueError('Covariance must be symmetric positive semidefinite')
    if np.any(np.diag(matrix)[:3] <= 0) or matrix[5, 5] <= 0:
        raise ValueError('Provide positive XYZ and yaw variances; units are m^2 and rad^2')
    return p, q, c
