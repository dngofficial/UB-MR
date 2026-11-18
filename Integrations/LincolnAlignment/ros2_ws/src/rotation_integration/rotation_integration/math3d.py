"""Frame-safe helpers independent of ROS for testing."""
import numpy as np


def transform_xyz(points, translation, quaternion):
    xyz = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    q = np.asarray(quaternion, dtype=np.float64)
    t = np.asarray(translation, dtype=np.float64)
    if q.shape != (4,) or t.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(t).all():
        raise ValueError('Invalid transform')
    norm = np.linalg.norm(q)
    if norm < 1e-12 or abs(norm - 1.0) > 1e-3:
        raise ValueError('Invalid TF quaternion norm')
    x, y, z, w = q / norm
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return xyz @ r.T + t
