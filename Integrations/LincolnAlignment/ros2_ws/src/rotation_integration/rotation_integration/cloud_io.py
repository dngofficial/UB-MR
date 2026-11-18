"""Read XYZ without mutating the message; honor PointCloud2 row padding and byte order."""
import numpy as np
from sensor_msgs.msg import PointField


def xyz(msg):
    if msg.width == 0 or msg.height == 0 or msg.point_step == 0:
        raise ValueError('PointCloud2 is empty')
    if msg.row_step < msg.width * msg.point_step:
        raise ValueError('PointCloud2 row_step is too small')
    needed = (msg.height - 1) * msg.row_step + msg.width * msg.point_step
    if len(msg.data) < needed:
        raise ValueError('PointCloud2 data is truncated')
    formats, offsets = [], []
    endian = '>' if msg.is_bigendian else '<'
    for name in ('x', 'y', 'z'):
        fields = [f for f in msg.fields if f.name == name]
        if len(fields) != 1 or fields[0].count != 1:
            raise ValueError('PointCloud2 requires one scalar field named ' + name)
        f = fields[0]
        sizes = {PointField.FLOAT32: 4, PointField.FLOAT64: 8}
        size = sizes.get(f.datatype)
        if size is None or f.offset + size > msg.point_step:
            raise ValueError('Invalid floating-point XYZ field layout')
        formats.append(endian + 'f' + str(size))
        offsets.append(f.offset)
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                      'offsets': offsets, 'itemsize': msg.point_step})
    raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step))
    points = np.column_stack([raw[name].reshape(-1) for name in ('x', 'y', 'z')]).astype(float)
    return points[np.isfinite(points).all(axis=1)]
