import importlib.util
import struct
import unittest
import numpy as np

ROS_MESSAGES_AVAILABLE = importlib.util.find_spec('sensor_msgs') is not None
if ROS_MESSAGES_AVAILABLE:
    from sensor_msgs.msg import PointCloud2, PointField
    from rotation_integration.cloud_io import xyz


@unittest.skipUnless(ROS_MESSAGES_AVAILABLE, 'Source ROS 2 to test PointCloud2 layout')
class CloudIOTests(unittest.TestCase):
    def cloud(self, endian):
        # Deliberately reorder z/x/y, add a fourth field, and pad each organized row.
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in [('z', 0), ('x', 4), ('y', 8), ('intensity', 12)]]
        rows = []
        for row in range(2):
            rows.append(b''.join(struct.pack(endian + 'ffff', 3 + i, 1 + i, 2 + i, 9.)
                                 for i in range(row * 2, row * 2 + 2)) + b'padding!')
        return PointCloud2(height=2, width=2, fields=fields, is_bigendian=endian == '>',
                           point_step=16, row_step=40, data=b''.join(rows), is_dense=True)

    def test_padded_organized_cloud_and_endianness_without_mutation(self):
        for endian in ('<', '>'):
            msg = self.cloud(endian)
            before = bytes(msg.data)
            np.testing.assert_allclose(xyz(msg), [[1, 2, 3], [2, 3, 4], [3, 4, 5], [4, 5, 6]])
            self.assertEqual(bytes(msg.data), before)

    def test_bad_layout_and_truncated_buffer(self):
        msg = self.cloud('<')
        msg.data = msg.data[:30]
        with self.assertRaisesRegex(ValueError, 'truncated'):
            xyz(msg)
        msg = self.cloud('<')
        msg.fields[0].count = 2
        with self.assertRaises(ValueError):
            xyz(msg)


if __name__ == '__main__':
    unittest.main()
