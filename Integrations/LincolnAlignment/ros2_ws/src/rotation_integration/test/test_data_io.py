import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from rotation_integration.data_io import crop_xyz, load_xyz, validate_seed


ROOT = Path(__file__).resolve().parents[4]


class InputTests(unittest.TestCase):
    @unittest.skipUnless((ROOT / 'current_alignment').is_dir(), 'Bundle inputs not present in standalone package')
    def test_actual_scan_and_radius(self):
        scan = load_xyz(ROOT / 'current_alignment/lincoln_lidar_10s_30m.csv')
        self.assertEqual(scan.shape, (31110, 3))
        self.assertTrue(np.isfinite(scan).all())
        self.assertEqual(len(crop_xyz(scan, [0, 0, 0], 20)), 18653)
        self.assertEqual(len(crop_xyz(scan, [0, 0, 0], 30)), 31110)

    @unittest.skipUnless((ROOT / 'current_alignment').is_dir(), 'Bundle inputs not present in standalone package')
    def test_actual_pcd(self):
        cloud = load_xyz(ROOT / 'current_alignment/pointcloud_uniform_50k.pcd')
        self.assertEqual(cloud.shape, (49950, 3))
        self.assertTrue(np.isfinite(cloud).all())
        self.assertEqual(len(crop_xyz(cloud, [0, 0, 0], 30)), 1220)

    def test_crop_uses_explicit_center_and_rejects_bad_radius(self):
        cloud = np.array([[100, 10, 2], [100, 11, 2], [0, 0, 0], [np.nan, 0, 0]])
        self.assertEqual(len(crop_xyz(cloud, [100, 10, 2], 1.1)), 2)
        with self.assertRaises(ValueError):
            crop_xyz(cloud, [0, 0, 0], -1)

    def test_reordered_pcd_fields_and_count(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'test.pcd'
            p.write_text('FIELDS intensity z x y\nCOUNT 2 1 1 1\nPOINTS 1\nDATA ascii\n8 9 3 1 2\n')
            np.testing.assert_equal(load_xyz(p), [[1, 2, 3]])
            p.write_text('FIELDS x y z\nPOINTS 2\nDATA ascii\n1 2 3\n')
            with self.assertRaises(ValueError):
                load_xyz(p)

    def test_unconfirmed_seed_refused(self):
        template = {'confirmed': False}
        with self.assertRaisesRegex(ValueError, 'unconfirmed'):
            validate_seed(template)

    def seed(self):
        return {'confirmed': True, 'frame_id': 'map', 'body_frame': 'base_link',
                'position': [10, 20, 1], 'orientation_xyzw': [0, 0, 0, 1],
                'covariance': np.diag([1, 1, .2, .01, .01, .1]).flatten().tolist()}

    def test_valid_seed_and_wrong_frame(self):
        data = self.seed()
        p, q, c = validate_seed(data)
        np.testing.assert_equal(p, [10, 20, 1])
        data['frame_id'] = 'unity_pcd_unverified'
        with self.assertRaises(ValueError):
            validate_seed(data)

    def test_invalid_covariance_and_quaternion(self):
        data = self.seed()
        data['covariance'][0] = -1
        with self.assertRaises(ValueError):
            validate_seed(data)
        data = self.seed()
        data['orientation_xyzw'] = [0, 0, 0, 0]
        with self.assertRaises(ValueError):
            validate_seed(data)


if __name__ == '__main__':
    unittest.main()
