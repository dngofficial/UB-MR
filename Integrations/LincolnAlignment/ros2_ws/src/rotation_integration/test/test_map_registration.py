import importlib.util
import unittest
import numpy as np
from rotation_integration.map_registration import align_maps, rigid, transform


@unittest.skipUnless(importlib.util.find_spec('open3d'), 'Optional Open3D dependency not installed')
class RegistrationTests(unittest.TestCase):
    def clouds(self):
        rng = np.random.default_rng(73)
        source = rng.uniform([-12, -9, -2], [12, 9, 6], (2500, 3))
        truth = np.eye(4)
        theta = np.radians(15.)
        truth[:3, :3] = [[np.cos(theta), -np.sin(theta), 0],
                          [np.sin(theta), np.cos(theta), 0], [0, 0, 1]]
        truth[:3, 3] = [500000, 4200000, 10]
        target = transform(source, truth) + rng.normal(0, .003, source.shape)
        initial = truth.copy()
        initial[:3, 3] += [.35, -.25, .1]
        angle = np.radians(12.)
        initial[:3, :3] = [[np.cos(angle), -np.sin(angle), 0],
                           [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
        return source, target, initial, truth

    def test_map_transform_with_large_origin(self):
        source, target, initial, truth = self.clouds()
        report, aligned = align_maps(source, target, initial, truth[:3, 3])
        self.assertTrue(report['accepted'], report)
        np.testing.assert_allclose(report['autoware_from_unity_ros'], truth, atol=.015)
        self.assertLess(np.mean(np.linalg.norm(aligned - target, axis=1)), .02)

    def test_refinement_limit_rejects(self):
        source, target, initial, truth = self.clouds()
        report, _ = align_maps(source, target, initial, truth[:3, 3], max_refinement_translation=.05)
        self.assertFalse(report['accepted'])

    def test_empty_overlap_and_planar_cloud_rejected(self):
        source, target, initial, truth = self.clouds()
        with self.assertRaises(ValueError):
            align_maps(source, target, initial, [0, 0, 0])
        source[:, 2] = 0
        with self.assertRaisesRegex(ValueError, 'planar'):
            align_maps(source, transform(source, truth), truth, truth[:3, 3])


class RigidTests(unittest.TestCase):
    def test_reflection_rejected(self):
        with self.assertRaises(ValueError):
            rigid(np.diag([1, -1, 1, 1]))


if __name__ == '__main__':
    unittest.main()
