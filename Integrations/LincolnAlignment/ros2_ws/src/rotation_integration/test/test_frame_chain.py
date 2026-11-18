import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from rotation_integration.frame_chain import FixedFrameChain, profile_urdf, replay_description, xyz_rpy_matrix

ROOT = Path(__file__).resolve().parents[4]


class FrameChainTests(unittest.TestCase):
    def profiles(self):
        return json.loads((ROOT / 'config/calibration_candidates.json').read_text())['profiles']

    def test_fixed_axis_rotation_order(self):
        matrix = xyz_rpy_matrix([10, 20, 30], [np.pi/2, np.pi/2, np.pi/2])
        np.testing.assert_allclose(matrix @ [1, 2, 3, 1], [13, 22, 29, 1], atol=1e-12)

    def test_reported_chain_is_incomplete_until_final_sensor_joint(self):
        graph = FixedFrameChain(profile_urdf(self.profiles()['jake_reported']))
        with self.assertRaisesRegex(ValueError, 'missing link'):
            graph.lookup('base_link', 'velodyne_top')
        matrix, _ = graph.lookup('base_link', 'velodyne_top_base_link')
        np.testing.assert_allclose(matrix[:3, 3], [.9, 0, 2])

    def test_rotated_mount_composes_optical_height_in_sensor_axes(self):
        graph = FixedFrameChain(profile_urdf(self.profiles()['jake_with_installed_scan_joint_hypothesis']))
        matrix, path = graph.lookup('base_link', 'velodyne_top')
        np.testing.assert_allclose(matrix[:3, 3], [.9009933613391747, .0000299794790477413, 2.066102529713171], atol=1e-12)
        self.assertEqual(path, ['base_link', 'sensor_kit_base_link', 'velodyne_top_base_link', 'velodyne_top'])
        reverse, _ = graph.lookup('velodyne_top', 'base_link')
        np.testing.assert_allclose(reverse @ matrix, np.eye(4), atol=1e-12)

    def test_native_installed_urdf_includes_missing_height(self):
        graph = FixedFrameChain((ROOT / 'config/reference/installed_lidar_chain.urdf').read_text())
        matrix, _ = graph.lookup('base_link', 'velodyne_top')
        np.testing.assert_allclose(matrix[:3, 3], [1.24, 0, 1.99611], atol=1e-12)
        from_profile, _ = FixedFrameChain(profile_urdf(self.profiles()['installed_20260303'])).lookup('base_link', 'velodyne_top')
        np.testing.assert_allclose(matrix, from_profile, atol=1e-12)

    def test_replay_refuses_unconfirmed_and_incomplete_even_if_flagged_confirmed(self):
        config = {'profiles': self.profiles()}
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'calibration.json'
            p.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, 'unconfirmed'):
                replay_description(p, 'jake_reported')
            config['profiles']['jake_reported']['confirmed_for_bag'] = True
            p.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, 'missing link'):
                replay_description(p, 'jake_reported')

    def test_replay_refuses_a_localization_transform_above_body(self):
        profile = copy.deepcopy(self.profiles()['installed_20260303'])
        profile['confirmed_for_bag'] = True
        profile['joints'].append(dict(parent='map', child='base_link', xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0]))
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'calibration.json'
            p.write_text(json.dumps({'profiles': {'invalid': profile}}))
            with self.assertRaisesRegex(ValueError, 'rooted at body_frame'):
                replay_description(p, 'invalid')

    def test_moving_joint_cannot_complete_static_sensor_chain(self):
        text = '<robot><link name="a"/><link name="b"/><joint name="moving" type="revolute"><parent link="a"/><child link="b"/></joint></robot>'
        with self.assertRaisesRegex(ValueError, 'fixed-joint path'):
            FixedFrameChain(text).lookup('a', 'b')

    def test_cycles_ambiguous_parents_and_unexpanded_xacro_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'cycle'):
            FixedFrameChain('<robot><link name="a"/><link name="b"/><joint type="fixed"><parent link="a"/><child link="b"/></joint><joint type="fixed"><parent link="b"/><child link="a"/></joint></robot>')
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            FixedFrameChain('<robot><link name="a"/><link name="b"/><joint type="fixed"><parent link="a"/><child link="b"/></joint><joint type="fixed"><parent link="a"/><child link="b"/></joint></robot>')
        with self.assertRaisesRegex(ValueError, 'expanded'):
            FixedFrameChain('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="x"/></robot>')
