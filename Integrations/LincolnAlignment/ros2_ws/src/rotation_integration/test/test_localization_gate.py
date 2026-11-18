import unittest
import numpy as np
from rotation_integration.localization_gate import LocalizationGate


class LocalizationGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = LocalizationGate(dwell_sec=1.)
        self.covariance = np.diag([.01, .01, .01, .001, .001, .001]).ravel()

    def observe(self, stamp, **changes):
        args = dict(stamp=stamp, now=stamp, position=[1, 2, 3], orientation=[0, 0, 0, 1],
                    linear=[0, 0, 0], angular=[0, 0, 0], covariance=self.covariance)
        args.update(changes)
        return self.gate.observe(**args)

    def fill(self, start=10.):
        for stamp in np.arange(start, start + 1.1, .1):
            self.assertTrue(self.observe(float(stamp)))

    def test_requires_continuous_dwell(self):
        self.observe(10.)
        with self.assertRaisesRegex(ValueError, 'Collecting'):
            self.gate.require_ready(10.)
        self.fill()
        self.assertGreaterEqual(self.gate.require_ready(11.1), .99)

    def test_motion_invalidates_previous_window(self):
        self.fill()
        self.assertFalse(self.observe(11.2, linear=[.5, 0, 0]))
        with self.assertRaisesRegex(ValueError, 'moving'):
            self.gate.require_ready(11.2)
        self.observe(11.3)
        with self.assertRaisesRegex(ValueError, 'Collecting'):
            self.gate.require_ready(11.3)

    def test_gap_backwards_and_duplicate_stamps_reset_dwell(self):
        for next_stamp in (12., 9., None):
            self.fill()
            if next_stamp is None:
                next_stamp = self.gate.last
            self.observe(next_stamp)
            with self.assertRaisesRegex(ValueError, 'Collecting'):
                self.gate.require_ready(next_stamp)

    def test_stale_and_future_pose_rejected(self):
        self.fill()
        with self.assertRaisesRegex(ValueError, 'stale'):
            self.gate.require_ready(15.)
        self.assertFalse(self.observe(11.2, now=10.))
        self.assertFalse(self.observe(0.))

    def test_covariance_and_quaternion_rejected(self):
        for covariance in ([0.] * 36, [float('nan')] * 36, (self.covariance * 1e6).tolist()):
            self.assertFalse(self.observe(10., covariance=covariance))
        self.assertFalse(self.observe(10., orientation=[0, 0, 0, 0]))

    def test_stationary_pose_drift_resets_window_even_if_twist_is_zero(self):
        self.fill()
        self.observe(11.2, position=[1.5, 2, 3])
        with self.assertRaisesRegex(ValueError, 'Collecting'):
            self.gate.require_ready(11.2)


if __name__ == '__main__':
    unittest.main()
