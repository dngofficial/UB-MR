"""Require a continuous, fresh, stationary localization window before alignment."""
import numpy as np


class LocalizationGate:
    def __init__(self, dwell_sec=2., max_gap_sec=.5, max_age_sec=2.,
                 max_linear_speed=.1, max_angular_speed=.05,
                 max_position_stddev=2., max_yaw_stddev_deg=15.,
                 max_position_drift=.2, max_rotation_drift_deg=2.):
        values = locals().copy()
        values.pop('self')
        if any(not np.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError('Localization gate thresholds must be positive and finite')
        self.settings = values
        self.reset()

    def reset(self, reason='Waiting for stationary localization'):
        self.since = self.last = None
        self.anchor_position = self.anchor_orientation = None
        self.reason = reason

    def observe(self, stamp, now, position, orientation, linear, angular, covariance):
        s = self.settings
        try:
            self._fresh(stamp, now)
            p, q, v, w, c = (np.asarray(x, dtype=float) for x in
                             (position, orientation, linear, angular, covariance))
            if (p.shape != (3,) or q.shape != (4,) or v.shape != (3,) or
                    w.shape != (3,) or c.shape != (36,) or
                    not all(np.isfinite(x).all() for x in (p, q, v, w, c))):
                raise ValueError('Invalid localization pose, twist or covariance')
            if abs(np.linalg.norm(q) - 1) > .001:
                raise ValueError('Localization quaternion is not normalized')
            q = q / np.linalg.norm(q)
            c = c.reshape(6, 6)
            if (not np.allclose(c, c.T, atol=1e-8) or np.linalg.eigvalsh(c).min() < -1e-8 or
                    np.any(np.diag(c)[[0, 1, 2, 5]] <= 0)):
                raise ValueError('Localization covariance is invalid or unknown (zero XYZ/yaw variance)')
            if (np.linalg.eigvalsh(c[:3, :3]).max() > s['max_position_stddev']**2 or
                    c[5, 5] > np.radians(s['max_yaw_stddev_deg'])**2):
                raise ValueError('Localization covariance exceeds configured limits')
            if np.linalg.norm(v) > s['max_linear_speed'] or np.linalg.norm(w) > s['max_angular_speed']:
                raise ValueError('Vehicle is moving; stationary window reset')
            if self.last is not None and (stamp <= self.last or stamp - self.last > s['max_gap_sec']):
                self.reset('Localization gap or clock reset; collecting a new stationary window')
            if self.since is not None:
                rotation_drift = np.degrees(2 * np.arccos(np.clip(abs(q @ self.anchor_orientation), 0, 1)))
                if (np.linalg.norm(p - self.anchor_position) > s['max_position_drift'] or
                        rotation_drift > s['max_rotation_drift_deg']):
                    self.reset('Pose drift exceeded the stationary limits; collecting a new window')
            if self.since is None:
                self.since, self.anchor_position, self.anchor_orientation = stamp, p.copy(), q.copy()
            self.last = stamp
            self.reason = ''
            return True
        except ValueError as exc:
            self.reset(str(exc))
            return False

    def _fresh(self, stamp, now):
        if not np.isfinite([stamp, now]).all() or stamp <= 0 or not -.1 <= now - stamp <= self.settings['max_age_sec']:
            raise ValueError('Localization is stale/future-dated; check use_sim_time and /clock')

    def require_ready(self, now):
        if self.last is None:
            raise ValueError(self.reason)
        self._fresh(self.last, now)
        elapsed = self.last - self.since
        if elapsed + 1e-9 < self.settings['dwell_sec']:
            raise ValueError(f'Collecting stationary localization: {elapsed:.2f}/{self.settings["dwell_sec"]:.2f} seconds')
        return elapsed
