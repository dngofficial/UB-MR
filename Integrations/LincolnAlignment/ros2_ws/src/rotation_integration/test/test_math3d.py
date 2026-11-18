import math
import unittest
import numpy as np
from rotation_integration.math3d import transform_xyz


class TransformTest(unittest.TestCase):
    def test_rotation_then_translation(self):
        result = transform_xyz([[1, 0, 0], [0, 1, 0]], [10, 20, 3],
                               [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)])
        np.testing.assert_allclose(result, [[10, 21, 3], [9, 20, 3]], atol=1e-10)

    def test_reject_invalid_quaternion(self):
        with self.assertRaises(ValueError):
            transform_xyz([[0, 0, 0]], [0, 0, 0], [0, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
