"""Run: python3 test_calibration.py. Tests numerical recovery, not real lens accuracy."""
import unittest
import cv2
import numpy as np
from calibrate_camera import board_points, calibrate


class CalibrationTest(unittest.TestCase):
    def test_recovers_intrinsics_from_diverse_views(self):
        obj = board_points()
        expected = np.array([[800., 0., 640.], [0., 810., 480.], [0., 0., 1.]])
        distortion = np.array([-.12, .03, .001, -.001, 0.])
        rng = np.random.default_rng(42)
        images, objects = [], []
        for _ in range(20):
            rotation = rng.uniform(-.5, .5, 3)
            translation = np.array([rng.uniform(-.2, .05), rng.uniform(-.15, .05), rng.uniform(.6, 1.)])
            points, _ = cv2.projectPoints(obj, rotation, translation, expected, distortion)
            images.append(points)
            objects.append(obj.copy())
        rms, camera, actual_distortion, errors = calibrate(objects, images, (1280, 960))
        np.testing.assert_allclose(camera, expected, atol=.1)
        np.testing.assert_allclose(actual_distortion.ravel(), distortion, atol=.001)
        self.assertLess(rms, .001)
        self.assertEqual(len(errors), 20)


if __name__ == '__main__':
    unittest.main()
