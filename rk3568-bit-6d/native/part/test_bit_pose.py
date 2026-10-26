"""Recover a known ideal planar-face pose; reject collinear input."""
import cv2
import numpy as np
from estimate_bit_pose import face_points, solve_face
from track_bit_pose import track_face

dimensions = {'total_length_mm':25, 'shaft_length_mm':17,
              'endface_corner_to_corner_width_mm':5, 'endface_parallel_flat_height_mm':4}
K = np.array([[2900.,0,1920], [0,2900,1080], [0,0,1]])
D = np.zeros(5)
r = np.array([.5, -.25, .1])
t = np.array([.01, -.02, .32])
pixels = cv2.projectPoints(face_points(dimensions), r, t, K, D)[0].reshape(-1, 2)
candidates = solve_face(pixels, dimensions, K, D)
assert len(candidates) == 2
best = min(candidates, key=lambda c: np.linalg.norm(np.array(c['xyz_m'])-t))
assert np.linalg.norm(np.array(best['xyz_m'])-t) < 1e-6
assert np.linalg.norm(np.array(best['rotation_matrix'])-cv2.Rodrigues(r)[0]) < 1e-5
try:
    solve_face([[0,0],[1,1],[2,2],[3,3]], dimensions, K, D)
    raise AssertionError('Collinear points accepted')
except ValueError:
    pass
# Track a small known translation on textured synthetic imagery and reject blank imagery.
rng = np.random.default_rng(7)
reference = rng.integers(20, 220, (180, 260), dtype=np.uint8)
reference = cv2.GaussianBlur(reference, (3,3), 0)
target = cv2.warpAffine(reference, np.array([[1.,0,3],[0,1.,2]]), (260,180))
corners = np.array([[60,60],[60,100],[170,100],[170,60]], np.float32)
tracked, _ = track_face(reference, target, corners)
assert tracked is not None and np.max(np.abs(tracked-corners-[3,2])) < .15
assert track_face(reference, np.zeros_like(reference), corners)[0] is None
print('Planar real-image PnP entry checks PASS')
