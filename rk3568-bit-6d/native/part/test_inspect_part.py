"""Synthetic isolation, distractor and ambiguity checks (not real-image accuracy)."""
import cv2
import numpy as np
from inspect_part import candidates

image = np.zeros((700, 1000), np.uint8)
assert len(candidates(image)[0]) == 0
cv2.fillConvexPoly(image, np.rint(cv2.boxPoints(((500, 350), (220, 30), 25))).astype(np.int32), 180)
found, _, _ = candidates(image)
assert len(found) == 1 and abs(found[0]['length_px'] - 220) < 4
assert abs(found[0]['axis_angle_deg_mod180'] - 25) < 1
cv2.circle(image, (160, 160), 30, 220, -1)
assert len(candidates(image)[0]) == 1
cv2.rectangle(image, (600, 550), (800, 580), 180, -1)
assert len(candidates(image)[0]) == 2  # Must preserve ambiguity, not pick an arbitrary object.
# A tapered end changes sides after a physical 180-degree image rotation.
taper = np.zeros((400, 600), np.uint8)
cv2.fillConvexPoly(taper, np.array([[150, 200], [190, 180], [400, 180],
                                  [400, 220], [190, 220]], np.int32), 200)
one = candidates(taper)[0][0]
two = candidates(cv2.rotate(taper, cv2.ROTATE_180))[0][0]
assert one['narrow_end_hint'] != 'ambiguous'
assert {one['narrow_end_hint'], two['narrow_end_hint']} == {'positive_axis', 'negative_axis'}
print('Part candidate checks PASS')
