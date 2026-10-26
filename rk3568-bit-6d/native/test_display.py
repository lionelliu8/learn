"""Check overlays from the physical landscape viewing direction."""
import cv2
import numpy as np
from view_live import display_frame

for calibrated in (False, True):
    source = np.zeros((360, 640, 3), dtype=np.uint8)
    logical = display_frame(source, calibrated)
    assert logical.shape == (640, 360, 3)
    physical = cv2.rotate(logical, cv2.ROTATE_90_CLOCKWISE)
    rows, columns = np.nonzero(physical.any(axis=2))
    assert rows.min() >= 10 and rows.max() < 50
    assert columns.min() > physical.shape[1] // 2
    source[100, 200] = (0, 0, 255)
    physical = cv2.rotate(display_frame(source, calibrated), cv2.ROTATE_90_CLOCKWISE)
    assert physical[259, 439].tolist() == [0, 0, 255]
print('PHYSICAL_LANDSCAPE_OVERLAY_PASS')
