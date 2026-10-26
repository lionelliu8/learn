"""Inspect one original-resolution frame without calculating camera intrinsics."""
import json
import sys
from pathlib import Path
import cv2
import numpy as np
from calibrate_charuco_4k import make_board, detect, IMAGE_SIZE

path = Path(sys.argv[1])
gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
if gray is None or gray.shape[::-1] != IMAGE_SIZE:
    raise SystemExit('Expected readable original 3840x2160 image')
dictionary, board = make_board()
corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
points, ids = detect(gray, dictionary, board)
report = dict(file=path.name, image_size=IMAGE_SIZE,
              markers=0 if marker_ids is None else len(marker_ids),
              charuco_corners=0 if ids is None else len(ids))
if corners:
    xy = np.concatenate(corners).reshape(-1, 2)
    lo = xy.min(axis=0)
    hi = xy.max(axis=0)
    report['marker_bbox_normalized'] = [float(lo[0]/gray.shape[1]), float(lo[1]/gray.shape[0]),
                                        float(hi[0]/gray.shape[1]), float(hi[1]/gray.shape[0])]
    roi = gray[int(lo[1]):int(hi[1])+1, int(lo[0]):int(hi[0])+1]
    report['roi_laplacian_variance'] = float(cv2.Laplacian(roi, cv2.CV_64F).var())
preview = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
if points is not None:
    cv2.aruco.drawDetectedCornersCharuco(preview, points, ids)
preview = cv2.rotate(cv2.resize(preview, (1280, 720)), cv2.ROTATE_180)
target = path.with_suffix('.inspection.jpg')
if not cv2.imwrite(str(target), preview):
    raise RuntimeError('Cannot write inspection preview')
print(json.dumps(report, indent=2))
print('Preview:', target)
