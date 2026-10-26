#!/usr/bin/env python3
"""Offline chessboard calibration. Edit the constants below before real use."""
import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# 修改这里：内角点数，不是黑白格子的数量。9×6内角点对应10×7个格子。
BOARD_SIZE = (9, 6)  # (columns, rows)
SQUARE_SIZE_M = 0.025  # 每个格子的实测边长：25mm
IMAGE_DIR = Path('/home/llx/6D/calibration/images')
OUTPUT_ROOT = Path('/home/llx/6D/calibration/results')
MIN_VIEWS = 10
CAMERA_NAME = 'monocular_camera'


def board_points():
    if min(BOARD_SIZE) < 2 or SQUARE_SIZE_M <= 0:
        raise ValueError('Invalid board dimensions or square size')
    points = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
    points[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
    points *= SQUARE_SIZE_M
    return points


def calibrate(object_views, image_views, image_size, flags=0):
    rms, camera, distortion, rotations, translations = cv2.calibrateCamera(
        object_views, image_views, image_size, None, None,
        flags=flags)  # plumb_bob: k1,k2,p1,p2,k3
    if not np.isfinite(rms) or not np.isfinite(camera).all() or not np.isfinite(distortion).all():
        raise RuntimeError('Calibration produced non-finite values')
    if camera[0, 0] <= 0 or camera[1, 1] <= 0:
        raise RuntimeError('Calibration produced invalid focal lengths')
    errors = []
    for obj, pixels, rvec, tvec in zip(object_views, image_views, rotations, translations):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera, distortion)
        delta = (projected - pixels).reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(delta * delta, axis=1)))))
    return float(rms), camera, distortion, errors


def matrix(rows, cols, data):
    return {'rows': rows, 'cols': cols, 'data': np.asarray(data).reshape(-1).tolist()}


def main():
    if not IMAGE_DIR.is_dir():
        raise ValueError(f'Create {IMAGE_DIR} and add chessboard photos first')
    paths = sorted(p for p in IMAGE_DIR.iterdir()
                   if p.suffix.lower() in {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'})
    if not paths:
        raise ValueError(f'No calibration images in {IMAGE_DIR}')
    template = board_points()
    objects, images, accepted, rejected = [], [], [], []
    image_size = None
    previews = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            rejected.append({'file': path.name, 'reason': 'unreadable'})
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is not None and size != image_size:
            raise ValueError(f'Mixed resolutions: {path.name} is {size}, expected {image_size}')
        image_size = size
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, BOARD_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            rejected.append({'file': path.name, 'reason': 'chessboard not detected'})
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
        objects.append(template.copy())
        images.append(corners)
        accepted.append(path.name)
        cv2.drawChessboardCorners(image, BOARD_SIZE, corners, True)
        previews.append((path, image))

    if len(images) < MIN_VIEWS:
        raise ValueError(f'Only {len(images)} valid views; need >= {MIN_VIEWS}. Rejected: {rejected}')
    rms, camera, distortion, errors = calibrate(objects, images, image_size)
    # Timestamped folders preserve earlier calibration and avoid overwriting real parameters.
    output = OUTPUT_ROOT / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    width, height = image_size
    projection = np.zeros((3, 4))
    projection[:, :3] = camera
    data = {
        'image_width': width, 'image_height': height, 'camera_name': CAMERA_NAME,
        'camera_matrix': matrix(3, 3, camera),
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': matrix(1, 5, distortion),
        'rectification_matrix': matrix(3, 3, np.eye(3)),
        'projection_matrix': matrix(3, 4, projection),
    }
    # JSON syntax is valid YAML 1.2; avoids an extra PyYAML dependency.
    (output / 'camera.yaml').write_text(json.dumps(data, indent=2), encoding='utf-8')
    report = {
        'board_inner_corners': BOARD_SIZE, 'square_size_m': SQUARE_SIZE_M,
        'image_size': image_size, 'rms_px': rms, 'flags': 'CALIB_FIX_K3',
        'views': [{'file': name, 'rms_px': error} for name, error in zip(accepted, errors)],
        'rejected': rejected,
    }
    (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    for index, (path, preview) in enumerate(previews):
        if not cv2.imwrite(str(output / f'corners_{index:03d}_{path.stem}.png'), preview):
            raise RuntimeError('Could not save corner preview')
    print(f'Valid views: {len(images)}/{len(paths)}; calibration RMS: {rms:.4f} px')
    print(f'K =\n{camera}\nD = {distortion.ravel()}')
    print(f'Results saved to {output}')
    print('Check corner previews and per-view errors. RMS is not real-world distance accuracy.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, cv2.error) as exc:
        raise SystemExit(str(exc))
