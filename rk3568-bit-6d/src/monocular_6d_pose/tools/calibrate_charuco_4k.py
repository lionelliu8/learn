#!/usr/bin/env python3
"""4K ChArUco calibration for OpenCV 4.5.x; no camera acquisition required."""
import argparse
import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from calibrate_camera import calibrate, matrix

IMAGE_SIZE = (3840, 2160)  # Change to actual ISP output; do not resize photos.
SQUARES = (7, 5)  # Number of squares, NOT inner corners.
SQUARE_M = 0.030
MARKER_M = 0.022
MIN_VIEWS = 15
MIN_CORNERS = 12
ROOT = Path('/home/llx/6D/calibration/charuco_4k')


def make_board():
    if not 0 < MARKER_M < SQUARE_M:
        raise ValueError('Require 0 < marker size < square size')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(*SQUARES, SQUARE_M, MARKER_M, dictionary)
    return dictionary, board


def detect(gray, dictionary, board):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None:
        return None, None
    count, points, corner_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if count < MIN_CORNERS:
        return None, None
    obj = board.chessboardCorners[corner_ids.ravel()]
    if np.linalg.matrix_rank(obj[:, :2] - obj[:, :2].mean(axis=0)) < 2:
        return None, None
    return points, corner_ids


def main():
    global ROOT, SQUARE_M, MARKER_M
    parser = argparse.ArgumentParser()
    parser.add_argument('--generate-board', action='store_true')
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--square-mm', type=float, default=SQUARE_M * 1000)
    parser.add_argument('--marker-mm', type=float, default=MARKER_M * 1000)
    parser.add_argument('--fix-k3', action='store_true', help='compatibility mode; default estimates k3')
    parser.add_argument('--rational', action='store_true', help='estimate k4-k6 rational distortion terms')
    args = parser.parse_args()
    ROOT, SQUARE_M, MARKER_M = args.root, args.square_mm / 1000, args.marker_mm / 1000
    dictionary, board = make_board()
    if args.generate_board:
        ROOT.mkdir(parents=True, exist_ok=True)
        target = ROOT / 'board_7x5_30mm_marker22mm.png'
        if target.exists():
            raise ValueError(f'Already exists: {target}')
        if not cv2.imwrite(str(target), board.draw((2100, 1500), marginSize=0, borderBits=1)):
            raise RuntimeError('Cannot write board')
        print(f'{target}\nPrint patterned region at 210 x 150 mm, add white margins; verify sizes with ruler.')
        return

    paths = sorted(p for p in (ROOT / 'images').glob('*')
                   if p.suffix.lower() in {'.png', '.jpg', '.jpeg', '.bmp', '.pgm'})
    objects, pixels, names, rejected = [], [], [], []
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            rejected.append({'file': path.name, 'reason': 'unreadable'})
            continue
        if (gray.shape[1], gray.shape[0]) != IMAGE_SIZE:
            raise ValueError(f'{path.name}: expected {IMAGE_SIZE}, got {gray.shape[::-1]}')
        points, ids = detect(gray, dictionary, board)
        if points is None:
            rejected.append({'file': path.name, 'reason': 'insufficient/non-2D ChArUco corners'})
            continue
        objects.append(board.chessboardCorners[ids.ravel()].copy())
        pixels.append(points)
        names.append(path.name)
    if len(names) < MIN_VIEWS:
        raise ValueError(f'Need {MIN_VIEWS} valid diverse views; got {len(names)}. Input: {ROOT / "images"}')
    flags = cv2.CALIB_FIX_K3 if args.fix_k3 else 0
    if args.rational:
        flags |= cv2.CALIB_RATIONAL_MODEL
    rms, camera, distortion, errors = calibrate(objects, pixels, IMAGE_SIZE, flags)
    output = ROOT / 'results' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    projection = np.zeros((3, 4))
    projection[:, :3] = camera
    camera_data = dict(image_width=IMAGE_SIZE[0], image_height=IMAGE_SIZE[1],
        camera_name='mipi_4k', camera_matrix=matrix(3, 3, camera),
        distortion_model='rational_polynomial' if args.rational else 'plumb_bob', distortion_coefficients=matrix(1, len(distortion.ravel()), distortion),
        rectification_matrix=matrix(3, 3, np.eye(3)), projection_matrix=matrix(3, 4, projection))
    report = dict(rms_px=rms, opencv=cv2.__version__, image_size=IMAGE_SIZE,
        squares=SQUARES, square_m=SQUARE_M, marker_m=MARKER_M, dictionary='DICT_4X4_50',
        flags=('CALIB_RATIONAL_MODEL' if args.rational else '') + ('|CALIB_FIX_K3' if args.fix_k3 else ''), rejected=rejected,
        views=[dict(file=n, corners=len(p), rms_px=e) for n,p,e in zip(names,pixels,errors)])
    for filename, data in [('camera.yaml', camera_data), ('report.json', report)]:
        (output / filename).write_text(json.dumps(data, indent=2), encoding='utf-8')
    # Re-read per frame instead of retaining 4K previews in RAM.
    for name, points in zip(names, pixels):
        image = cv2.imread(str(ROOT / 'images' / name))
        for point in points.reshape(-1, 2):
            cv2.circle(image, tuple(np.round(point).astype(int)), 7, (0,0,255), 2)
        preview = cv2.resize(image, (1280, 720))
        if not cv2.imwrite(str(output / (name + '.corners.jpg')), preview):
            raise RuntimeError('Cannot write preview')
    print(f'Valid views={len(names)}, RMS={rms:.4f}px; results={output}')
    print('K=', camera, '\nD=', distortion.ravel())


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, cv2.error) as exc:
        raise SystemExit(str(exc))
