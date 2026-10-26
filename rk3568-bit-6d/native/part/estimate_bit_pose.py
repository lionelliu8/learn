#!/usr/bin/env python3
"""Real-image planar-face PnP candidates, with explicit manual correspondence provenance."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from build_bit_model import mesh


def face_points(dimensions):
    length = dimensions['total_length_mm']
    shaft = dimensions['shaft_length_mm']
    width = dimensions['endface_corner_to_corner_width_mm']
    height = dimensions['endface_parallel_flat_height_mm']
    # This is the +Z flat of the affine hexagon. Its width is W/2, NOT W.
    # Real face width/shoulder geometry remain uncertain; record this assumption.
    rear, shoulder = -length/2, -length/2+shaft
    return np.array([[rear, -width/4, height/2], [rear, width/4, height/2],
                     [shoulder, width/4, height/2], [shoulder, -width/4, height/2]], np.float64)/1000


def solve_face(points, dimensions, K, D):
    points = np.array(points, np.float64)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError('Exactly four finite image points required')
    contour = points.astype(np.float32)
    if not cv2.isContourConvex(contour) or abs(cv2.contourArea(contour)) < 25:
        raise ValueError('Click a nondegenerate convex face in perimeter order')
    objects = face_points(dimensions)
    count, rotations, translations, _ = cv2.solvePnPGeneric(objects, points, K, D, flags=cv2.SOLVEPNP_IPPE)
    results = []
    for index in range(count):
        r, t = rotations[index], translations[index]
        R = cv2.Rodrigues(r)[0]
        camera = objects @ R.T + t.reshape(1, 3)
        if not np.isfinite(camera).all() or np.min(camera[:, 2]) <= 0:
            continue
        projected = cv2.projectPoints(objects, r, t, K, D)[0].reshape(-1, 2)
        residual = np.linalg.norm(projected-points, axis=1)
        results.append({'xyz_m': t.ravel().tolist(), 'rvec_rad': r.ravel().tolist(),
                        'rotation_matrix': R.tolist(), 'rms_px': float(np.sqrt(np.mean(residual**2))),
                        'max_residual_px': float(residual.max()),
                        'face_normal_toward_camera': bool(np.dot(R[:, 2], camera.mean(axis=0)) < 0)})
    return sorted(results, key=lambda item: item['rms_px'])


def annotate(image, roi):
    x, y, w, h = roi
    scale = min(3., 1100/w, 800/h)
    selected = []
    window = 'Real bit face: click 4 corners | R reset | Enter save | Esc cancel'
    cv2.namedWindow(window)
    def mouse(event, u, v, flags, unused):
        if event == cv2.EVENT_LBUTTONDOWN and len(selected) < 4:
            selected.append([u/scale+x, v/scale+y])
    cv2.setMouseCallback(window, mouse)
    try:
        while True:
            view = cv2.resize(image[y:y+h, x:x+w], None, fx=scale, fy=scale)
            for i, point in enumerate(selected):
                p = tuple(np.rint((np.array(point)-[x, y])*scale).astype(int))
                cv2.circle(view, p, 4, (0, 0, 255), -1)
                cv2.putText(view, str(i+1), p, cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 1)
            cv2.imshow(window, view)
            key = cv2.waitKey(30) & 255
            if key in (10, 13) and len(selected) == 4:
                return selected
            if key in (ord('r'), ord('R')):
                selected.clear()
            if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0:
                raise RuntimeError('Annotation cancelled; no pose accepted')
    finally:
        cv2.destroyWindow(window)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--dimensions', type=Path, default=Path(__file__).with_name('dimensions.json'))
    parser.add_argument('--points', type=Path, help='JSON with image path, points_px and provenance')
    parser.add_argument('--annotate', action='store_true')
    parser.add_argument('--roi', type=int, nargs=4, default=[2010, 660, 340, 100])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if bool(args.points) == args.annotate:
        parser.error('Choose exactly one of --points or --annotate')
    if args.output.exists():
        parser.error('Output exists; choose a new directory')
    cal, dimensions = json.loads(args.calibration.read_text()), json.loads(args.dimensions.read_text())
    K = np.array(cal['camera_matrix']['data'], np.float64).reshape(3, 3)
    D = np.array(cal['distortion_coefficients']['data'], np.float64)
    image = cv2.imread(str(args.image))
    if image is None or image.shape[:2] != (cal['image_height'], cal['image_width']):
        parser.error('Image must match calibration resolution')
    x, y, w, h = args.roi
    if min(x, y) < 0 or min(w, h) <= 0 or x+w > image.shape[1] or y+h > image.shape[0]:
        parser.error('ROI outside image')
    record = {'image': str(args.image.resolve()), 'points_px': annotate(image, args.roi),
              'provenance': 'user_manual_face_annotation', 'physically_confirmed': False} if args.annotate else json.loads(args.points.read_text())
    if Path(record['image']).name != args.image.name:
        parser.error('Annotation image mismatch')
    points = np.array(record['points_px'], float)
    if not np.all((points >= [0, 0]) & (points < [image.shape[1], image.shape[0]])):
        parser.error('Annotated points outside image')
    estimates = solve_face(points, dimensions, K, D)
    report = {'mode': 'manual_face_correspondences_pnp', 'annotation': record, 'candidates': estimates,
              'camera_coordinates': 'x right, y down, z forward; metres',
              'object_coordinates': dimensions['coordinate_frame'],
              'automatic_recognition': False, 'absolute_accuracy_validated': False,
              'unique_pose_validated': False,
              'assumptions': ['The clicked face is the +Z flat in the approximate affine-hexagon model.',
                              'Face dimensions17x2.5 mm are derived from reported dimensions, not independently measured.',
                              'Image shoulder/corner identities need physical confirmation.',
                              'Two planar solutions and a low reprojection residual do not prove correct 6D pose.']}
    args.output.mkdir(parents=True)
    (args.output/'pose.json').write_text(json.dumps(report, indent=2)+'\n')
    (args.output/'keypoints.json').write_text(json.dumps(record, indent=2)+'\n')
    vertices, faces = mesh(dimensions['total_length_mm'], dimensions['endface_corner_to_corner_width_mm'],
                          dimensions['endface_parallel_flat_height_mm'], dimensions['head_length_mm'],
                          dimensions['tip_width_ratio_assumed'])
    for index, candidate in enumerate(estimates):
        view = image.copy()
        r, t = np.array(candidate['rvec_rad']), np.array(candidate['xyz_m'])
        projected = cv2.projectPoints(np.array(vertices, float)/1000, r, t, K, D)[0].reshape(-1, 2)
        if np.isfinite(projected).all() and np.abs(projected).max() < 1e6:
            for face in faces:
                cv2.polylines(view, [np.rint(projected[list(face)]).astype(np.int32)], True, (100, 170, 255), 1)
        for i, point in enumerate(points):
            cv2.circle(view, tuple(np.rint(point).astype(int)), 3, (0, 255, 255), -1)
            cv2.putText(view, str(i+1), tuple(np.rint(point+[3, -3]).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1)
        cv2.drawFrameAxes(view, K, D, r, t, .008, 2)
        crop = view[max(0, y-60):min(image.shape[0], y+h+60), max(0, x-60):min(image.shape[1], x+w+60)]
        if not cv2.imwrite(str(args.output/f'candidate_{index}.png'), crop):
            raise RuntimeError('Failed to write candidate visualization')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
