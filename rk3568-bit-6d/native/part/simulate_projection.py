#!/usr/bin/env python3
"""Ideal labeled-point PnP simulation. Not a real-object detector or accuracy benchmark."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from build_bit_model import mesh


def run(calibration, dimensions, output):
    cal = json.loads(calibration.read_text())
    model = json.loads(dimensions.read_text())
    K = np.array(cal['camera_matrix']['data'], np.float64).reshape(3, 3)
    D = np.array(cal['distortion_coefficients']['data'], np.float64)
    vertices, faces = mesh(model['total_length_mm'], model['endface_corner_to_corner_width_mm'],
                          model['endface_parallel_flat_height_mm'], model['head_length_mm'],
                          model['tip_width_ratio_assumed'])
    objects = np.array(vertices, np.float64) / 1000  # CAD mm -> camera metres.
    truth_r = np.array([.4, -.3, .15], np.float64)
    truth_t = np.array([.01, -.01, .34], np.float64)
    pixels = cv2.projectPoints(objects, truth_r, truth_t, K, D)[0]
    rng = np.random.default_rng(20261025)
    true_rotation = cv2.Rodrigues(truth_r)[0]
    results = []
    for noise in (0., .25, .5, 1.):
        translations, rotations, failures = [], [], 0
        for _ in range(40):
            noisy = pixels + rng.normal(0, noise, pixels.shape)
            try:
                ok, r, t = cv2.solvePnP(objects, noisy, K, D, flags=cv2.SOLVEPNP_SQPNP)
                if not ok or t[2, 0] <= 0:
                    failures += 1
                    continue
                r, t = cv2.solvePnPRefineLM(objects, noisy, K, D, r, t)
                if not np.isfinite(r).all() or not np.isfinite(t).all() or t[2, 0] <= 0:
                    failures += 1
                    continue
            except cv2.error:
                failures += 1
                continue
            translations.append(float(np.linalg.norm(t.ravel()-truth_t)*1000))
            delta = cv2.Rodrigues(r)[0] @ true_rotation.T
            rotations.append(float(np.degrees(np.arccos(np.clip((np.trace(delta)-1)/2, -1, 1)))))
        result = {'noise_sigma_px': noise, 'trials': 40, 'failures': failures}
        if translations:
            result.update(translation_error_median_mm=float(np.median(translations)),
                          translation_error_p95_mm=float(np.percentile(translations, 95)),
                          rotation_error_median_deg=float(np.median(rotations)),
                          rotation_error_p95_deg=float(np.percentile(rotations, 95)))
        if noise == 0:
            assert len(translations) == 40 and max(translations) < 1e-4 and max(rotations) < 1e-3
        results.append(result)
    report = {'self_check': 'PASS', 'results': results,
              'assumptions': ['Exact calibration and exact synthetic geometry.',
                              'Every virtual vertex has a known correct 2D/3D identity, including hidden vertices.',
                              'No reflection, occlusion, blur, symmetry matching or measurement errors simulated.',
                              'Not evidence of real-object detection or metric accuracy.']}
    output.mkdir(parents=True, exist_ok=False)
    (output/'simulation.json').write_text(json.dumps(report, indent=2)+'\n')
    # An enlarged native-pixel crop makes the small target readable; annotations are synthetic.
    center = np.mean(pixels.reshape(-1, 2), axis=0)
    points = np.rint(pixels.reshape(-1, 2)-center+[400, 220]).astype(int)
    image = np.full((440, 800, 3), 245, np.uint8)
    for face in faces:
        cv2.polylines(image, [points[list(face)]], True, (140, 140, 140), 1, cv2.LINE_AA)
    for i, point in enumerate(points):
        cv2.circle(image, tuple(point), 3, (40, 60, 210), -1)
        cv2.putText(image, str(i), tuple(point+[3, -3]), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 0), 1)
    cv2.putText(image, 'Ideal labeled-point projection (not a camera detection)', (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (30, 30, 30), 1)
    if not cv2.imwrite(str(output/'projection.png'), image):
        raise RuntimeError('Failed to save projection')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--dimensions', type=Path, default=Path(__file__).with_name('dimensions.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.calibration, args.dimensions, args.output)
