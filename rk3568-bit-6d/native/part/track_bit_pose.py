#!/usr/bin/env python3
"""Small-motion reference tracking + PnP. Manual initialization; no automatic reacquisition."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from estimate_bit_pose import solve_face


def track_face(reference, current, points):
    points = np.array(points, np.float32).reshape(4, 2)
    lo = np.maximum(np.floor(points.min(axis=0)).astype(int)-24, 0)
    hi = np.minimum(np.ceil(points.max(axis=0)).astype(int)+25, [reference.shape[1], reference.shape[0]])
    x, y = lo
    ref = reference[y:hi[1], x:hi[0]]
    cur = current[y:hi[1], x:hi[0]]
    local = points-lo.astype(np.float32)
    mask = np.zeros_like(ref)
    cv2.fillConvexPoly(mask, np.rint(local).astype(np.int32), 255)
    # Include the face boundary: a smooth metallic interior alone has no reliable
    # longitudinal gradient. This context also limits invariance to background changes.
    mask = cv2.dilate(mask, np.ones((9, 9), np.uint8))
    if np.count_nonzero(mask) < 30 or ref[mask>0].std() < 3 or cur.std() < 3:
        return None, 'insufficient_texture'
    # Register the face patch, rather than four weak/reflective endpoint patches.
    # ponytail: affine, small-motion approximation; reinitialize after large viewpoint changes.
    warp = np.eye(2, 3, dtype=np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-5)
    try:
        score, warp = cv2.findTransformECC(ref, cur, warp, cv2.MOTION_AFFINE, criteria, mask, 3)
        if not np.isfinite(warp).all() or not np.isfinite(score) or score < .7:
            return None, 'appearance_mismatch'
        scales = np.linalg.svd(warp[:, :2], compute_uv=False)
        if np.linalg.det(warp[:, :2]) <= 0 or scales.min() < .8 or scales.max() > 1.25:
            return None, 'large_scale_or_shear_change'
        moved = cv2.transform(local.reshape(-1,1,2), warp).reshape(4,2)
        backward_score, backward = cv2.findTransformECC(cur, ref, cv2.invertAffineTransform(warp),
                                                        cv2.MOTION_AFFINE, criteria, mask, 3)
        returned = cv2.transform(moved.reshape(-1,1,2), backward).reshape(4,2)
    except cv2.error as error:
        return None, 'patch_registration_failed: ' + str(error).splitlines()[-1]
    round_trip = float(np.max(np.linalg.norm(returned-local, axis=1)))
    if not np.isfinite(backward_score) or backward_score < .7 or round_trip > 1.:
        return None, {'reason': 'bidirectional_registration_inconsistent',
                      'forward_correlation': float(score), 'backward_correlation': float(backward_score),
                      'round_trip_px': round_trip}
    candidate = moved+lo
    if not np.all((candidate >= [0,0]) & (candidate < [current.shape[1], current.shape[0]])):
        return None, 'point_outside_image'
    return candidate, {'method': 'bidirectional_affine_ECC', 'patch_correlation': float(score),
                       'backward_patch_correlation': float(backward_score),
                       'forward_backward_max_px': round_trip}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--points', type=Path, required=True)
    parser.add_argument('--frames', type=Path, required=True, help='Directory of PGM images')
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--dimensions', type=Path, default=Path(__file__).with_name('dimensions.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reference = cv2.imread(str(args.reference), cv2.IMREAD_GRAYSCALE)
    cal, dimensions = json.loads(args.calibration.read_text()), json.loads(args.dimensions.read_text())
    record = json.loads(args.points.read_text())
    if Path(record['image']).name != args.reference.name:
        parser.error('Reference and annotation image mismatch')
    expected = (cal['image_height'], cal['image_width'])
    if reference is None or reference.shape != expected:
        parser.error('Reference resolution mismatch')
    K = np.array(cal['camera_matrix']['data'], float).reshape(3, 3)
    D = np.array(cal['distortion_coefficients']['data'], float)
    paths = sorted(args.frames.glob('*.pgm'))
    if not paths:
        parser.error('No PGM frames')
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        row = {'image': str(path), 'status': 'rejected', 'unique_pose_validated': False}
        if image is None or image.shape != expected:
            row['reason'] = 'image_resolution_mismatch'
        else:
            points, quality = track_face(reference, image, record['points_px'])
            if points is None:
                row['reason'] = quality
            else:
                estimates = solve_face(points, dimensions, K, D)
                row.update(status='tracked_candidates' if estimates else 'rejected',
                           quality=quality, points_px=points.tolist(), candidates=estimates)
        rows.append(row)
    (args.output/'poses.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    summary = {'frames': len(rows), 'tracked_frames': sum(r['status'] == 'tracked_candidates' for r in rows),
               'mode': 'manual_reference_small_motion_tracking', 'real_time_performance_measured': False,
               'automatic_reacquisition': False, 'absolute_accuracy_validated': False,
               'annotation_provenance': record['provenance'],
               'note': 'Successful tracking preserves the seed assumptions; it cannot validate the physical corner labels or resolve planar ambiguity.'}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
