#!/usr/bin/env python3
"""Summarize a stationary-board run; repeatability is NOT absolute accuracy."""
import argparse
import json
import math
import statistics
from pathlib import Path


def percentile(values, fraction=.95):
    return sorted(values)[max(0, math.ceil(fraction * len(values)) - 1)]


def quaternion(rvec):
    angle = math.sqrt(sum(v * v for v in rvec))
    scale = math.sin(angle / 2) / angle if angle else .5
    return [math.cos(angle / 2)] + [scale * v for v in rvec]


def angle_degrees(a, b):
    # q and -q represent the same orientation; also handles rvec branch changes.
    dot = abs(sum(x * y for x, y in zip(quaternion(a), quaternion(b))))
    return math.degrees(2 * math.acos(min(1., dot)))


def summarize(rows):
    if not rows:
        raise ValueError('No frames after warmup')
    valid = [r['board'] for r in rows if r.get('board', {}).get('valid')]
    result = {'frames': len(rows), 'valid_frames': len(valid),
              'valid_fraction': len(valid) / len(rows)}
    result['rejections'] = {
        'insufficient_corners': sum(r.get('board', {}).get('corners', 0) < 12 for r in rows),
        'rms_over_2px': sum(r.get('board', {}).get('rms_4k_px', 0) > 2 for r in rows)}
    gap = longest = 0
    for row in rows:
        gap = 0 if row.get('board', {}).get('valid') else gap + 1
        longest = max(longest, gap)
    result['longest_invalid_run_frames'] = longest
    for field in ('processing_ms', 'pose_ms'):
        values = sorted(r[field] for r in rows)
        result[field] = {'p50': statistics.median(values),
                         'p95': values[math.ceil(.95 * len(values)) - 1]}
    if valid:
        for b in valid:
            assert b['corners'] >= 12 and 0 <= b['rms_4k_px'] <= 2
            assert len(b['center_xyz_m']) == len(b['rvec_rad']) == 3
            assert all(math.isfinite(v) for v in b['center_xyz_m'] + b['rvec_rad'])
            assert b['center_xyz_m'][2] > 0
        result['mean_rms_4k_px'] = statistics.mean(b['rms_4k_px'] for b in valid)
        axes = [[b['center_xyz_m'][i] * 1000 for b in valid] for i in range(3)]
        result['center_mean_mm'] = [statistics.mean(a) for a in axes]
        result['center_std_mm'] = [statistics.pstdev(a) for a in axes]
        result['center_peak_to_peak_mm'] = [max(a) - min(a) for a in axes]
        median = [statistics.median(a) for a in axes]
        radial = [math.sqrt(sum((b['center_xyz_m'][i] * 1000 - median[i]) ** 2
                               for i in range(3))) for b in valid]
        result['center_p95_deviation_from_median_mm'] = percentile(radial)
        count = max(1, len(valid) // 5)
        result['last_minus_first_center_mean_mm'] = [
            statistics.mean(a[-count:]) - statistics.mean(a[:count]) for a in axes]
        # Use a recorded central frame as reference, avoiding a synthetic Euler average.
        reference = valid[len(valid) // 2]['rvec_rad']
        angles = [angle_degrees(b['rvec_rad'], reference) for b in valid]
        result['rotation_deviation_from_middle_valid_frame_deg'] = {
            'p50': statistics.median(angles), 'p95': percentile(angles), 'max': max(angles)}
        result['note'] = 'Std/peak-to-peak measure repeatability only if board and camera stayed still.'
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--warmup', type=int, default=10)
    args = parser.parse_args()
    if args.warmup < 0:
        parser.error('warmup must be nonnegative')
    rows = [json.loads(s) for s in args.log.read_text().splitlines() if s.strip()]
    print(json.dumps(summarize(rows[args.warmup:]), indent=2))
