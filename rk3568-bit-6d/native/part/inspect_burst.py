#!/usr/bin/env python3
"""Summarize an image-space pilot burst; no physical accuracy is inferred."""
import argparse
import json
import math
from pathlib import Path
import statistics
import cv2
from inspect_part import candidates

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('files', type=Path)
parser.add_argument('--roi', type=int, nargs=4, required=True)
args = parser.parse_args()
x, y, w, h = args.roi
samples, detections = [], []
for name in args.files.read_text().splitlines():
    if not name.strip():
        continue
    image = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
    if image is None or min(x, y) < 0 or min(w, h) <= 0 or x+w > image.shape[1] or y+h > image.shape[0]:
        raise ValueError('Unreadable image or invalid ROI: ' + name)
    found, _, _ = candidates(image[y:y+h, x:x+w])
    detections.append({'image': name, 'candidates': len(found)})
    if len(found) == 1:
        item = found[0]
        item['center_px'] = [item['center_px'][0]+x, item['center_px'][1]+y]
        samples.append(item)
result = {'roi_xywh': args.roi, 'frames': len(detections), 'single_candidate_frames': len(samples),
          'detections': detections, 'metric_pose_valid': False, 'head_tail_validated': False}
if samples:
    angles = [math.radians(s['axis_angle_deg_mod180']) for s in samples]
    mean = math.atan2(sum(math.sin(2*a) for a in angles), sum(math.cos(2*a) for a in angles))/2
    deltas = [math.degrees(math.atan2(math.sin(2*(a-mean)), math.cos(2*(a-mean)))/2) for a in angles]
    result.update(center_mean_px=[statistics.mean(s['center_px'][i] for s in samples) for i in range(2)],
                  center_std_px=[statistics.pstdev(s['center_px'][i] for s in samples) for i in range(2)],
                  mean_length_px=statistics.mean(s['length_px'] for s in samples),
                  mean_width_px=statistics.mean(s['width_px'] for s in samples),
                  axis_angle_std_deg_mod180=statistics.pstdev(deltas),
                  head_shape_hints=[s['narrow_end_hint'] for s in samples])
print(json.dumps(result, indent=2))
