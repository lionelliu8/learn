#!/usr/bin/env python3
"""Candidate localization in a clean dark ROI, without claiming a metric 6D pose."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def end_shape_hint(contour, shape, center, angle):
    component = np.zeros(shape, np.uint8)
    cv2.drawContours(component, [contour], -1, 255, cv2.FILLED)
    yy, xx = np.nonzero(component)
    direction = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])
    delta = np.column_stack((xx, yy)) - center
    along, across = delta @ direction, delta @ np.array([-direction[1], direction[0]])
    low, high = float(along.min()), float(along.max())
    bins = np.floor((along-low)/(high-low)*30).astype(int).clip(0, 29)
    widths = [float(np.ptp(across[bins == i])+1) if np.any(bins == i) else 0. for i in range(30)]
    negative, positive = float(np.median(widths[1:4])), float(np.median(widths[-4:-1]))
    ratio = max(negative, positive)/max(1., min(negative, positive))
    # A taper is a cue, not a validated semantic identity; keep ambiguity explicit.
    tip = 'negative_axis' if negative < positive else 'positive_axis'
    hint = tip if ratio >= 1.35 else 'ambiguous'
    endpoints = [np.array(center) + low*direction, np.array(center) + high*direction]
    return {'end_widths_px': [negative, positive], 'end_width_ratio': ratio,
            'narrow_end_hint': hint, 'axis_endpoints_px': [p.tolist() for p in endpoints],
            'width_profile_px': widths}


def candidates(gray):
    smooth = cv2.GaussianBlur(gray, (3, 3), 0)
    threshold, mask = cv2.threshold(smooth, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # A small close connects narrow specular gaps; dark/flared fragments may still fail.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        (cx, cy), (w, h), angle = cv2.minAreaRect(contour)
        length, width = max(w, h), min(w, h)
        if not (70 <= length <= 450 and 8 <= width <= 100 and 3 <= length / width <= 12):
            continue
        if cv2.contourArea(contour) / (w * h) < .3:
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        if x <= 2 or y <= 2 or x + bw >= gray.shape[1] - 2 or y + bh >= gray.shape[0] - 2:
            continue
        axis = (angle + (90 if h > w else 0)) % 180
        found.append({'center_px': [cx, cy], 'length_px': length, 'width_px': width,
                      'axis_angle_deg_mod180': axis,
                      'box_px': cv2.boxPoints(((cx, cy), (w, h), angle)).tolist(),
                      **end_shape_hint(contour, gray.shape, [cx, cy], axis)})
    return found, mask, threshold


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--roi', type=int, nargs=4, metavar=('X', 'Y', 'W', 'H'),
                        default=[960, 540, 1920, 1080])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    gray = cv2.imread(str(args.image), cv2.IMREAD_GRAYSCALE)
    if gray is None or gray.shape != (2160, 3840):
        parser.error('Expected original 3840x2160 image')
    x, y, w, h = args.roi
    if min(x, y) < 0 or min(w, h) <= 0 or x + w > 3840 or y + h > 2160:
        parser.error('ROI outside image')
    roi = gray[y:y+h, x:x+w]
    found, mask, threshold = candidates(roi)
    view = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    for item in found:
        box = np.array(item['box_px'])
        cv2.polylines(view, [np.rint(box).astype(np.int32)], True, (0, 255, 0), 2)
        item['center_px'] = [item['center_px'][0] + x, item['center_px'][1] + y]
        item['box_px'] = (box + [x, y]).tolist()
        endpoints = np.array(item['axis_endpoints_px'])
        if item['narrow_end_hint'] != 'ambiguous':
            tip = endpoints[0 if item['narrow_end_hint'] == 'negative_axis' else 1]
            cv2.circle(view, tuple(np.rint(tip).astype(int)), 5, (0, 0, 255), 2)
        item['axis_endpoints_px'] = (endpoints + [x, y]).tolist()
    result = {'image': str(args.image), 'roi_xywh': args.roi, 'threshold': threshold,
              'candidate_count': len(found), 'candidates': found,
              'metric_pose_valid': False, 'head_tail_known': False,
              'note': 'Image candidates only. Clean dark background required; no depth, roll or pitch estimated.'}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'inspection.json').write_text(json.dumps(result, indent=2))
    for name, image in [('roi.png', view), ('mask.png', mask)]:
        if not cv2.imwrite(str(args.output / name), image):
            raise RuntimeError('Image write failed: ' + name)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
