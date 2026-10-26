#!/usr/bin/env python3
"""Inspect the approximate CAD with existing OpenCV; drag to rotate, Q to exit."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from build_bit_model import mesh


def render(model, yaw, pitch):
    vertices, faces = mesh(model['total_length_mm'], model['endface_corner_to_corner_width_mm'],
                          model['endface_parallel_flat_height_mm'], model['head_length_mm'],
                          model['tip_width_ratio_assumed'])
    vertices = np.array(vertices, float)
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    rotation = np.array([[cy, 0, sy], [sp*sy, cp, -sp*cy], [-cp*sy, sp, cp*cy]])
    rotated = vertices @ rotation.T
    points = rotated[:, :2]*25 + [450, 310]
    image = np.full((650, 900, 3), 242, np.uint8)
    # Paint far faces first; wireframe is retained for reading the simplified shape.
    for face in sorted(faces, key=lambda f: float(rotated[list(f), 2].mean())):
        polygon = np.rint(points[list(face)]).astype(np.int32)
        color = (80, 170, 230) if max(face) >= 12 else (180, 185, 195)
        cv2.fillConvexPoly(image, polygon, color, cv2.LINE_AA)
        cv2.polylines(image, [polygon], True, (65, 65, 65), 1, cv2.LINE_AA)
    lines = ['Approximate driver-bit model | units: mm',
             'Total 25 | hex shaft 17 | head + transition 8',
             'End face width 5, height 4 | +X toward tip',
             'Orange: assumed taper envelope; Phillips flutes NOT modeled',
             'Drag to rotate. Q / Esc to close.']
    for i, line in enumerate(lines):
        cv2.putText(image, line, (20, 30+i*27), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (35, 35, 35), 1, cv2.LINE_AA)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Save preview and exit without opening a window')
    args = parser.parse_args()
    model = json.loads(Path(__file__).with_name('dimensions.json').read_text())
    state = {'yaw': .35, 'pitch': -.6, 'mouse': None}
    if args.output:
        if not cv2.imwrite(str(args.output), render(model, state['yaw'], state['pitch'])):
            raise RuntimeError('Preview write failed')
        return
    def mouse(event, x, y, flags, unused):
        if event == cv2.EVENT_LBUTTONDOWN:
            state['mouse'] = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            state['mouse'] = None
        elif event == cv2.EVENT_MOUSEMOVE and state['mouse']:
            px, py = state['mouse']
            state['yaw'] += (x-px)*.01
            state['pitch'] += (y-py)*.01
            state['mouse'] = (x, y)
    window = 'Driver bit CAD study'
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, mouse)
    while True:
        cv2.imshow(window, render(model, state['yaw'], state['pitch']))
        if cv2.waitKey(30) & 255 in (27, ord('q')):
            break
        # GTK does not implement WND_PROP_VISIBLE (returns -1). AUTOSIZE is supported.
        try:
            if cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0:
                break
        except cv2.error:
            break  # The user closed the native window.
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
