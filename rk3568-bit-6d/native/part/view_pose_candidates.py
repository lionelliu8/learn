#!/usr/bin/env python3
"""Display real-image PnP alternatives without silently accepting one."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, default=Path(__file__).parent/'results/manual_face_seed_v01')
    parser.add_argument('--output', type=Path, help='Save comparison without opening a GUI')
    args = parser.parse_args()
    report = json.loads((args.results/'pose.json').read_text())
    panels = []
    for i, candidate in enumerate(report['candidates']):
        image = cv2.imread(str(args.results/f'candidate_{i}.png'))
        if image is None:
            raise ValueError('Missing visualization for candidate ' + str(i))
        image = cv2.resize(image, (690, 330))
        panel = np.full((480, 690, 3), 245, np.uint8)
        panel[150:] = image
        xyz = [v*1000 for v in candidate['xyz_m']]
        lines = [f'Candidate {i} | manual seed | NOT validated',
                 'XYZ mm: ' + ', '.join(f'{v:.2f}' for v in xyz),
                 f'4-point reprojection RMS: {candidate["rms_px"]:.3f} px',
                 'X red, Y green, Z blue | orange: approximate model']
        for row, line in enumerate(lines):
            cv2.putText(panel, line, (12, 28+row*31), cv2.FONT_HERSHEY_SIMPLEX, .54, (25,25,25), 1)
        panels.append(panel)
    if not panels:
        raise ValueError('No pose candidates to display')
    canvas = np.hstack(panels)
    if args.output:
        if not cv2.imwrite(str(args.output), canvas):
            raise RuntimeError('Failed to save comparison')
        return
    window = 'Real bit pose candidates - manual initialization, unresolved ambiguity'
    cv2.namedWindow(window)
    cv2.moveWindow(window, 60, 100)
    while True:
        cv2.imshow(window, canvas)
        if cv2.waitKey(30) & 255 in (27, ord('q')):
            break
        try:
            if cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0:
                break
        except cv2.error:
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
