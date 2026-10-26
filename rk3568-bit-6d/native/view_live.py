"""Optional board-local Wayland viewer; the native process owns the camera."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ['XDG_RUNTIME_DIR'] = '/run'
os.environ.setdefault('WAYLAND_DISPLAY', 'wayland-0')
os.environ.setdefault('QT_QPA_PLATFORM', 'wayland')
import cv2
import numpy as np


def display_frame(image, calibrated, label=None):
    image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    label = label or ('Calibrated marker pose' if calibrated else 'Detection only - NOT calibrated')
    font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, .5, 1
    (width, height), baseline = cv2.getTextSize(label, font, scale, thickness)
    tile = np.zeros((height + baseline + 10, width + 10, 3), dtype=image.dtype)
    cv2.putText(tile, label, (5, height + 5), font, scale,
                (255, 255, 255), thickness, cv2.LINE_AA)
    # The LCD is mounted landscape: logical top-left maps to the viewer's top-right.
    tile = cv2.rotate(tile, cv2.ROTATE_90_COUNTERCLOCKWISE)
    image[10:10 + tile.shape[0], 10:10 + tile.shape[1]] = tile
    return image


def main():
    os.chdir(Path(__file__).resolve().parent)
    preview = Path('latest.pgm')
    preview.unlink(missing_ok=True)
    command = [os.environ.get('POSE6D_BINARY','./pose6d'), '--rga', '--width', '640', '--height', '360',
               '--threads', '2', '--frames', '0', '--preview', '--output', '/dev/null']
    command.extend(sys.argv[1:])
    process = subprocess.Popen(command)
    window = '6D live - Q or ESC to stop'
    previous = None
    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        while process.poll() is None:
            try:
                stamp = preview.stat().st_mtime_ns
                if stamp != previous:
                    image = cv2.imread(str(preview))
                    if image is not None:
                        label = os.environ.get('POSE6D_LABEL') or ('Marker-assisted bit pose' if '--assembly' in command else None)
                        cv2.imshow(window, display_frame(image, '--calibration' in command, label))
                        previous = stamp
            except FileNotFoundError:
                pass
            if cv2.waitKey(30) & 255 in (27, ord('q')):
                break
            time.sleep(.01)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=7)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        cv2.destroyAllWindows()
    return process.returncode


if __name__ == '__main__':
    raise SystemExit(main())
