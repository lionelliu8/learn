#!/usr/bin/env python3
"""Capture lossless 4K Y images through the existing native V4L2/RGA process."""
import argparse
import os
from datetime import datetime
from pathlib import Path
import signal
import subprocess
import time


def capture_existing(pid, count):
    command = Path('/proc') / str(pid) / 'cmdline'
    arguments = command.read_bytes().decode().rstrip('\0').split('\0')
    if Path(arguments[0]).name not in ('pose6d', 'pose6d-board-v2', 'pose6d-mounted') or '--snapshot-dir' not in arguments:
        raise ValueError('PID must be a verified native capture with --snapshot-dir')
    images = Path(arguments[arguments.index('--snapshot-dir')+1])
    if not images.is_absolute() or not images.is_dir():
        raise ValueError('An existing absolute snapshot directory is required')
    before = set(images.glob('*.pgm'))
    for index in range(count):
        os.kill(pid, signal.SIGUSR1)
        deadline = time.monotonic() + 8
        while len(set(images.glob('*.pgm')) - before) < index + 1:
            if not command.exists() or time.monotonic() > deadline:
                raise RuntimeError('Existing capture snapshot failed')
            time.sleep(.05)
        time.sleep(.3)
    return sorted(set(images.glob('*.pgm')) - before)


def capture(root, count):
    session = root / 'part/data' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    images = session / 'images'
    images.mkdir(parents=True)
    with (session / 'capture.log').open('w') as log:
        process = subprocess.Popen([
            str(root / 'pose6d-board-v2'), '--rga', '--width', '640', '--height', '360',
            '--threads', '2', '--frames', '0', '--output', '/dev/null',
            '--snapshot-dir', str(images)], cwd=root, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 12
            while 'frame=0 ' not in (session / 'capture.log').read_text():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Capture startup failed; inspect ' + str(session / 'capture.log'))
                time.sleep(.1)
            for index in range(count):
                process.send_signal(signal.SIGUSR1)
                deadline = time.monotonic() + 8
                while len(list(images.glob('*.pgm'))) < index + 1:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('Snapshot failed; inspect capture.log')
                    time.sleep(.05)
                time.sleep(.3)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=7)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
    return session


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--existing-pid', type=int, help='Keep the current preview/capture process running')
    args = parser.parse_args()
    if not 1 <= args.count <= 10:
        parser.error('count must be between 1 and 10')
    if args.existing_pid:
        for path in capture_existing(args.existing_pid, args.count):
            print(path)
    else:
        print(capture(Path(__file__).resolve().parents[1], args.count))
