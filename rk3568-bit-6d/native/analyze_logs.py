#!/usr/bin/env python3
"""Compare measured processing latency; this is not camera end-to-end latency."""
import argparse
import json
import math
import statistics
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('logs',nargs='+',type=Path)
parser.add_argument('--warmup',type=int,default=10)
args=parser.parse_args()
for path in args.logs:
    rows=[json.loads(line) for line in path.read_text().splitlines() if line.strip()][args.warmup:]
    if not rows:
        raise SystemExit(f'{path}: insufficient samples after warmup')
    result={'file':str(path),'samples':len(rows),
            'detected_frames':sum(bool(row['markers']) for row in rows)}
    for field in ['resize_ms','detect_ms','pose_ms','processing_ms']:
        values=sorted(row[field] for row in rows)
        result[field]={'mean':round(statistics.mean(values),3),
                       'p50':round(statistics.median(values),3),
                       'p95':round(values[math.ceil(.95*len(values))-1],3)}
    print(json.dumps(result,indent=2))
