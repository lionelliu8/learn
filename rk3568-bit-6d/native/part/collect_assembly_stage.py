#!/usr/bin/env python3
"""Copy a bounded stage from a live JSONL log without interrupting the preview."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,help='Defaults to the board current-marker42-session.txt')
    parser.add_argument('--label',required=True)
    parser.add_argument('--frames',type=int,default=400)
    parser.add_argument('--warmup',type=int,default=60)
    args=parser.parse_args()
    if not args.label.replace('_','').isalnum() or args.frames<=0 or args.warmup<0:
        parser.error('Invalid stage label or counts')
    session=args.session or Path(Path(__file__).with_name('current-marker42-session.txt').read_text().strip())
    output=session/(args.label+'.jsonl')
    if output.exists():
        parser.error('Stage exists; preserve it and choose a new label')
    path=session/'continuous.jsonl'
    started=time.monotonic()
    deadline=started+90
    rows=[]
    # Read the tail once, then stream only new bytes; avoid repeatedly parsing a growing log.
    with path.open('rb') as stream:
        end=stream.seek(0,2)
        begin=max(0,end-16384)
        stream.seek(begin)
        tail=stream.read(end-begin)
        complete=tail[:tail.rfind(b'\n')+1]
        lines=complete.splitlines()
        last=json.loads(lines[-1]) if lines else None
        first=(last['frame']+1 if last else 0)+args.warmup
        stream.seek(begin+len(complete))
        while len(rows)<args.frames:
            offset=stream.tell()
            line=stream.readline()
            if not line.endswith(b'\n'):
                stream.seek(offset)
                if time.monotonic()>deadline:
                    raise RuntimeError('Not enough frames; preview may have stopped. Raw log preserved.')
                time.sleep(.1)
                continue
            if line.strip():
                row=json.loads(line)
                if row['frame']>=first:
                    rows.append(row)
    with output.open('x') as stream:
        for row in rows:
            stream.write(json.dumps(row)+'\n')
    metadata={'label':args.label,'frames':len(rows),'first_frame':rows[0]['frame'],
              'last_frame':rows[-1]['frame'],'warmup_skipped':args.warmup,'source':str(path),
              'collector_elapsed_including_warmup_s':time.monotonic()-started}
    output.with_suffix('.stage.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps({'stage':str(output),**metadata},indent=2))


if __name__=='__main__':
    main()
