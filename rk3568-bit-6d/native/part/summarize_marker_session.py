#!/usr/bin/env python3
"""Reproduce bounded-stage response results; do not conflate response with accuracy."""
import argparse
import hashlib
import json
from pathlib import Path
from analyze_marker import summarize,compare,read


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stages',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    labels=('baseline01','approach01','raised01','right_tilt01','motion01','motion_end_hold01','covered01','recovered01')
    summaries={}
    for label in labels:
        path=args.stages/(label+'.jsonl')
        rows=read(path)
        stage=json.loads(path.with_suffix('.stage.json').read_text())
        assert len(rows)==stage['frames']
        assert rows[0]['frame']==stage['first_frame'] and rows[-1]['frame']==stage['last_frame']
        assert all(b['frame']==a['frame']+1 for a,b in zip(rows,rows[1:]))
        result=summarize(rows)
        assert not result['object_pose_fields_present']
        result['not_detected_frames']=sum(not any(m['id']==42 for m in r['markers']) for r in rows)
        result['pose_rejected_frames']=sum(any(m['id']==42 and m.get('pose_rejected') for m in r['markers']) for r in rows)
        result['raw_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
        result['stage_metadata']=stage
        summaries[label]=result
    pairs=(('baseline01','approach01'),('approach01','raised01'),('raised01','right_tilt01'),
           ('right_tilt01','motion_end_hold01'),('motion_end_hold01','recovered01'))
    report={'scope':'ID42 marker pose only; bit mount unavailable',
            'metadata':json.loads((args.stages/'metadata.json').read_text()),'stages':summaries,
            'comparisons':{a+'__'+b:compare(summaries[a],summaries[b]) for a,b in pairs},
            'record_checks':{'contiguous_stage_frames':True,'no_object_pose_fields':True},
            'assessment':{'z_approach_response_observed':summaries['raised01']['xyz_median_mm'][2]<summaries['approach01']['xyz_median_mm'][2],
                          'occluded_stage_no_pose_output':summaries['covered01']['valid_marker_frames']==0,
                          'recovery_stage_all_frames_with_pose':summaries['recovered01']['valid_fraction']==1,
                          'continuous_motion_tracking_passed':False,'absolute_accuracy_validated':False,
                          'bit_object_pose_test_completed':False}}
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'response-summary-all.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'assessment':report['assessment'],
                     'counts':{k:[v['valid_marker_frames'],v['frames']] for k,v in summaries.items()}},indent=2))


if __name__=='__main__':
    main()
