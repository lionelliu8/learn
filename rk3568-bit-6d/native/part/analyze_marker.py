#!/usr/bin/env python3
"""Marker pose stage summary and relative response, with no object-mount assumptions."""
import argparse
import json
import math
from pathlib import Path
import statistics
from analyze_assembly import angle_degrees, percentile, rotate_vector


def summarize(rows, marker_id=42):
    if not rows:
        raise ValueError('Empty stage')
    poses=[]
    for row in rows:
        matches=[m for m in row['markers'] if m['id']==marker_id]
        if len(matches)==1 and not matches[0].get('pose_rejected') and 'xyz_m' in matches[0]:
            marker=matches[0]
            if len(marker['xyz_m'])!=3 or len(marker['rvec_rad'])!=3 or not all(
                    math.isfinite(v) for v in marker['xyz_m']+marker['rvec_rad']) or marker['xyz_m'][2]<=0:
                raise ValueError('Nonfinite or invalid accepted pose')
            poses.append(marker)
    result={'frames':len(rows),'marker_id':marker_id,'valid_marker_frames':len(poses),
            'valid_fraction':len(poses)/len(rows),'absolute_accuracy_validated':False,
            'object_pose_fields_present':any('object_pose' in m for row in rows for m in row['markers'])}
    if not poses:
        return result
    axes=[[p['xyz_m'][i]*1000 for p in poses] for i in range(3)]
    result['xyz_mean_mm']=[statistics.mean(a) for a in axes]
    result['xyz_median_mm']=[statistics.median(a) for a in axes]
    result['xyz_std_mm']=[statistics.pstdev(a) for a in axes]
    result['mean_reprojection_px']=statistics.mean(p['reprojection_px'] for p in poses)
    rotations=[p['rvec_rad'] for p in poses]
    # Use an actual observed medoid; do not average Rodrigues components across ±pi.
    reference=min(rotations,key=lambda r:sum(angle_degrees(r,q)**2 for q in rotations))
    deviations=[angle_degrees(r,reference) for r in rotations]
    result['rotation_reference_rvec_rad']=reference
    result['face_normal_reference_camera']=rotate_vector(reference,[0,0,1])
    result['rotation_deviation_p95_deg']=percentile(deviations)
    jumps=[angle_degrees(a,b) for a,b in zip(rotations,rotations[1:])]
    result['rotation_jumps_over_10deg']=sum(j>10 for j in jumps)
    result['processing_ms']={'p50':statistics.median(r['processing_ms'] for r in rows),
                             'p95':percentile([r['processing_ms'] for r in rows])}
    result['note']='Hand-held stages check response, not static precision; pose ambiguity and calibration error may remain.'
    return result


def compare(reference,current):
    if not reference.get('valid_marker_frames') or not current.get('valid_marker_frames'):
        return {'comparison_available':False}
    normal_a=reference['face_normal_reference_camera']
    normal_b=current['face_normal_reference_camera']
    normal_angle=math.degrees(math.acos(max(-1.,min(1.,sum(a*b for a,b in zip(normal_a,normal_b))))))
    return {'comparison_available':True,
            'delta_xyz_median_mm':[b-a for a,b in zip(reference['xyz_median_mm'],current['xyz_median_mm'])],
            'rotation_between_references_deg':angle_degrees(reference['rotation_reference_rvec_rad'],current['rotation_reference_rvec_rad']),
            'face_normal_change_deg':normal_angle,
            'reference_within_stage_rotation_p95_deg':reference['rotation_deviation_p95_deg'],
            'current_within_stage_rotation_p95_deg':current['rotation_deviation_p95_deg'],
            'note':'Differences are estimated responses; no independent displacement or angle truth was supplied.'}


def read(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--reference',type=Path)
    args=parser.parse_args()
    result=summarize(read(args.log))
    if args.reference:
        result['comparison']=compare(summarize(read(args.reference)),result)
    print(json.dumps(result,indent=2))
