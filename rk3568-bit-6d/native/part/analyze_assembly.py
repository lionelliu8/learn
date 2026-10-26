#!/usr/bin/env python3
"""Summarize marker-assisted stages; static repeatability is not absolute accuracy."""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze_board import angle_degrees, percentile, quaternion


def rotate_vector(rvec, vector):
    w,x,y,z=quaternion(rvec)
    a,b,c=vector
    u=(y*c-z*b,z*a-x*c,x*b-y*a)
    v=(y*u[2]-z*u[1],z*u[0]-x*u[2],x*u[1]-y*u[0])
    return [vector[i]+2*w*u[i]+2*v[i] for i in range(3)]


def summarize(rows, assembly):
    if not rows:
        raise ValueError('No frames')
    valid=[]
    for row in rows:
        markers=[item for item in row['markers'] if item['id']==assembly['marker_id']]
        if len(markers)==1 and not markers[0].get('pose_rejected') and 'object_pose' in markers[0]:
            marker=markers[0]
            obj=marker['object_pose']
            values=marker['xyz_m']+marker['rvec_rad']+obj['xyz_m']+obj['rvec_rad']
            if not all(math.isfinite(v) for v in values) or obj['xyz_m'][2]<=0:
                raise ValueError('Invalid pose in accepted output')
            valid.append((row['frame'],marker,obj))
    result={'frames':len(rows),'valid_object_frames':len(valid),'valid_fraction':len(valid)/len(rows),
            'absolute_accuracy_validated':False,'mount_independently_verified':False}
    for field in ('processing_ms','resize_ms','detect_ms','pose_ms'):
        values=[r[field] for r in rows]
        result[field]={'p50':statistics.median(values),'p95':percentile(values)}
    if not valid:
        result['note']='No accepted object poses in this stage. No last-known pose was substituted.'
        return result
    for prefix,index in (('marker',1),('object',2)):
        axes=[[item[index]['xyz_m'][i]*1000 for item in valid] for i in range(3)]
        result[prefix+'_xyz_mean_mm']=[statistics.mean(a) for a in axes]
        result[prefix+'_xyz_std_mm']=[statistics.pstdev(a) for a in axes]
        result[prefix+'_xyz_range_mm']=[max(a)-min(a) for a in axes]
    reference=valid[len(valid)//2][2]['rvec_rad']
    angles=[angle_degrees(item[2]['rvec_rad'],reference) for item in valid]
    result['rotation_relative_to_middle_valid_frame_deg']={'p50':statistics.median(angles),'p95':percentile(angles),'max':max(angles)}
    jumps=[angle_degrees(a[2]['rvec_rad'],b[2]['rvec_rad']) for a,b in zip(valid,valid[1:])]
    result['rotation_max_between_valid_samples_deg']=max(jumps,default=0)
    result['rotation_jumps_over_10deg']=sum(v>10 for v in jumps)
    result['mean_marker_reprojection_px']=statistics.mean(v[1]['reprojection_px'] for v in valid)
    tilts=[math.degrees(math.acos(max(-1.,min(1.,-rotate_vector(v[1]['rvec_rad'],[0,0,1])[2])))) for v in valid]
    result['marker_normal_tilt_from_optical_axis_deg']={'p50':statistics.median(tilts),'p95':percentile(tilts)}
    viewing=[]
    for _,marker,_ in valid:
        normal=rotate_vector(marker['rvec_rad'],[0,0,1])
        position=marker['xyz_m']
        distance=math.sqrt(sum(v*v for v in position))
        cosine=-sum(a*b for a,b in zip(normal,position))/distance
        viewing.append(math.degrees(math.acos(max(-1.,min(1.,cosine)))))
    result['marker_obliquity_to_local_viewing_ray_deg']={'p50':statistics.median(viewing),'p95':percentile(viewing)}
    result['reprojection_coordinate_spaces']=sorted({v[1].get('reprojection_coordinate_space','unspecified') for v in valid})
    errors=[]
    offset=assembly['T_marker_object']['translation_m']
    for _,marker,obj in valid:
        rotated=rotate_vector(marker['rvec_rad'],offset)
        errors.append(math.sqrt(sum((marker['xyz_m'][i]+rotated[i]-obj['xyz_m'][i])**2 for i in range(3))))
    result['rigid_transform_numeric_max_error_mm']=max(errors)*1000
    result['reference_pose']={'frame':valid[len(valid)//2][0],**valid[len(valid)//2][2]}
    result['note']='Std/range describe this stage only. They represent repeatability only when the camera and assembly were static. Numeric transform consistency is not a physical accuracy check.'
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--assembly',type=Path,default=Path(__file__).with_name('assembly_marker42.json'))
    args=parser.parse_args()
    rows=[json.loads(s) for s in args.log.read_text().splitlines() if s.strip()]
    print(json.dumps(summarize(rows,json.loads(args.assembly.read_text())),indent=2))
