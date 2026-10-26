#!/usr/bin/env python3
"""Replay recorded 4K corners; compare raw IPPE with both locally refined candidates."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_board import angle_degrees


def candidates(points, K, D, length=.020):
    h = length/2
    obj = np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]], np.float32)
    points = np.array(points, np.float32)
    count, rotations, translations, _ = cv2.solvePnPGeneric(obj, points, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    output = []
    for i in range(count):
        r, t = rotations[i].copy(), translations[i].copy()
        before = cv2.projectPoints(obj, r, t, K, D)[0].reshape(4,2)
        initial_error = float(np.linalg.norm(before-points, axis=1).mean())
        ok, r, t = cv2.solvePnP(obj, points, K, D, r, t, True, flags=cv2.SOLVEPNP_ITERATIVE)
        R = cv2.Rodrigues(r)[0]
        if not ok or not np.isfinite(r).all() or not np.isfinite(t).all() or np.min((obj@R.T+t.reshape(1,3))[:,2])<=0:
            continue
        projection = cv2.projectPoints(obj, r, t, K, D)[0].reshape(4,2)
        output.append(dict(seed=i, initial_mean_error_px=initial_error,
            mean_error_px=float(np.linalg.norm(projection-points, axis=1).mean()),
            xyz_m=t.ravel().tolist(), rvec_rad=r.ravel().tolist()))
    return sorted(output, key=lambda x:x['mean_error_px'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False, parents=True)
    cal = json.loads(args.calibration.read_text())
    K = np.array(cal['camera_matrix']['data'], float).reshape(3,3)
    D = np.array(cal['distortion_coefficients']['data'], float)
    # Known oblique synthetic pose checks corner ordering, units and positive depth.
    obj = np.array([[-.01,.01,0],[.01,.01,0],[.01,-.01,0],[-.01,-.01,0]], np.float32)
    rv, tv = np.array([2.8,.2,.1]), np.array([.01,-.02,.3])
    points = cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(4,2)
    check = candidates(points,K,D)[0]
    assert np.linalg.norm(np.array(check['xyz_m'])-tv)<1e-5
    assert angle_degrees(check['rvec_rad'],rv)<.01
    records=[]
    for row in map(json.loads,args.log.read_text().splitlines()):
        marker=next((m for m in row['markers'] if m['id']==42),None)
        record=dict(frame=row['frame'],native_pose_accepted=bool(marker and 'rvec_rad' in marker))
        if marker and 'corners_sensor_px' in marker:
            record['candidates']=candidates(marker['corners_sensor_px'],K,D)
            if record['candidates']:
                best=record['candidates'][0]
                record['refined_pose_accepted']=best['mean_error_px']<=2
                if 'rvec_rad' in marker:
                    record['change_from_native_deg']=angle_degrees(best['rvec_rad'],marker['rvec_rad'])
                if len(record['candidates'])>1:
                    other=record['candidates'][1]
                    record['candidate_angle_gap_deg']=angle_degrees(best['rvec_rad'],other['rvec_rad'])
                    record['candidate_error_gap_px']=other['mean_error_px']-best['mean_error_px']
        records.append(record)
    jumps=[]
    for a,b in zip(records,records[1:]):
        if a.get('refined_pose_accepted') and b.get('refined_pose_accepted'):
            angle=angle_degrees(a['candidates'][0]['rvec_rad'],b['candidates'][0]['rvec_rad'])
            if angle>10:jumps.append(dict(frames=[a['frame'],b['frame']],angle_deg=angle))
    summary=dict(frames=len(records),refined_accepted=sum(r.get('refined_pose_accepted',False) for r in records),
        refined_jumps_over_10deg=jumps,absolute_accuracy_validated=False,
        note='Offline solver comparison; rounded logged 4K corners, no temporal smoothing or altered 2px gate. Independent physical validation still required.')
    (args.output/'candidates.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
