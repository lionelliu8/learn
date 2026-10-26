import math
from analyze_marker import summarize,compare
def row(frame,depth,rotation):
    return {'frame':frame,'markers':[{'id':42,'xyz_m':[0,0,depth],'rvec_rad':rotation,'reprojection_px':.2}],
            'processing_ms':40}
baseline=summarize([row(i,.25,[3.14,0,0]) for i in range(3)])
near=summarize([row(i,.22,[3.14,0,0]) for i in range(3)])
assert abs(compare(baseline,near)['delta_xyz_median_mm'][2]+30)<1e-8
assert baseline['valid_marker_frames']==3 and not baseline['object_pose_fields_present']
lost=summarize([{'frame':1,'markers':[],'processing_ms':20}])
assert lost['valid_marker_frames']==0 and not compare(baseline,lost)['comparison_available']
# A90-degree in-plane rotation changes orientation, but not the face normal.
front=summarize([row(0,.25,[math.pi,0,0])])
turned=summarize([row(0,.25,[math.pi/math.sqrt(2),math.pi/math.sqrt(2),0])])
delta=compare(front,turned)
assert abs(delta['rotation_between_references_deg']-90)<1e-6 and delta['face_normal_change_deg']<1e-5
print('Marker response analysis checks PASS')
