import argparse,json
from pathlib import Path
import cv2
import numpy as np

p=argparse.ArgumentParser()
p.add_argument('image',type=Path);p.add_argument('--calibration',type=Path,required=True);p.add_argument('--assembly',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
im=cv2.imread(str(a.image));cal=json.loads(a.calibration.read_text());cfg=json.loads(a.assembly.read_text())
assert im is not None and im.shape[:2]==(cal['image_height'],cal['image_width'])
K=np.array(cal['camera_matrix']['data'],float).reshape(3,3);D=np.array(cal['distortion_coefficients']['data'],float)
gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);small=cv2.resize(gray,(1280,720),interpolation=cv2.INTER_AREA)
params=cv2.aruco.DetectorParameters_create();params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
corners,ids,_=cv2.aruco.detectMarkers(small,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),parameters=params)
assert ids is not None and cfg['marker_id'] in ids
index=ids.ravel().tolist().index(cfg['marker_id']);pts=(corners[index].reshape(4,2)+.5)*3-.5
cv2.cornerSubPix(gray,pts,(5,5),(-1,-1),(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01))
h=cfg['marker_length_m']/2;obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
ok,r,t=cv2.solvePnP(obj,pts,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE);assert ok
R=cv2.Rodrigues(r)[0];mount=cfg['T_marker_object'];Ro=R@np.array(mount['rotation_matrix']).reshape(3,3)
ro=cv2.Rodrigues(Ro)[0];to=R@np.array(mount['translation_m'])+t.ravel()
mesh=np.array(cfg['preview_geometry']['vertices_m']).reshape(-1,3)
projected=cv2.projectPoints(mesh,ro,to,K,D)[0].reshape(-1,2)
view=im.copy();edges=cfg['preview_geometry']['edges']
for i in range(0,len(edges),2):cv2.line(view,tuple(np.rint(projected[edges[i]]).astype(int)),tuple(np.rint(projected[edges[i+1]]).astype(int)),(0,255,0),2,cv2.LINE_AA)
endpoints=np.array([[-.0125,0,0],[.0125,0,0],[0,0,0]],float)
ep=cv2.projectPoints(endpoints,ro,to,K,D)[0].reshape(-1,2)
for label,point in zip(('TAIL','TIP','CENTER'),ep):
 xy=tuple(np.rint(point).astype(int));cv2.circle(view,xy,5,(0,0,255),2);cv2.putText(view,label,(xy[0]+8,xy[1]-10),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2)
allpoints=np.vstack([pts,projected]);lo=np.maximum(np.floor(allpoints.min(axis=0)-60).astype(int),0);hi=np.minimum(np.ceil(allpoints.max(axis=0)+60).astype(int),[im.shape[1],im.shape[0]])
x,y=lo;u,v=hi
cv2.imwrite(str(a.output/'alignment-crop.png'),view[y:v,x:u]);cv2.imwrite(str(a.output/'raw-crop.png'),im[y:v,x:u]);cv2.imwrite(str(a.output/'alignment-full.jpg'),view)
proj=cv2.projectPoints(obj,r,t,K,D)[0].reshape(4,2)
report=dict(marker_id=cfg['marker_id'],marker_xyz_m=t.ravel().tolist(),object_xyz_m=to.tolist(),object_rvec_rad=ro.ravel().tolist(),predicted_tail_tip_center_px=ep.tolist(),mean_marker_reprojection_px=float(np.linalg.norm(proj-pts,axis=1).mean()),mount_independently_verified=False,note='Nominal geometry projection only. Inspect endpoint alignment; no independent physical accuracy measurement.')
(a.output/'alignment.json').write_text(json.dumps(report,indent=2))
