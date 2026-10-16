import importlib.util,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path.cwd()/'scripts'))
from projective_depth_common import Client,audit_sequence,pose,sha,write
spec=importlib.util.spec_from_file_location('exposure',Path('scripts/check-projective-rgbd-exposure.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
binary=Path('artifacts/projective-affine/before-binary').absolute();stationary=np.eye(4);moving=np.eye(4);a=.01;moving[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]];moving[:2,3]=[.015,-.01]
rows={}
for name,t,alpha,beta,outliers in (('stationary darkened',stationary,.75,25,False),('moving darkened',moving,.75,25,False),('moving brightened',moving,1.1,5,False),('moving exposed with outliers',moving,.75,25,True)):
 depth=np.full((120,160),2.5);first=m.texture(stationary);current=np.rint(alpha*m.texture(t).astype(float)+beta).clip(0,255).astype(np.uint8)
 if outliers:current.ravel()[::19]=255
 client=Client(binary,m.CAMERA,configuration=dict(photometric='keyframe'))
 initial=client.frame(1.,depth,image=dict(timestamp=1.001,gray=first.ravel().tolist()));u=client.frame(1.03,depth,image=dict(timestamp=1.031,gray=current.ravel().tolist()));client.finish();audit_sequence([dict(update=initial),dict(update=u)],[1.,1.03])
 r=dict(initial=initial,update=u,within_known_motion_gate=False)
 if u['accepted']:
  e=np.linalg.inv(t)@pose(u['first_from_current']);pos=float(np.linalg.norm(e[:3,3]));rot=float(np.arccos(np.clip((np.trace(e[:3,:3])-1)/2,-1,1)));r.update(position_error_m=pos,rotation_error_rad=rot,within_known_motion_gate=pos<.004 and rot<.004)
 rows[name]=r
out=Path('artifacts/projective-affine/before-exposure');out.mkdir();write(out/'controls.json',rows);write(out/'summary.json',dict(binary_sha256=sha(binary.read_bytes()),controls=4,within_known_motion_gate=sum(r['within_known_motion_gate'] for r in rows.values()),script_sha256=sha(Path(__file__).read_bytes())))
print({n:{k:v for k,v in r.items() if k not in ('initial','update')} for n,r in rows.items()})
