import importlib.util,sys,json
from pathlib import Path
import numpy as np
r=Path('/workspace/rust_drive');sys.path.insert(0,str(r/'scripts'))
import projective_photo_raw as photo
from projective_depth_common import pose
spec=importlib.util.spec_from_file_location('raw_auditor',r/'scripts/audit-projective-depth-raw.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
original=photo.metadata;count=0;maximum=0.
class Done(Exception):pass
def inspect(update,context):
 global count,maximum
 original(update,context);count+=1
 if count>27:raise Done()
 if context.get('warps') is not None:
  measured=tuple(pose(update['photometric']['timing'][k]['image_from_depth']) for k in ('reference','source'))
  maximum=max(maximum,max(float(np.max(abs(a-b))) for a,b in zip(measured,context['warps'])))
  context['warps']=measured
photo.metadata=inspect
out=r/'artifacts/projective-timing/rounding-diagnostic';out.mkdir()
try:audit.recorded('tum-fr1-room-temporal',out,r/'artifacts/projective-timing/recorded')
except Done:pass
print(json.dumps(dict(frames_checked=count-1,max_independent_warp_matrix_difference=maximum,original_failure_frame_reconstructed=True)))
