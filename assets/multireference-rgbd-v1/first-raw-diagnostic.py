from pathlib import Path
import sys,json,io,importlib.util
sys.path.insert(0,str(Path('scripts').absolute()))
import numpy as np
from PIL import Image
from multireference_rgbd_common import raw_reference,raw_equation,compare_raw
from projective_depth_common import CAMERA,pose
case='tum-fr1-room-temporal';folder=Path('data')/case
frames=json.loads((folder/'manifest.json').read_text())['frames']
rows=json.loads((Path('artifacts/multireference-rgbd-dev/rust-first')/(case+'-phase-gated-rgbd')/'sensor-only.json').read_text())['frames']
raw={}
for i,(frame,row) in enumerate(zip(frames,rows)):
 d=np.asarray(Image.open(folder/'raw'/frame['depth_file']),dtype=float)/5000
 color=np.asarray(Image.open(folder/'raw'/frame['rgb_file']),dtype=np.uint32);gray=((77*color[:,:,0]+150*color[:,:,1]+29*color[:,:,2])>>8).astype(np.uint8)
 source=raw_reference(d,gray,CAMERA,np.eye(4),frame['rgb_timestamp']-frame['depth_timestamp'])
 u=row['update'];refs=[raw[t] for t in u['reference_timestamps']]
 for k,s in enumerate(u['iterations']):
  expected=raw_equation(refs,source,pose(s['before']),s['scale'],u['photo_enabled'])
  try:compare_raw(s['equation'],expected)
  except (AssertionError,ValueError) as error:
   print('FIRST_MISMATCH',i,k,s['scale'],str(error),flush=True)
   print('actual',s['equation']['reference_support'],s['equation']['reference_photo_support'],'expected',expected['reference_support'],expected['reference_photo_support'],flush=True)
   raise
 if u['accepted']:
  source['root']=pose(u['first_from_current']);raw[frame['depth_timestamp']]=source
 if i%10==0:print('raw checked',i,flush=True)
 if i==30:break
