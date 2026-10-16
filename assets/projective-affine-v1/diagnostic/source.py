import sys,json,io
from pathlib import Path
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path.cwd()/'scripts'))
import projective_photo_raw as photo
from projective_depth_raw import pyramid,raster,fuse
from projective_depth_common import CAMERA,pose,write
text=Path('scripts/projective_photo_raw.py').read_text().split('\ndef compare')[0]
text=text.replace('    r=value-color;valid=',"    out['diagnostic'] = dict(samples=len(value),mean=float(np.mean(value-color)),median=float(np.median(value-color)),rms=float(np.sqrt(np.mean((value-color)**2))),reference_mean=float(np.mean(value)),source_mean=float(np.mean(color)),gain_offset=np.linalg.lstsq(np.column_stack([color,np.ones(len(color))]),value,rcond=None)[0].tolist())\n    r=value-color;valid=")
namespace={};exec(text,namespace)
allrows={}
for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
 frames=json.loads((Path('data')/case/'manifest.json').read_text())['frames'];rows=json.loads((Path('artifacts/projective-rgbd/maintained')/case/'results.json').read_text())['frames']; key=np.eye(4);model=weights=color=None;summary=[]
 for index,(frame,row) in enumerate(zip(frames,rows)):
  folder=Path('data')/case/'raw';raw=np.asarray(Image.open(folder/frame['depth_file']),dtype=float)/5000
  rgb=np.asarray(Image.open(folder/frame['rgb_file']),dtype=np.uint32);gray=((77*rgb[:,:,0]+150*rgb[:,:,1]+29*rgb[:,:,2])>>8).astype(np.uint8)
  u=row['update']
  if not index:model=raw.copy();weights=(model>.3).astype(np.uint8);color=gray;continue
  root=pose(u['first_from_current']);relative=np.linalg.inv(key)@root
  if color is not None and u['photometric']['phase_qualified'] and u['photometric']['fresh']:
   target=pyramid(model,CAMERA);source=pyramid(raw,CAMERA);tg=photo.gray_pyramid(color);sg=photo.gray_pyramid(gray)
   samples=[]
   for scale in (4,2,1):
    steps=[s for s in u['iterations'] if s['scale']==scale]
    s=steps[0];p=namespace['equation'](target[scale],source[scale],tg[scale],sg[scale],pose(s['before']),scale,True)
    h=np.asarray(s['equation']['h']);g=np.asarray(s['equation']['g']);ph=p['scale_m2_per_gray2']*p['h'];pg=p['scale_m2_per_gray2']*p['g'];d=np.asarray(s['delta']);gd=np.linalg.solve(h,-g)
    samples.append(dict(scale=scale,**p['diagnostic'],geometry_delta=gd.tolist(),joint_delta=d.tolist(),h_ratio=(np.diag(ph)/np.diag(h)).tolist()))
   summary.append(dict(source_index=frame['source_index'],position_error_m=row['position_error_m'],rotation_error_rad=row['rotation_error_rad'],phase=frame['rgb_timestamp']-frame['depth_timestamp'],samples=samples))
  if u['keyframe_replaced']:
   model,weights,_=raster(model,weights,np.linalg.inv(relative),CAMERA);relative=np.eye(4);key=root;color=gray if u['photometric']['fresh'] else None
  model,weights,_=fuse(model,weights,raw,relative,CAMERA)
 allrows[case]=summary
 print(case,'qualified',len(summary),flush=True)
 for scale in (4,2,1):
  s=[r['samples'][(4,2,1).index(scale)] for r in summary];print(scale,'abs mean median/max',np.median([abs(x['mean']) for x in s]),max(abs(x['mean']) for x in s),'gain range',min(x['gain_offset'][0] for x in s),max(x['gain_offset'][0] for x in s),'photo/geo H diagonal median',np.median([x['h_ratio'] for x in s],axis=0).tolist(),flush=True)
 for r in summary:
  if r['source_index'] in (160,165,166,167,168,169,170,175,180,200,210,220):print(r['source_index'],[(s['scale'],round(s['mean'],2),[round(x,3) for x in s['gain_offset']],round(np.linalg.norm(np.asarray(s['joint_delta'])[3:]-np.asarray(s['geometry_delta'])[3:]),4)) for s in r['samples']],flush=True)
write(Path('artifacts/projective-photo-diagnostic/diagnostic.json'),allrows)
