import sys, json, hashlib
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path('/workspace/rust_drive')
sys.path.insert(0,str(ROOT/'scripts'))
from projective_depth_common import CAMERA, sha, write
from projective_depth_raw import pyramid

out=ROOT/'artifacts/projective-normal'
out.mkdir(exist_ok=True)
def wide(depth,c):
    windows=np.lib.stride_tricks.sliding_window_view(depth,(5,5))
    lo,hi=windows.min(axis=(-2,-1)),windows.max(axis=(-2,-1))
    center=depth[2:-2,2:-2]
    valid=(lo>.3)&(hi<5)&(np.max(abs(windows-center[:,:,None,None]),axis=(-2,-1))<.05+.01*center)
    inv=1/np.maximum(windows,1e-15)
    dx=np.arange(-2,3)
    a=np.sum(inv*dx,axis=(-2,-1))/50
    b=np.sum(inv*dx[:,None],axis=(-2,-1))/50
    g=inv.mean(axis=(-2,-1))
    y,x=np.mgrid[2:depth.shape[0]-2,2:depth.shape[1]-2]
    n=np.stack([c['fx']*a,c['fy']*b,g-(x-c['cx'])*a-(y-c['cy'])*b],axis=-1)
    n/=np.maximum(np.linalg.norm(n,axis=-1),1e-15)[:,:,None]
    normal=np.zeros((*depth.shape,3));normal[2:-2,2:-2]=n*valid[:,:,None]
    # Plane residual on the same observed support, for a noise diagnostic only.
    fitted=g[:,:,None,None]+a[:,:,None,None]*dx+b[:,:,None,None]*dx[:,None]
    rms=np.sqrt(np.mean(((1/np.maximum(fitted,1e-15))-windows)**2,axis=(-2,-1)))
    mask=np.zeros_like(depth,dtype=bool);mask[2:-2,2:-2]=valid&(rms<=.002)
    return normal,mask

cases={}
for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
    base=ROOT/'data'/case;manifest=json.loads((base/'manifest.json').read_bytes())
    frames=[]
    for i in range(0,180,15):
        f=manifest['frames'][i];body=(base/'raw'/f['depth_file']).read_bytes()
        pin=next(v for v in manifest['files'] if v['file']==f['depth_file'])
        assert len(body)==pin['bytes'] and sha(body)==pin['sha256']
        d=np.asarray(Image.open(base/'raw'/f['depth_file']),dtype=float)/5000
        original=pyramid(d,CAMERA)[1]['normals'];candidate,mask=wide(d,CAMERA)
        common=mask[:,1:]&mask[:,:-1]&(np.sum(original[:,1:]**2,axis=-1)>.25)&(np.sum(original[:,:-1]**2,axis=-1)>.25)
        def angles(n):
            return np.arccos(np.clip(np.sum(n[:,1:]*n[:,:-1],axis=-1)[common],-1,1))
        a,b=angles(original),angles(candidate)
        frames.append(dict(source_index=f['source_index'],depth_sha256=pin['sha256'],pairs=len(a),
            central_median_rad=float(np.median(a)),plane_median_rad=float(np.median(b)),
            central_p90_rad=float(np.percentile(a,90)),plane_p90_rad=float(np.percentile(b,90))))
    cases[case]=frames
result=dict(hypothesis='one-pixel normals amplify depth noise; a five-by-five inverse-depth plane reduces local normal variation',
    ground_truth_decoded=False,selection='12 evenly spaced viewed frames per case; common adjacent valid normals on five-by-five patches with fitted depth RMS <= 2 mm',
    limitation='same-support plane residual selection favors planar patches; normal consistency is not pose accuracy or proof of drift cause',cases=cases)
write(out/'diagnostic.json',result)
for k,v in cases.items():
    print(k,json.dumps({name:float(np.median([f[name] for f in v])) for name in ('central_median_rad','plane_median_rad','central_p90_rad','plane_p90_rad')}))
