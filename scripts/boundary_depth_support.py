"""Conservative support at nearly integer image coordinates; no snapping."""
import numpy as np

from one_sided_depth_slopes import prepare as original_prepare,sample as original_sample
from projective_depth_common import require

EPSILON_PIXELS = 1e-9


def cells(layer):
    z = layer['depth'];slopes = layer['depth_slopes'];h,w = z.shape
    good = np.ones((h-1,w-1),dtype=bool);low = np.full((h-1,w-1),np.inf);high = np.full((h-1,w-1),-np.inf)
    for dy in (0,1):
        for dx in (0,1):
            d = z[dy:dy+h-1,dx:dx+w-1]
            good &= (d>.3)&(d<5)&slopes['valid'][dy:dy+h-1,dx:dx+w-1]
            good &= np.isfinite(slopes['du'][dy:dy+h-1,dx:dx+w-1])&np.isfinite(slopes['dv'][dy:dy+h-1,dx:dx+w-1])
            low = np.minimum(low,d);high = np.maximum(high,d)
    good &= high-low<=.05+.01*low
    result = np.zeros((h,w),dtype=bool);result[:-1,:-1] = good
    result[0,:] = result[:,0] = False;result[h-2:,:] = False;result[:,w-2:] = False
    return result


def prepare(nodes):
    return [dict(n,surface={s:dict(layer,boundary_cells=cells(layer)) for s,layer in n['surface'].items()}) for n in original_prepare(nodes)]


def mask(layer,query):
    require('boundary_cells' in layer,'prepared boundary cell support required')
    result = np.zeros(len(query),dtype=bool);ids = np.flatnonzero(np.isfinite(query).all(axis=1)&(query[:,2]>.3))
    q = query[ids];c = layer['camera']
    with np.errstate(over='ignore',invalid='ignore'):
        u = c['fx']*q[:,0]/q[:,2]+c['cx'];v = c['fy']*q[:,1]/q[:,2]+c['cy']
    inside = np.isfinite(u)&np.isfinite(v)&(u>0)&(u<c['width'])&(v>0)&(v<c['height'])
    ids,u,v = ids[inside],u[inside],v[inside]
    lo_x,hi_x = np.floor(u-EPSILON_PIXELS).astype(int),np.floor(u+EPSILON_PIXELS).astype(int)
    lo_y,hi_y = np.floor(v-EPSILON_PIXELS).astype(int),np.floor(v+EPSILON_PIXELS).astype(int)
    valid = np.ones(len(ids),dtype=bool)
    for xx,yy in ((lo_x,lo_y),(lo_x,hi_y),(hi_x,lo_y),(hi_x,hi_y)):
        bounded = (xx>=1)&(xx<c['width']-2)&(yy>=1)&(yy<c['height']-2)
        available = np.zeros(len(ids),dtype=bool);available[bounded] = layer['boundary_cells'][yy[bounded],xx[bounded]]
        valid &= available
    result[ids] = valid
    return result


def sample(layer,query):
    ids = np.flatnonzero(mask(layer,query));chosen,r,df,target = original_sample(layer,query[ids])
    return ids[chosen],r,df,target
