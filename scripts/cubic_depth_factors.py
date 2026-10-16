"""C1 tensor Catmull-Rom inverse-depth interpolation with complete support."""
import numpy as np


def weights(t):
    t2,t3 = t*t,t*t*t
    w = np.column_stack([-.5*t+t2-.5*t3,1-2.5*t2+1.5*t3,.5*t+2*t2-1.5*t3,-.5*t2+.5*t3])
    dw = np.column_stack([-.5+2*t-1.5*t2,-5*t+4.5*t2,.5+4*t-4.5*t2,-t+1.5*t2])
    return w,dw


def sample(layer,query):
    c = layer['camera'];ids = np.flatnonzero(query[:,2] > .3);q = query[ids];z = q[:,2]
    u = c['fx']*q[:,0]/z+c['cx'];v = c['fy']*q[:,1]/z+c['cy']
    x,y = np.floor(u),np.floor(v)
    inside = (x >= 1)&(x < c['width']-2)&(y >= 1)&(y < c['height']-2)
    ids,q,z,u,v,x,y = [a[inside] for a in (ids,q,z,u,v,x,y)];x,y = x.astype(int),y.astype(int)
    depth = np.stack([np.stack([layer['depth'][y+dy,x+dx] for dx in (-1,0,1,2)],axis=1) for dy in (-1,0,1,2)],axis=1)
    normals = np.stack([np.stack([layer['normals'][y+dy,x+dx] for dx in (-1,0,1,2)],axis=1) for dy in (-1,0,1,2)],axis=1)
    good = ((depth > .3)&(depth < 5)).all(axis=(1,2))
    good &= (np.sum(normals*normals,axis=3) > .25).all(axis=(1,2))
    for dy in range(3):
        for dx in range(3):
            patch = depth[:,dy:dy+2,dx:dx+2];lo = patch.min(axis=(1,2));hi = patch.max(axis=(1,2))
            good &= hi-lo <= .05+.01*lo
    ids,q,z,u,v,x,y,depth = [a[good] for a in (ids,q,z,u,v,x,y,depth)]
    wu,du = weights(u-x);wv,dv = weights(v-y);inverse = 1/depth
    rho = np.einsum('ni,nij,nj->n',wv,inverse,wu)
    ru = np.einsum('ni,nij,nj->n',wv,inverse,du)
    rv = np.einsum('ni,nij,nj->n',dv,inverse,wu)
    good = np.isfinite(rho)&(rho > 0)
    ids,q,z,u,v,rho,ru,rv = [a[good] for a in (ids,q,z,u,v,rho,ru,rv)]
    gradient = np.column_stack([ru*c['fx']/z,rv*c['fy']/z,
        1/z**2-(ru*c['fx']*q[:,0]+rv*c['fy']*q[:,1])/z**2])
    target = np.column_stack([(u-c['cx'])/(c['fx']*rho),(v-c['cy'])/(c['fy']*rho),1/rho])
    return ids,rho-1/z,gradient,target
