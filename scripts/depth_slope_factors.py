"""C1 inverse depth with shared, limited depth-difference vertex slopes."""
import numpy as np

from normal_hermite_depth_factors import weights
from projective_depth_common import require


def limited(a,b):
    result = np.zeros_like(a);same = ((a>0)&(b>0))|((a<0)&(b<0))
    result[same] = 2*a[same]*b[same]/(a[same]+b[same])
    return result


def field(layer):
    depth = layer['depth'];center = depth[2:-2,2:-2];h,w = center.shape
    valid = (center>.3)&(center<5);rho = np.zeros_like(depth)
    np.divide(1.,depth,out=rho,where=(depth>.3)&(depth<5))
    for dy,dx in ((2,1),(2,3),(1,2),(3,2)):
        z = depth[dy:dy+h,dx:dx+w]
        valid &= (z>.3)&(z<5)&(abs(z-center)<.05+.01*center)
    du,dv = np.zeros_like(depth),np.zeros_like(depth)
    du[2:-2,2:-2] = limited(rho[2:-2,2:-2]-rho[2:-2,1:-3],rho[2:-2,3:-1]-rho[2:-2,2:-2])
    dv[2:-2,2:-2] = limited(rho[2:-2,2:-2]-rho[1:-3,2:-2],rho[3:-1,2:-2]-rho[2:-2,2:-2])
    support = np.zeros_like(depth,dtype=bool);support[2:-2,2:-2] = valid
    du[~support] = 0.;dv[~support] = 0.
    return dict(du=du,dv=dv,valid=support)


def prepare(nodes):
    return [dict(node,surface={s:dict(layer,depth_slopes=field(layer)) for s,layer in node['surface'].items()}) for node in nodes]


def sample(layer,query):
    require('depth_slopes' in layer,'prepared depth-slope field required')
    c = layer['camera'];slopes = layer['depth_slopes']
    ids = np.flatnonzero((query[:,2]>.3)&np.isfinite(query).all(axis=1));q = query[ids];z = q[:,2]
    u = c['fx']*q[:,0]/z+c['cx'];v = c['fy']*q[:,1]/z+c['cy'];x,y = np.floor(u),np.floor(v)
    inside = (x>=1)&(x<c['width']-2)&(y>=1)&(y<c['height']-2)
    ids,q,z,u,v,x,y = [a[inside] for a in (ids,q,z,u,v,x,y)];x,y = x.astype(int),y.astype(int)
    xx = x[:,None,None]+np.arange(2)[None,None,:];yy = y[:,None,None]+np.arange(2)[None,:,None]
    depth = layer['depth'][yy,xx];du = slopes['du'][yy,xx];dv = slopes['dv'][yy,xx]
    good = slopes['valid'][yy,xx].all(axis=(1,2))&((depth>.3)&(depth<5)).all(axis=(1,2))
    good &= depth.max(axis=(1,2))-depth.min(axis=(1,2))<=.05+.01*depth.min(axis=(1,2))
    ids,q,z,u,v,x,y,depth,du,dv = [a[good] for a in (ids,q,z,u,v,x,y,depth,du,dv)]
    data = np.zeros((len(ids),4,4));data[:,:2,:2] = 1/depth;data[:,:2,2:] = du;data[:,2:,:2] = dv
    wu,gu = weights(u-x);wv,gv = weights(v-y)
    rho = np.einsum('ni,nij,nj->n',wv,data,wu)
    ru = np.einsum('ni,nij,nj->n',wv,data,gu);rv = np.einsum('ni,nij,nj->n',gv,data,wu)
    good = np.isfinite(rho)&(rho>0)&np.isfinite(ru)&np.isfinite(rv)
    ids,q,z,u,v,rho,ru,rv = [a[good] for a in (ids,q,z,u,v,rho,ru,rv)]
    gradient = np.column_stack([ru*c['fx']/z,rv*c['fy']/z,1/z**2-(ru*c['fx']*q[:,0]+rv*c['fy']*q[:,1])/z**2])
    target = np.column_stack([(u-c['cx'])/(c['fx']*rho),(v-c['cy'])/(c['fy']*rho),1/rho])
    return ids,rho-1/z,gradient,target
