"""Four-corner C1 inverse depth using fixed measured tangent planes (SI)."""
import numpy as np


def weights(t):
    t2,t3 = t*t,t*t*t
    return (np.column_stack([2*t3-3*t2+1,-2*t3+3*t2,t3-2*t2+t,t3-t2]),
            np.column_stack([6*t2-6*t,-6*t2+6*t,3*t2-4*t+1,3*t2-2*t]))


def sample(layer,query):
    c = layer['camera'];ids = np.flatnonzero((query[:,2]>.3)&np.isfinite(query).all(axis=1));q = query[ids];z = q[:,2]
    u = c['fx']*q[:,0]/z+c['cx'];v = c['fy']*q[:,1]/z+c['cy'];x,y = np.floor(u),np.floor(v)
    inside = (x>=1)&(x<c['width']-2)&(y>=1)&(y<c['height']-2)
    ids,q,z,u,v,x,y = [a[inside] for a in (ids,q,z,u,v,x,y)];x,y = x.astype(int),y.astype(int)
    depth = np.stack([np.stack([layer['depth'][y+dy,x+dx] for dx in (0,1)],axis=1) for dy in (0,1)],axis=1)
    normals = np.stack([np.stack([layer['normals'][y+dy,x+dx] for dx in (0,1)],axis=1) for dy in (0,1)],axis=1)
    size = np.linalg.norm(normals,axis=3)
    good = ((depth>.3)&(depth<5)).all(axis=(1,2))&(np.isfinite(normals).all(axis=3)&(size>.5)).all(axis=(1,2))
    good &= depth.max(axis=(1,2))-depth.min(axis=(1,2))<=.05+.01*depth.min(axis=(1,2))
    ids,q,z,u,v,x,y,depth,normals,size = [a[good] for a in (ids,q,z,u,v,x,y,depth,normals,size)]
    normal = normals/size[:,:,:,None]
    xx = x[:,None,None]+np.arange(2)[None,None,:];yy = y[:,None,None]+np.arange(2)[None,:,None]
    vertex = np.stack([np.broadcast_to((xx-c['cx'])/c['fx'],depth.shape),np.broadcast_to((yy-c['cy'])/c['fy'],depth.shape),np.ones_like(depth)],axis=3)*depth[:,:,:,None]
    offset = np.sum(normal*vertex,axis=3)
    good = (np.isfinite(offset)&(abs(offset)>1e-6)).all(axis=(1,2))
    ids,q,z,u,v,x,y,depth,normal,offset = [a[good] for a in (ids,q,z,u,v,x,y,depth,normal,offset)]
    du = normal[:,:,:,0]/(c['fx']*offset);dv = normal[:,:,:,1]/(c['fy']*offset)
    # Corner mixed derivatives are fixed to zero; shared data make adjacent
    # jointly supported bicubic cells C1, including nonintegrable normal data.
    field = np.zeros((len(ids),4,4));field[:,:2,:2] = 1/depth
    field[:,:2,2:] = du;field[:,2:,:2] = dv
    wu,gu = weights(u-x);wv,gv = weights(v-y)
    rho = np.einsum('ni,nij,nj->n',wv,field,wu)
    ru = np.einsum('ni,nij,nj->n',wv,field,gu);rv = np.einsum('ni,nij,nj->n',gv,field,wu)
    good = np.isfinite(rho)&(rho>0)&np.isfinite(ru)&np.isfinite(rv)
    ids,q,z,u,v,rho,ru,rv = [a[good] for a in (ids,q,z,u,v,rho,ru,rv)]
    gradient = np.column_stack([ru*c['fx']/z,rv*c['fy']/z,1/z**2-(ru*c['fx']*q[:,0]+rv*c['fy']*q[:,1])/z**2])
    target = np.column_stack([(u-c['cx'])/(c['fx']*rho),(v-c['cy'])/(c['fy']*rho),1/rho])
    return ids,rho-1/z,gradient,target
