"""Continuous inverse-depth factors for the optional fixed-window experiment.

The residual and its exact projective derivative are converted to metres by a
normalizer frozen at each linearization. This is an engineering weight, without
a calibrated noise or covariance interpretation.
"""
import numpy as np


def sample(layer, query):
    """Return supported source indices, F, dF/dq and interpolated target XYZ."""
    c = layer['camera']
    ids = np.flatnonzero(query[:,2] > .3)
    q = query[ids];z = q[:,2]
    u = c['fx']*q[:,0]/z+c['cx'];v = c['fy']*q[:,1]/z+c['cy']
    u = np.where(abs(u-np.rint(u)) <= 1e-10,np.rint(u),u)
    v = np.where(abs(v-np.rint(v)) <= 1e-10,np.rint(v),v)
    x,y = np.floor(u),np.floor(v)
    good = (x >= 1)&(x < c['width']-2)&(y >= 1)&(y < c['height']-2)
    ids,q,z,u,v,x,y = [a[good] for a in (ids,q,z,u,v,x,y)]
    x,y = x.astype(int),y.astype(int)
    offsets = ((0,0),(0,1),(1,0),(1,1))
    depths = np.stack([layer['depth'][y+dy,x+dx] for dy,dx in offsets],axis=1)
    normals = np.stack([layer['normals'][y+dy,x+dx] for dy,dx in offsets],axis=1)
    lo,hi = depths.min(axis=1),depths.max(axis=1)
    good = ((lo > .3)&(hi < 5)&(hi-lo <= .05+.01*lo)
            &np.all(np.sum(normals*normals,axis=2) > .25,axis=1))
    ids,q,z,u,v,x,y,depths = [a[good] for a in (ids,q,z,u,v,x,y,depths)]
    a,b,cc,d = (1/depths).T;dx,dy = u-x,v-y
    rho = (1-dx)*(1-dy)*a+dx*(1-dy)*b+(1-dx)*dy*cc+dx*dy*d
    du = (1-dy)*(b-a)+dy*(d-cc)
    dv = (1-dx)*(cc-a)+dx*(d-b)
    derivative = np.column_stack([du*c['fx']/z,dv*c['fy']/z,
        1/z**2-(du*c['fx']*q[:,0]+dv*c['fy']*q[:,1])/z**2])
    target = np.column_stack([(u-c['cx'])/(c['fx']*rho),
                              (v-c['cy'])/(c['fy']*rho),1/rho])
    return ids,rho-1/z,derivative,target


def pair(target,source,target_pose,source_pose,scale):
    stride = {4:1,2:2,1:4}[scale]
    layer = source['surface'][scale]
    xyz = layer['vertices'][::stride,::stride].reshape(-1,3)
    normal = layer['normals'][::stride,::stride].reshape(-1,3)
    good = np.sum(normal*normal,axis=1) > .25
    xyz,normal = xyz[good],normal[good]
    root = xyz@source_pose[:3,:3].T+source_pose[:3,3]
    source_normal = normal@source_pose[:3,:3].T
    local = (root-target_pose[:3,3])@target_pose[:3,:3]
    ids,f,df,point = sample(target['surface'][scale],local)
    length = np.linalg.norm(df,axis=1)
    n = df/length[:,None];nr = n@target_pose[:3,:3].T
    good = ((np.linalg.norm(local[ids]-point,axis=1) < (.12 if scale == 4 else .08))
            &(np.sum(source_normal[ids]*nr,axis=1) > .8))
    ids,nr,r = ids[good],nr[good],(f/length)[good]
    j = np.column_stack([nr,np.cross(root[ids],nr)])
    w = np.minimum(1.,.01/np.maximum(abs(r),1e-15))
    return len(r),float(r@r),j.T@(w[:,None]*j),j.T@(w*r)
