"""Tukey-weighted symmetric depth factors for optional offline research."""
import numpy as np
from symmetric_depth_factors import residual


def correspondences(target,source,target_pose,source_pose,scale):
    stride = {4:1,2:2,1:4}[scale]
    a,b = target['surface'][scale],source['surface'][scale]
    xyz = b['vertices'][::stride,::stride].reshape(-1,3)
    normal = b['normals'][::stride,::stride].reshape(-1,3)
    keep = np.sum(normal*normal,axis=1) > .25
    xyz,normal = xyz[keep],normal[keep]
    q = xyz@source_pose[:3,:3].T+source_pose[:3,3]
    normal = normal@source_pose[:3,:3].T
    local = (q-target_pose[:3,3])@target_pose[:3,:3]
    ids = np.flatnonzero(local[:,2] > .3)
    c = a['camera']
    u = np.rint(c['fx']*local[ids,0]/local[ids,2]+c['cx'])
    v = np.rint(c['fy']*local[ids,1]/local[ids,2]+c['cy'])
    inside = (u >= 1)&(u < c['width']-1)&(v >= 1)&(v < c['height']-1)
    ids,u,v = ids[inside],u[inside].astype(int),v[inside].astype(int)
    n = a['normals'][v,u];nr = n@target_pose[:3,:3].T
    valid = ((np.sum(n*n,axis=1) > .25)
             &(np.linalg.norm(local[ids]-a['vertices'][v,u],axis=1) < (.12 if scale == 4 else .08))
             &(np.sum(normal[ids]*nr,axis=1) > .8))
    ids,nr,u,v = ids[valid],nr[valid],u[valid],v[valid]
    point = a['vertices'][v,u]@target_pose[:3,:3].T+target_pose[:3,3]
    return point,nr,q[ids],normal[ids]


def weights(r,scale):
    cutoff = .03*scale
    ratio = np.abs(r)/cutoff
    return np.where(ratio < 1.,(1-ratio*ratio)**2,0.)


def pair(target,source,target_pose,source_pose,scale):
    r,j = residual(*correspondences(target,source,target_pose,source_pose,scale))
    w = weights(r,scale)
    if float(w.sum()) < 200.:return 0,0.,np.zeros((6,6)),np.zeros(6)
    return len(r),float(r@r),j.T@(w[:,None]*j),j.T@(w*r)
