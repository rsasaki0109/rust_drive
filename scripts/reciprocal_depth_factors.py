"""Symmetric measured-normal factors with exact reciprocal projective pixels."""
import numpy as np

from symmetric_depth_factors import residual


def pair(target,source,target_pose,source_pose,scale):
    stride = {4:1,2:2,1:4}[scale]
    a,b = target['surface'][scale],source['surface'][scale]
    xyz = b['vertices'][::stride,::stride].reshape(-1,3)
    normal = b['normals'][::stride,::stride].reshape(-1,3)
    y,x = np.indices(b['depth'].shape)
    pixels = np.column_stack([x[::stride,::stride].ravel(),y[::stride,::stride].ravel()])
    keep = np.sum(normal*normal,axis=1) > .25
    xyz,normal,pixels = xyz[keep],normal[keep],pixels[keep]
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
    back = (point-source_pose[:3,3])@source_pose[:3,:3]
    front = np.flatnonzero(back[:,2] > .3);camera = b['camera']
    returned = np.column_stack([np.rint(camera['fx']*back[front,0]/back[front,2]+camera['cx']),
                                np.rint(camera['fy']*back[front,1]/back[front,2]+camera['cy'])])
    selected = front[np.all(returned == pixels[ids[front]],axis=1)]
    r,j = residual(point[selected],nr[selected],q[ids[selected]],normal[ids[selected]])
    w = np.minimum(1.,.01/np.maximum(abs(r),1e-15))
    return len(r),float(r@r),j.T@(w[:,None]*j),j.T@(w*r)
