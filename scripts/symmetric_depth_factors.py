"""Measured symmetric point-to-plane factors, with rotating-normal derivatives."""
import numpy as np


def residual(target_points,target_normals,source_points,source_normals):
    """Fixed point correspondences, in the common first optical-camera frame."""
    summed = target_normals+source_normals
    length = np.linalg.norm(summed,axis=1)
    normal = summed/length[:,None]
    difference = source_points-target_points
    r = np.einsum('ij,ij->i',normal,difference)
    tangent = (difference-r[:,None]*normal)/length[:,None]
    j = np.column_stack([normal,np.cross(source_points,normal)+np.cross(source_normals,tangent)])
    return r,j


def pair(target,source,target_pose,source_pose,scale):
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
    r,j = residual(point,nr,q[ids],normal[ids])
    w = np.minimum(1.,.01/np.maximum(abs(r),1e-15))
    return len(r),float(r@r),j.T@(w[:,None]*j),j.T@(w*r)
