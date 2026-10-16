"""Independent reciprocal pixel indexing, camera matrices and normal differential."""
import numpy as np
from projective_depth_common import require


def cross_matrices(v):
    # Each matrix maps w to v cross w, independent of the factor's cross-product J.
    result = np.zeros((len(v),3,3))
    result[:,0,1],result[:,0,2] = -v[:,2],v[:,1]
    result[:,1,0],result[:,1,2] = v[:,2],-v[:,0]
    result[:,2,0],result[:,2,1] = -v[:,1],v[:,0]
    return result


def independent_raw(references,source,pose,scale,enabled):
    require(len(references) == 1 and not enabled,'one geometric pair without RGB')
    ref = references[0];a,b = ref['surface'][scale],source['surface'][scale]
    stride = {4:1,2:2,1:4}[scale];pixels = np.arange(b['depth'].size).reshape(b['depth'].shape)[::stride,::stride].ravel()
    xyz,normals = b['vertices'].reshape(-1,3)[pixels],b['normals'].reshape(-1,3)[pixels]
    ok = np.linalg.norm(normals,axis=1) > .5;xyz,normals,pixels = xyz[ok],normals[ok],pixels[ok]
    relative = np.linalg.solve(ref['root'],pose)
    q = xyz@relative[:3,:3].T+relative[:3,3]
    sn = normals@relative[:3,:3].T
    ids = np.flatnonzero(q[:,2] > .3);c = a['camera']
    k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    image = q[ids]@k.T;uv = np.round(image[:,:2]/image[:,2,None])
    inside = ((uv >= 1).all(axis=1)&(uv[:,0] < c['width']-1)&(uv[:,1] < c['height']-1))
    ids,uv = ids[inside],uv[inside].astype(int);u,v = uv.T
    point,tn = a['vertices'][v,u],a['normals'][v,u]
    ok = (np.linalg.norm(tn,axis=1) > .5)
    ok &= np.sqrt(np.sum((q[ids]-point)**2,axis=1)) < (.12 if scale == 4 else .08)
    ok &= np.einsum('ij,ij->i',sn[ids],tn) > .8
    ids,point,tn = ids[ok],point[ok],tn[ok]
    forward_count = len(ids)
    inverse = np.linalg.inv(relative)
    back = point@inverse[:3,:3].T+inverse[:3,3]
    front = np.flatnonzero(back[:,2] > .3)
    camera = b['camera'];matrix = np.array([[camera['fx'],0,camera['cx']],[0,camera['fy'],camera['cy']],[0,0,1.]])
    projected = back[front]@matrix.T
    returned = np.round(projected[:,:2]/projected[:,2,None])
    expected = np.column_stack([pixels[ids[front]]%camera['width'],pixels[ids[front]]//camera['width']])
    chosen = front[np.logical_and.reduce(returned == expected,axis=1)]
    ids,point,tn = ids[chosen],point[chosen],tn[chosen]
    summed = sn[ids]+tn;length = np.sqrt(np.sum(summed*summed,axis=1));n = summed/length[:,None]
    difference = q[ids]-point;r = np.sum(n*difference,axis=1)
    # Differentiate query and normalized measured-normal sum by matrix products.
    rot = ref['root'][:3,:3]
    root_points = xyz[ids]@pose[:3,:3].T+pose[:3,3]
    root_normals = normals[ids]@pose[:3,:3].T
    dq = -np.einsum('ij,njk->nik',rot.T,cross_matrices(root_points))
    dn_sum = -np.einsum('ij,njk->nik',rot.T,cross_matrices(root_normals))
    dn = np.eye(3)[None,:,:]-np.einsum('ni,nj->nij',n,n)
    dn = np.einsum('nij,njk->nik',dn,dn_sum)/length[:,None,None]
    jr = np.einsum('ni,nij->nj',n,dq)+np.einsum('ni,nij->nj',difference,dn)
    j = np.concatenate([n@rot.T,jr],axis=1)
    w = np.where(abs(r) <= .01,1.,.01/np.maximum(abs(r),1e-15))
    return dict(selection=dict(forward_count=forward_count,reciprocal_count=len(r)),geometry=dict(count=len(r),rms_m=float(np.sqrt(np.mean(r*r))) if len(r) else 0.,
                             h=(j*w[:,None]).T@j,g=j.T@(w*r)))

