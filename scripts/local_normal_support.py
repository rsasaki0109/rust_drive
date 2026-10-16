"""Complete only absent sampling normals from fully measured 3x3 planes (SI)."""
import numpy as np

from normal_hermite_depth_factors import sample as original_sample
from projective_depth_common import require


def complete(layer):
    """Keep original normals, depths, vertices and two-pixel border unchanged.

    A missing interior normal gets one candidate: all nine 3x3 depths must
    satisfy the original center-depth gate, and covariance must pass the same
    middle-eigenvalue and plane-ratio tests as the 5x5 normal frontend.
    """
    depth,xyz = layer['depth'],layer['vertices'];result = layer['normals'].copy()
    center = depth[2:-2,2:-2];origin = xyz[2:-2,2:-2];h,w = center.shape
    valid = (center>.3)&(center<5)&(np.linalg.norm(result[2:-2,2:-2],axis=2)<=.5)
    first = np.zeros((h,w,3));second = np.zeros((h,w,3,3))
    for dy in (1,2,3):
        for dx in (1,2,3):
            z = depth[dy:dy+h,dx:dx+w];offset = xyz[dy:dy+h,dx:dx+w]-origin
            valid &= (z>.3)&(z<5)&(abs(z-center)<.05+.01*center)
            first += offset;second += np.einsum('...i,...j->...ij',offset,offset)
    mean = first[valid]/9;covariance = second[valid]/9-np.einsum('ni,nj->nij',mean,mean)
    eigen,basis = np.linalg.eigh(covariance)
    planar = (eigen[:,1]>1e-8)&(eigen[:,0]<=.1*eigen[:,1])
    normal = basis[:,:,0];normal *= np.where(np.sum(normal*origin[valid],axis=1)>=0,1.,-1.)[:,None]
    normal[~planar] = 0.;result[2:-2,2:-2][valid] = normal
    return result


def prepare(nodes):
    return [dict(node,surface={scale:dict(layer,hermite_normals=complete(layer)) for scale,layer in node['surface'].items()}) for node in nodes]


def sample(layer,query):
    require('hermite_normals' in layer,'prepared sampling-normal field required')
    return original_sample(dict(layer,normals=layer['hermite_normals']),query)
