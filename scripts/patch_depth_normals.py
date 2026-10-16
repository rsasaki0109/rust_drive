"""5x5 local-plane normals; original axial depths and vertices remain untouched."""
import numpy as np

from projective_depth_raw import pyramid as depth_pyramid


def normals(depth, xyz):
    """Population covariance of gated offsets, smallest eigenvector toward view ray.

    Optical x right, y down, z forward; depths/coordinates are metres. A normal
    requires >=13 of 25 samples, two-dimensional spread and a plane-like patch.
    The two-pixel image border has no normal. No depth is filled or averaged here.
    """
    result = np.zeros_like(xyz)
    center = depth[2:-2,2:-2];origin = xyz[2:-2,2:-2]
    h,w = center.shape
    count = np.zeros((h,w),dtype=int)
    first = np.zeros((h,w,3));second = np.zeros((h,w,3,3))
    for dy in range(5):
        for dx in range(5):
            z = depth[dy:dy+h,dx:dx+w]
            valid = (z > .3)&(z < 5)&(abs(z-center) < .05+.01*center)
            offset = np.where(valid[:,:,None],xyz[dy:dy+h,dx:dx+w]-origin,0.)
            count += valid;first += offset
            second += np.einsum('...i,...j->...ij',offset,offset)
    eligible = (center > .3)&(center < 5)&(count >= 13)
    mean = first[eligible]/count[eligible,None]
    covariance = second[eligible]/count[eligible,None,None]-np.einsum('ni,nj->nij',mean,mean)
    eigen,basis = np.linalg.eigh(covariance)
    planar = (eigen[:,1] > 1e-8)&(eigen[:,0] <= .1*eigen[:,1])
    normal = basis[:,:,0]
    normal *= np.where(np.sum(normal*origin[eligible],axis=1) >= 0,1.,-1.)[:,None]
    normal[~planar] = 0.
    result[2:-2,2:-2][eligible] = normal
    return result


def pyramid(raw, camera):
    layers = depth_pyramid(raw,camera)
    for layer in layers.values():
        layer['normals'] = normals(layer['depth'],layer['vertices'])
    return layers
