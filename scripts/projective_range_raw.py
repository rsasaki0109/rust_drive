"""Independent forward-pair sampling and relative axial-range weighting oracle."""
import numpy as np
from projective_depth_common import require


def pairs(target,source,transform,scale):
    stride=2 if scale==1 else 1
    ps=source['vertices'][::stride,::stride].reshape(-1,3)
    ns=source['normals'][::stride,::stride].reshape(-1,3)
    valid=np.sum(ns**2,axis=1)>.25
    ps,ns=ps[valid],ns[valid]
    query=ps@transform[:3,:3].T+transform[:3,3]
    ns=ns@transform[:3,:3].T
    valid=np.isfinite(query).all(axis=1)&(query[:,2]>.3)
    ps,query,ns=ps[valid],query[valid],ns[valid]
    c=target['camera']
    u=np.rint(c['fx']*query[:,0]/query[:,2]+c['cx'])
    v=np.rint(c['fy']*query[:,1]/query[:,2]+c['cy'])
    valid=(u>=1)&(u<c['width']-1)&(v>=1)&(v<c['height']-1)
    ps,query,ns=ps[valid],query[valid],ns[valid]
    u,v=u[valid].astype(np.int64),v[valid].astype(np.int64)
    pt,nt=target['vertices'][v,u],target['normals'][v,u]
    valid=(np.sum(nt**2,axis=1)>.25)&(np.linalg.norm(query-pt,axis=1)<(.12 if scale==4 else .08))
    valid&=np.einsum('ij,ij->i',ns,nt)>.8
    ps,query,pt,nt=ps[valid],query[valid],pt[valid],nt[valid]
    residual=np.einsum('ij,ij->i',nt,query-pt)
    return dict(source=ps,query=query,target=pt,normal=nt,residual=residual,
                jacobian=np.column_stack([nt,np.cross(query,nt)]))


def coefficient(source_z,target_z):
    # Axial z=fb/disparity gives Var(z) proportional to z^4 under constant
    # disparity noise. This is only a bounded, relative engineering hypothesis.
    return 1/np.maximum(1.,(np.asarray(source_z)**4+np.asarray(target_z)**4)/2)


def equation(samples,weighted):
    residual=samples['residual'];j=samples['jacobian']
    weight=np.minimum(1.,.01/np.maximum(abs(residual),1e-15))
    if weighted:weight*=coefficient(samples['source'][:,2],samples['target'][:,2])
    return dict(count=len(residual),rms_m=float(np.sqrt(np.mean(residual**2))) if len(residual) else 0.,
                h=j.T@(weight[:,None]*j),g=j.T@(weight*residual))


def controls():
    z=np.array([.3,1.,2.,4.,5.])
    np.testing.assert_allclose(coefficient(z,z),[1.,1.,1/16,1/256,1/625],atol=0,rtol=1e-15)
    require(coefficient(1.,3.)==1/41,'two endpoint fourth-power variance average')
    np.testing.assert_array_equal(coefficient(z,z[::-1]),coefficient(z[::-1],z))
    return dict(metre_floor=True,quadratic_depth_standard_deviation=True,
                symmetric_endpoint_variance=True,bounded_relative_weight=[1/625,1.],
                calibrated_noise_or_covariance=False)
