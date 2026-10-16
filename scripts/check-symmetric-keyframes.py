#!/usr/bin/env python3
"""Independent normalized-normal differential, pose differences and prior audit."""
import argparse
import importlib.util
from pathlib import Path

import numpy as np

import symmetric_keyframe_depth as kernel
from projective_depth_common import ROOT, fresh, require, sha, write
from projective_depth_raw import pyramid


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
    ok = np.linalg.norm(normals,axis=1) > .5;xyz,normals = xyz[ok],normals[ok]
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
    return dict(geometry=dict(count=len(r),rms_m=float(np.sqrt(np.mean(r*r))) if len(r) else 0.,
                             h=(j*w[:,None]).T@j,g=j.T@(w*r)))


def auditor():
    spec = importlib.util.spec_from_file_location('symmetric_prior_audit',ROOT/'scripts/check-marginalized-keyframes.py')
    audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    audit.kernel = kernel;audit.raw_equation = independent_raw
    return audit


def factor_controls(audit):
    from symmetric_depth_factors import residual,pair
    from joint_keyframe_depth import pair as original_pair
    target = audit.increment(np.eye(4),np.array([.4,-.1,.2,.1,-.04,.03]))
    source = audit.increment(np.eye(4),np.array([.44,-.11,.22,.12,-.03,.01]))
    pt = np.array([[.1,-.2,2.],[.2,.1,2.5]])
    nt = np.array([[.1,.2,1.],[-.2,.1,1.]])
    ns = np.array([[.15,.12,1.],[-.13,.18,1.]])
    nt /= np.linalg.norm(nt,axis=1)[:,None];ns /= np.linalg.norm(ns,axis=1)[:,None]
    ps = pt+np.array([[.02,-.01,.006],[-.01,.02,-.005]])
    def arrays(t,s):
        return pt@t[:3,:3].T+t[:3,3],nt@t[:3,:3].T,ps@s[:3,:3].T+s[:3,3],ns@s[:3,:3].T
    def value(t,s):
        p,n,q,m = arrays(t,s);summed = n+m
        return np.sum(summed*(q-p),axis=1)/np.linalg.norm(summed,axis=1)
    points = arrays(target,source);r,j = residual(*points)
    np.testing.assert_allclose(r,value(target,source),atol=1e-14,rtol=0)
    for which,sign in [('source',1),('target',-1)]:
        columns = []
        for axis in range(6):
            d = np.zeros(6);d[axis] = 1e-6
            plus,minus = audit.increment(source if which == 'source' else target,d),audit.increment(source if which == 'source' else target,-d)
            columns.append((value(target,plus)-value(target,minus))/2e-6 if which == 'source'
                           else (value(plus,source)-value(minus,source))/2e-6)
        np.testing.assert_allclose(np.array(columns).T,sign*j,atol=2e-9,rtol=0)
    swapped,_ = residual(points[2],points[3],points[0],points[1])
    np.testing.assert_allclose(swapped,-r,atol=1e-14,rtol=0)
    common = audit.increment(np.eye(4),np.array([2.,-.3,.7,.3,-.15,.1]))
    np.testing.assert_allclose(value(common@target,common@source),r,atol=1e-14,rtol=0)
    average = (points[1]+points[3]);average /= np.linalg.norm(average,axis=1)[:,None]
    frozen_normal_j = np.cross(points[2],average)
    require(np.max(abs(frozen_normal_j-j[:,3:])) > 1e-3,'normal-rotation omission is discriminating')
    camera = audit.rays.CAMERA;truth = audit.increment(np.eye(4),np.array([.04,0,.002,0,.0015,0]))
    a = dict(surface=pyramid(audit.rays.render(np.eye(4)),camera));b = dict(surface=pyramid(audit.rays.render(truth),camera))
    counts = {}
    for scale in (4,2,1):
        actual = pair(a,b,np.eye(4),truth,scale);old = original_pair(a,b,np.eye(4),truth,scale)
        require(actual[0] == old[0] >= 200,'original fixed-pose support parity');counts[scale] = actual[0]
    parent = (ROOT/'scripts/marginalized_keyframe_depth.py').read_text()
    require((ROOT/'scripts/symmetric_keyframe_depth.py').read_text()
            == parent.replace('from joint_keyframe_depth import pair','from symmetric_depth_factors import pair'),
            'only pair implementation differs from original retained engine')
    return dict(source_and_target_central_difference_axes=12,swapped_residual_sign=True,common_rigid_frame_invariance=True,
        omitted_normal_rotation_detected=True,original_fixed_pose_support_counts=counts,parent_engine_only_pair_import_changed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);audit = auditor()
    controls = factor_controls(audit);report = audit.analytic()
    report.update(symmetric_factor=controls,source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in
        [Path(__file__),ROOT/'scripts/symmetric_depth_factors.py',ROOT/'scripts/symmetric_keyframe_depth.py']})
    write(out/'summary.json',report)


if __name__ == '__main__':main()
