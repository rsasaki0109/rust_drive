#!/usr/bin/env python3
"""Independent inverse-depth interpolation/Jacobians and inherited prior audit."""
import argparse
import importlib.util
from pathlib import Path

import numpy as np

import subpixel_keyframe_depth as kernel
from projective_depth_common import ROOT, fresh, require, sha, write


def independent_sample(layer,points):
    # Homogeneous projection and polynomial interpolation, separate from the
    # operational corner-weight and explicit derivative implementation.
    c = layer['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    ids = np.flatnonzero(points[:,2] > .3);q = points[ids]
    image = q@k.T;uv = image[:,:2]/image[:,2,None]
    nearest = np.round(uv);uv = np.where(np.abs(uv-nearest) <= 1e-10,nearest,uv)
    cell = np.floor(uv).astype(int)
    inside = np.all(cell >= 1,axis=1)&(cell[:,0] < c['width']-2)&(cell[:,1] < c['height']-2)
    ids,q,image,uv,cell = [x[inside] for x in (ids,q,image,uv,cell)]
    xx = cell[:,0,None]+[0,1,0,1];yy = cell[:,1,None]+[0,0,1,1]
    depth = layer['depth'][yy,xx];n = layer['normals'][yy,xx]
    ok = ((depth > .3)&(depth < 5)).all(axis=1)
    ok &= (np.ptp(depth,axis=1) <= .05+.01*np.min(depth,axis=1))
    ok &= (np.sum(n*n,axis=2) > .25).all(axis=1)
    ids,q,image,uv,cell,depth = [x[ok] for x in (ids,q,image,uv,cell,depth)]
    coefficients = (1/depth)@np.array([[1,-1,-1,1],[0,1,0,-1],[0,0,1,-1],[0,0,0,1.]])
    xy = uv-cell;x,y = xy.T
    rho = np.sum(coefficients*np.column_stack([np.ones(len(x)),x,y,x*y]),axis=1)
    gradient = coefficients[:,1:3]+coefficients[:,3,None]*xy[:,::-1]
    projective = np.repeat(k[None,:2,:],len(q),axis=0)/q[:,2,None,None]
    projective[:,:,2] -= image[:,:2]/q[:,2,None]**2
    derivative = np.einsum('ni,nij->nj',gradient,projective)
    derivative[:,2] += 1/q[:,2]**2
    target = np.column_stack([uv,np.ones(len(uv))])@np.linalg.inv(k).T/rho[:,None]
    return ids,rho-1/q[:,2],derivative,target


def independent_raw(references,source,pose,scale,enabled):
    require(len(references) == 1 and not enabled,'one geometric pair, no RGB')
    reference = references[0];a,b = reference['surface'][scale],source['surface'][scale]
    stride = {4:1,2:2,1:4}[scale]
    pixels = np.arange(b['depth'].size).reshape(b['depth'].shape)[::stride,::stride].ravel()
    xyz = b['vertices'].reshape(-1,3)[pixels];normals = b['normals'].reshape(-1,3)[pixels]
    good = np.linalg.norm(normals,axis=1) > .5;xyz,normals = xyz[good],normals[good]
    relative = np.linalg.solve(reference['root'],pose)
    local = np.column_stack([xyz,np.ones(len(xyz))])@relative.T
    ids,f,df,target = independent_sample(a,local[:,:3]);size = np.linalg.norm(df,axis=1)
    nr = (df/size[:,None])@reference['root'][:3,:3].T
    sn = normals@pose[:3,:3].T
    ok = np.sqrt(np.sum((local[ids,:3]-target)**2,axis=1)) < (.12 if scale == 4 else .08)
    ok &= np.einsum('ij,ij->i',nr,sn[ids]) > .8
    root = xyz@pose[:3,:3].T+pose[:3,3];ids,nr,r = ids[ok],nr[ok],(f/size)[ok]
    j = np.concatenate([nr,np.cross(root[ids],nr)],axis=1)
    w = np.where(abs(r) <= .01,1.,.01/np.maximum(abs(r),1e-15))
    return dict(geometry=dict(count=len(r),rms_m=float(np.sqrt(np.mean(r*r))) if len(r) else 0.,
                              h=(j*w[:,None]).T@j,g=j.T@(w*r)))


def auditor():
    # Bind a private instance of the unchanged audit to this independent raw
    # oracle. Operational module globals and the archived auditor stay intact.
    spec = importlib.util.spec_from_file_location('subpixel_prior_audit',ROOT/'scripts/check-marginalized-keyframes.py')
    audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    audit.kernel = kernel;audit.raw_equation = independent_raw
    return audit


def interpolation_controls():
    import subpixel_depth_factors as factors
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5)
    y,x = np.indices((32,32));rho = .55+.001*x+.0007*y+.00002*x*x
    layer = dict(camera=c,depth=1/rho,normals=np.tile([0.,0.,1.],(32,32,1)))
    uv = np.array([[10.2,11.4],[14.3,10.5],[8.7,20.8]])
    z = np.array([1.75,1.8,1.7]);points = np.column_stack([(uv[:,0]-c['cx'])*z/c['fx'],(uv[:,1]-c['cy'])*z/c['fy'],z])
    expected = independent_sample(layer,points);actual = factors.sample(layer,points)
    for a,b in zip(actual,expected):np.testing.assert_allclose(a,b,atol=1e-13,rtol=0)
    # A scalar four-corner sum checks interpolation without either vectorized form.
    for i,(u,v) in enumerate(uv):
        xx,yy = int(np.floor(u)),int(np.floor(v));value = 0.
        for dx,dy in ((0,0),(1,0),(0,1),(1,1)):
            value += ((u-xx) if dx else (1-u+xx))*((v-yy) if dy else (1-v+yy))/layer['depth'][yy+dy,xx+dx]
        np.testing.assert_allclose(actual[1][i],value-1/z[i],atol=1e-14,rtol=0)
    normalizer = np.linalg.norm(actual[2],axis=1)
    for axis in range(3):
        step = np.zeros_like(points);step[:,axis] = 1e-6
        numeric = (factors.sample(layer,points+step)[1]-factors.sample(layer,points-step)[1])/2e-6/normalizer
        np.testing.assert_allclose(numeric,actual[2][:,axis]/normalizer,atol=2e-9,rtol=0)
    # Preserve the declared derivative at a pixel knot and value continuity.
    knot = points[:1].copy();knot[:,0] = (10-c['cx'])*knot[:,2]/c['fx']
    center = factors.sample(layer,knot);epsilon = np.array([[1e-7,0.,0.]])
    right,left = factors.sample(layer,knot+epsilon),factors.sample(layer,knot-epsilon)
    np.testing.assert_allclose((right[1]-center[1])/1e-7,center[2][:,0],atol=1e-8,rtol=0)
    require(abs(right[1][0]-left[1][0]) < 1e-7,'continuous value across pixel knot')
    negatives = []
    for name,change in [('missing depth',lambda a:a['depth'].__setitem__((11,10),0.)),
                        ('invalid corner normal',lambda a:a['normals'].__setitem__((12,11),[0.,0.,0.])),
                        ('depth discontinuity',lambda a:a['depth'].__setitem__((11,11),3.))]:
        bad = dict(camera=c,depth=layer['depth'].copy(),normals=layer['normals'].copy());change(bad)
        require(len(factors.sample(bad,points[:1])[0]) == 0,'four-corner rejection: '+name);negatives.append(name)
    behind = points[:1].copy();behind[:,2] = -.1
    require(len(factors.sample(layer,behind)[0]) == 0,'behind-camera rejection')
    # Dropping the image-gradient term must fail the Jacobian comparison.
    wrong = np.zeros_like(actual[2]);wrong[:,2] = 1/z**2
    require(np.max(abs(wrong-actual[2])) > 1e-3,'gradient omission control is discriminating')
    parent = (ROOT/'scripts/marginalized_keyframe_depth.py').read_text()
    require((ROOT/'scripts/subpixel_keyframe_depth.py').read_text()
            == parent.replace('from joint_keyframe_depth import pair','from subpixel_depth_factors import pair'),
            'only pair implementation differs from the retained-information engine')
    return dict(scalar_samples=3,frozen_normalizer_central_difference_axes=3,right_knot_derivative=True,
                continuous_knot_value=True,rejected_corner_controls=negatives,behind_camera_rejected=True,
                omitted_projective_gradient_detected=True,parent_engine_only_pair_import_changed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    controls = interpolation_controls();report = auditor().analytic()
    report.update(interpolation=controls,source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in
        [Path(__file__),ROOT/'scripts/subpixel_depth_factors.py',ROOT/'scripts/subpixel_keyframe_depth.py']})
    write(out/'summary.json',report)


if __name__ == '__main__':main()
