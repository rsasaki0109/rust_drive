#!/usr/bin/env python3
"""Independent patch SVD, normal fixtures and retained-factor reconstruction."""
import argparse
import importlib.util
from pathlib import Path

import numpy as np

import patch_keyframe_depth as kernel
from patch_depth_normals import pyramid
from projective_depth_common import ROOT, fresh, require, sha, write
from projective_depth_raw import pyramid as depth_pyramid


def independent_pyramid(raw, camera):
    # Only frozen depth decimation, intrinsics and vertices are shared. Rebuild
    # every normal with weighted centered patch SVD, never the operational PCA.
    layers = depth_pyramid(raw,camera)
    for layer in layers.values():
        depth,xyz = layer['depth'],layer['vertices'];h,w = depth.shape
        output = np.zeros_like(xyz)
        yy,xx = np.mgrid[2:h-2,2:w-2];yy,xx = yy.ravel(),xx.ravel()
        offsets = np.array([(y,x) for y in range(-2,3) for x in range(-2,3)])
        for begin in range(0,len(yy),8192):
            y,x = yy[begin:begin+8192],xx[begin:begin+8192]
            indices = (y[:,None]+offsets[:,0],x[:,None]+offsets[:,1])
            points,z = xyz[indices],depth[indices];center = depth[y,x]
            present = (z > .3)&(z < 5)&(np.abs(z-center[:,None]) < .05+.01*center[:,None])
            count = present.sum(axis=1)
            valid = (center > .3)&(center < 5)&(count >= 13)
            points,present,count,y,x = points[valid],present[valid],count[valid],y[valid],x[valid]
            mean = (points*present[:,:,None]).sum(axis=1)/count[:,None]
            deviations = np.where(present[:,:,None],points-mean[:,None,:],0.)
            _,singular,vh = np.linalg.svd(deviations,full_matrices=False)
            keep = (singular[:,1]**2/count > 1e-8)&(singular[:,2]**2 <= .1*singular[:,1]**2)
            normal = vh[:,-1,:]
            flipped = np.sum(normal*xyz[y,x],axis=1) < 0
            normal[flipped] *= -1
            output[y[keep],x[keep]] = normal[keep]
        layer['normals'] = output
    return layers


def auditor():
    spec = importlib.util.spec_from_file_location('patch_symmetric_audit',ROOT/'scripts/check-symmetric-keyframes.py')
    symmetric = importlib.util.module_from_spec(spec);spec.loader.exec_module(symmetric)
    audit = symmetric.auditor()
    audit.kernel = kernel;audit.pyramid = independent_pyramid
    return audit


def normal_controls():
    camera = dict(width=160,height=120,fx=130.,fy=130.,cx=79.5,cy=59.5)
    yy,xx = np.indices((120,160))
    directions = np.stack([(xx-79.5)/130,(yy-59.5)/130,np.ones((120,160))],axis=-1)
    truth = np.array([-.15,.12,1.]);truth /= np.linalg.norm(truth)
    exact = 2./np.einsum('...i,i->...',directions,truth)
    rng = np.random.default_rng(421016)
    noisy = exact+rng.normal(0,.002,exact.shape)
    step = np.where(xx < 80,2.,2.4)
    hole = exact.copy();hole[40:60,70:90] = 0
    thin = np.zeros_like(exact);thin[60,:] = exact[60,:]
    textured = 2.+.06*((xx+yy)%2)
    fixtures = dict(plane=exact,noisy_plane=noisy,depth_step=step,hole=hole,line=thin,rough=textured)
    checked = 0;max_difference = 0.
    for name,depth in fixtures.items():
        actual,expected,old = pyramid(depth,camera),independent_pyramid(depth,camera),depth_pyramid(depth,camera)
        for scale in (1,2,4):
            a,b = actual[scale],expected[scale]
            for key in ('depth','vertices'):np.testing.assert_array_equal(a[key],old[scale][key])
            require(a['camera'] == old[scale]['camera'],'unchanged pyramid intrinsics')
            np.testing.assert_array_equal(np.linalg.norm(a['normals'],axis=2) > .5,np.linalg.norm(b['normals'],axis=2) > .5)
            np.testing.assert_allclose(a['normals'],b['normals'],atol=2e-12,rtol=0)
            max_difference = max(max_difference,float(np.max(abs(a['normals']-b['normals']))));checked += 1
        n = actual[1]['normals'];valid = np.linalg.norm(n,axis=2) > .5
        if name == 'plane':
            require(valid.sum() == 156*116,'full interior exact-plane support')
            np.testing.assert_allclose(n[valid],np.broadcast_to(truth,n[valid].shape),atol=2e-12,rtol=0)
        if name == 'depth_step':
            require(valid[2:-2,78:82].all(),'same-side majority supports the two depths at the step')
            np.testing.assert_allclose(n[valid],np.broadcast_to([0,0,1.],n[valid].shape),atol=2e-12,rtol=0)
        if name == 'hole':require(not valid[40:60,70:90].any(),'no normal at missing center')
        if name == 'line':require(not valid.any(),'collinear or insufficient points rejected')
        if name == 'rough':require(not valid.any(),'nonplanar patch rejected')
    old,new = depth_pyramid(noisy,camera)[1]['normals'],pyramid(noisy,camera)[1]['normals']
    common = (np.linalg.norm(old,axis=2) > .5)&(np.linalg.norm(new,axis=2) > .5)
    angles = [np.arccos(np.clip(n[common]@truth,-1,1)) for n in (old,new)]
    before,after = [np.percentile(a,[50,95]).tolist() for a in angles]
    require(after[0] < .5*before[0] and after[1] < .5*before[1],'known 2mm Gaussian Z noise: median and p95 normal error reduced')
    # Centered partial support distinguishes the fixed majority rule from
    # accepting arbitrary sparse patches. These are separate from a line.
    small = np.zeros_like(exact);small[58:63,78:83] = exact[58:63,78:83]
    positions = [(y,x) for y in range(58,63) for x in range(78,83) if (y,x) != (60,80)]
    for y,x in positions[:12]:small[y,x] = 0
    require(np.linalg.norm(pyramid(small,camera)[1]['normals'][60,80]) > .5,'13 planar samples admitted')
    y,x = positions[12];small[y,x] = 0
    require(np.linalg.norm(pyramid(small,camera)[1]['normals'][60,80]) == 0,'12 samples rejected')
    # Reject zero middle eigenvalue even with 25 depth-gated points.
    from patch_depth_normals import normals
    xyz = np.zeros((9,9,3));xyz[:,:,0] = np.arange(9);xyz[:,:,2] = 2
    require(not normals(np.full((9,9),2.),xyz).any(),'covariance line rank rejected')
    parent = (ROOT/'scripts/symmetric_keyframe_depth.py').read_text()
    require((ROOT/'scripts/patch_keyframe_depth.py').read_text() == parent.replace(
        'from projective_depth_raw import pyramid','from patch_depth_normals import pyramid'),'only normal frontend import differs from symmetric control')
    return dict(independent_normal_method='centered patch SVD with zero rows for excluded samples',
        normal_layers_compared=checked,max_component_difference=max_difference,depth_vertices_intrinsics_unchanged=True,
        noisy_plane_common_pixels=int(common.sum()),noise_seed=421016,axial_gaussian_sigma_m=.002,
        normal_error_rad_median_p95_before=before,normal_error_rad_median_p95_after=after,
        exact_plane_and_depth_step=True,missing_center_and_sparse_line_rejected=True,
        minimum_13_samples_enforced=True,rank_and_roughness_rejected=True,parent_engine_only_pyramid_import_changed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);audit = auditor()
    controls = normal_controls()
    spec = importlib.util.spec_from_file_location('shared_symmetric_controls',ROOT/'scripts/check-symmetric-keyframes.py')
    shared = importlib.util.module_from_spec(spec);spec.loader.exec_module(shared)
    factor = shared.factor_controls(audit);report = audit.analytic()
    report.update(patch_normals=controls,symmetric_factor=factor,source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in
        [Path(__file__),ROOT/'scripts/patch_depth_normals.py',ROOT/'scripts/patch_keyframe_depth.py']})
    write(out/'summary.json',report)


if __name__ == '__main__':main()
