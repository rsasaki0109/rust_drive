#!/usr/bin/env python3
"""Reciprocal image association, independent raw systems and retained state audit."""
import argparse
import importlib.util
from pathlib import Path

import numpy as np

import reciprocal_keyframe_depth as kernel
from reciprocal_depth_oracle import independent_raw
from projective_depth_common import ROOT, fresh, require, sha, write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def auditor():
    base = load('reciprocal_selective_audit','check-selective-keyframes.py')
    audit = base.auditor();audit.kernel = kernel;audit.raw_equation = independent_raw
    return audit


def association_controls(audit):
    from reciprocal_depth_factors import pair
    from symmetric_depth_factors import pair as old_pair
    patch = load('reciprocal_patch_oracle','check-patch-keyframes.py')
    camera = audit.rays.CAMERA
    first = np.eye(4);second = audit.increment(first,np.array([.04,0,.002,0,.0015,0]))
    target = dict(root=first,surface=patch.independent_pyramid(audit.rays.render(first),camera))
    source = dict(root=second,surface=patch.independent_pyramid(audit.rays.render(second),camera))
    counts = {}
    for scale in (4,2,1):
        n,ss,h,g = pair(target,source,first,second,scale)
        expected = independent_raw([target],source,second,scale,False)
        e = expected['geometry'];selection = expected['selection']
        require(n == e['count'] == selection['reciprocal_count'] >= 200,'independent reciprocal sample support')
        np.testing.assert_allclose(h,e['h'],atol=1e-7,rtol=1e-10)
        np.testing.assert_allclose(g,e['g'],atol=1e-9,rtol=1e-10)
        np.testing.assert_allclose(np.sqrt(ss/n),e['rms_m'],atol=1e-12,rtol=0)
        old = old_pair(target,source,first,second,scale)
        require(old[0] == selection['forward_count'] and n <= old[0],'fixed-pose filter only removes forward candidates')
        counts[scale] = dict(forward=old[0],reciprocal=n)
        if scale == 4:
            reverse = pair(source,target,second,first,scale)
            require(n == reverse[0],'unstrided reciprocal endpoint count symmetry')
            np.testing.assert_allclose(ss,reverse[1],atol=1e-12,rtol=0)
            np.testing.assert_allclose(h,reverse[2],atol=1e-7,rtol=1e-10)
            np.testing.assert_allclose(g,-reverse[3],atol=1e-9,rtol=1e-10)
    require(any(x['reciprocal'] < x['forward'] for x in counts.values()),'nontrivial reciprocity filtering exercised')
    # One fake return path is isolated from pose fitting. Both forward distance
    # and normal gates pass; a deliberately inconsistent target point projects
    # outside the source image. Reciprocal matching must reject all candidates.
    c = dict(width=32,height=32,fx=100.,fy=100.,cx=15.5,cy=15.5)
    point = np.zeros((32,32,3));point[:,:,2] = .31
    normal = np.zeros_like(point);normal[:,:,2] = 1
    # Forward queries all hit the central target pixel. The target vertex is
    # 7cm to the right, still under the finest 8cm 3D-distance gate.
    target_points = point.copy();target_points[:,:,0] = .07
    a = dict(surface={1:dict(depth=point[:,:,2],vertices=target_points,normals=normal,camera=c)},root=first)
    b = dict(surface={1:dict(depth=point[:,:,2],vertices=point,normals=normal,camera=c)},root=first)
    require(old_pair(a,b,first,first,1)[0] > 0 and pair(a,b,first,first,1)[0] == 0,'out-of-image return and many-to-one collisions rejected')
    require(independent_raw([a],b,first,1,False)['geometry']['count'] == 0,'independent return rejection')
    a['surface'][1]['vertices'] = point
    require(old_pair(a,b,first,first,1)[0] == 64 and pair(a,b,first,first,1)[0] == 1,'one reciprocal source survives a 64-to-one collision')
    require(independent_raw([a],b,first,1,False)['geometry']['count'] == 1,'independent collision selection')
    shifted = first.copy();shifted[2,3] = .02
    require(old_pair(a,b,first,shifted,1)[0] == 64 and pair(a,b,first,shifted,1)[0] == 0,'return depth below 0.3m rejected before projection')
    require(independent_raw([a],b,shifted,1,False)['geometry']['count'] == 0,'independent return depth rejection')
    # Exact identity on a real rendered image preserves the admitted population
    # and the Huber system; reciprocal checking does not add measurements.
    for scale in (4,2,1):
        actual = pair(target,target,first,first,scale);old = old_pair(target,target,first,first,scale)
        require(actual[0] == old[0],'identity support unchanged')
        np.testing.assert_allclose(actual[2],old[2],atol=1e-12,rtol=0)
        np.testing.assert_allclose(actual[3],old[3],atol=1e-12,rtol=0)
    parent = (ROOT/'scripts/selective_keyframe_depth.py').read_text()
    require((ROOT/'scripts/reciprocal_keyframe_depth.py').read_text() == parent.replace(
        'from symmetric_depth_factors import pair','from reciprocal_depth_factors import pair'),'only pair import changes')
    return dict(raw_oracle_layers_checked=3,fixed_pose_counts=counts,unstrided_endpoint_swap_invariance=True,
        out_of_image_return_and_collisions_rejected=True,return_depth_guard=True,identity_support_and_system_parity=True,parent_only_pair_import_changed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);audit = auditor()
    controls = association_controls(audit)
    shared = load('reciprocal_shared_residual','check-symmetric-keyframes.py');factor = shared.factor_controls(audit)
    report = audit.analytic()
    report.update(reciprocal_association=controls,symmetric_factor=factor,source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in
        [Path(__file__),ROOT/'scripts/reciprocal_depth_factors.py',ROOT/'scripts/reciprocal_depth_oracle.py',ROOT/'scripts/reciprocal_keyframe_depth.py']})
    write(out/'summary.json',report)


if __name__ == '__main__':main()
