#!/usr/bin/env python3
"""Independent initial geometry, interpolation-only stencil and fixed-domain audit."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import sampling_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


# Separate private instances keep the old initial geometric qualifier intact
# while changing only the new verifier's candidate-domain and initial stencil.
selector = load('sampling_initial_geometry','check-continuous-depth-refinement.py')
independent = load('sampling_fixed_objective_audit','check-continuous-depth-refinement.py')


def values(nodes,poses,a,b,points,normals,scale):
    relative = np.linalg.solve(poses[a],poses[b])
    q = np.column_stack([points,np.ones(len(points))])@relative.T
    ids,f,df,_ = independent.interpolation.independent_sample(nodes[a]['surface'][scale],q[:,:3])
    size = np.sqrt(np.einsum('ij,ij->i',df,df))
    require(np.isfinite(size).all() and np.all(size > 1e-12),'finite continuous gradient')
    return ids,f,df,size


def sampling_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];relative = np.linalg.solve(poses[a],poses[b])
    local = item['points']@relative[:3,:3].T+relative[:3,3]
    c = nodes[a]['surface'][scale]['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    image = local@k.T;uv = image[:,:2]/image[:,2,None];kept = np.arange(len(local))
    for offset in ((-1,-1),(0,-1),(1,-1),(-1,0),(0,0),(1,0),(-1,1),(0,1),(1,1)):
        rays = np.column_stack([uv+offset,np.ones(len(uv))])@np.linalg.inv(k).T
        ids,_,df,_ = independent.interpolation.independent_sample(nodes[a]['surface'][scale],rays*local[:,2,None])
        size = np.sqrt(np.einsum('ij,ij->i',df,df))
        require(np.isfinite(size).all() and np.all(size > 1e-12),'finite sampling gradient')
        kept = np.intersect1d(kept,ids,assume_unique=True)
    result = np.zeros(len(local),dtype=bool);result[kept] = True;return result


def population(nodes,poses,scale):
    rows = []
    for item in selector.population(nodes,poses,scale):
        mask = sampling_mask(item,nodes,poses,scale)
        if int(mask.sum()) >= 200:
            result = dict(item)
            for key in ('points','normals','source_ids','normalizer'):result[key] = item[key][mask]
            rows.append(result)
    return rows


independent.values = values
independent.population = population
verify = independent.verify


def source_proof():
    def bodies(name):
        text = (ROOT/'scripts'/name).read_text()
        return {n.name:ast.get_source_segment(text,n) for n in ast.parse(text).body if isinstance(n,ast.FunctionDef)}
    old,new = bodies('continuous_depth_refinement.py'),bodies('sampling_depth_refinement.py')
    names = ('evaluate','equation','energy','search','refine')
    for name in names:require(new[name] == old[name],'unchanged continuous function body '+name)
    require(kernel.metadata is kernel.parent.metadata and kernel.discrete is kernel.parent.discrete,'shared metadata/solve/prior dependencies')
    return list(names)


def controls():
    proof = source_proof();independent.kernel = kernel;inherited = independent.controls()
    # A qualified interpolation can remain defined after the initial source
    # normal/gradient gate ceases to hold. This is a domain-policy fixture,
    # not evidence that such a sample is physically correct.
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5)
    z = np.full((32,32),2.);normals = np.tile([0.,0.,1.],(32,32,1))
    y,x = np.indices(z.shape);vertices = np.stack([(x-c['cx'])*z/c['fx'],(y-c['cy'])*z/c['fy'],z],axis=-1)
    layer = dict(depth=z,normals=normals,vertices=vertices,camera=c)
    uv = np.array([[10.25,12.25],[1.25,12.25],[20.25,12.25]])
    points = np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(3,2.)])
    item = dict(target=0,source=1,points=points,normals=np.tile([0.,0.,1.],(3,1)),source_ids=np.arange(3),normalizer=np.ones(3))
    nodes = [dict(timestamp=t,root=np.eye(4),surface={1:layer}) for t in (1.,1.03)];poses = np.array([np.eye(4),np.eye(4)])
    altered = item['normals'].copy();altered[:,2] = -1
    require(len(kernel.parent.values(nodes[0],nodes[1],poses,0,1,points,altered,1)[0]) == 0,'old geometric agreement gate discriminates')
    np.testing.assert_array_equal(kernel.values(nodes[0],nodes[1],poses,0,1,points,altered,1)[0],[0,1,2])
    np.testing.assert_array_equal(values(nodes,poses,0,1,points,altered,1)[0],[0,1,2])
    shifted = poses.copy();shifted[1,2,3] = .09
    require(len(kernel.parent.values(nodes[0],nodes[1],shifted,0,1,points,item['normals'],1)[0]) == 0,'old distance gate discriminates')
    np.testing.assert_array_equal(kernel.values(nodes[0],nodes[1],shifted,0,1,points,item['normals'],1)[0],[0,1,2])
    np.testing.assert_array_equal(values(nodes,shifted,0,1,points,item['normals'],1)[0],[0,1,2])
    z[12,19] = 0.
    actual = kernel.sampling_mask(item,nodes,poses,1);expected = sampling_mask(item,nodes,poses,1)
    np.testing.assert_array_equal(actual,[True,False,False]);np.testing.assert_array_equal(actual,expected)
    # The initial geometric selector is an unmodified, separate instance.
    require(selector.values is not values,'candidate-domain change cannot alter initial qualifier')
    audit = load('sampling_rendered_subset','check-reciprocal-keyframes.py').auditor();camera = audit.rays.CAMERA
    depth = audit.rays.render(np.eye(4));truth = audit.increment(np.eye(4),np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    raw = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = [dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(raw,(depth,source),strict=True)]
    old = kernel.parent.population(raw,initial,1);new = kernel.population(raw,initial,1)
    report = kernel.refine(raw,initial,None);checked = verify(report,other,initial,None,audit)
    require(sum(len(i['source_ids']) for i in new) < sum(len(i['source_ids']) for i in old),'sampling stencil changes real initial support')
    rejected = []
    bad = copy.deepcopy(report);bad['frozen_population'] = kernel.parent.metadata(old)
    try:verify(bad,other,initial,None,audit)
    except (ValueError,AssertionError):rejected.append('center-only population substituted for sampling stencil')
    else:raise ValueError('missing sampling stencil accepted')
    return dict(analytic_pass=True,unchanged_function_bodies=proof,inherited_continuous_controls=inherited,
        additional_rendered_reported_systems=checked,hand_counted_sampling_mask=actual.tolist(),
        geometric_gate_and_sampling_domain_distinguished=True,initial_geometry_selector_isolated=True,
        rendered_center_source_occurrences=sum(len(i['source_ids']) for i in old),
        rendered_sampling_source_occurrences=sum(len(i['source_ids']) for i in new),
        rejected_sampling_corruptions=rejected,stencil_is_not_a_continuous_domain_guarantee=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('sampling_depth_refinement.py','check-sampling-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
