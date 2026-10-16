#!/usr/bin/env python3
"""Separate relative-camera margin stencil with the unchanged fixed-objective audit."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import margin_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


# Private verifier instance: only its initial selection changes. Operational
# modules and the archived verifier's source/globals remain untouched.
independent = load('margin_private_continuous_audit','check-continuous-depth-refinement.py')
original_population = independent.population


def margin_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];relative = np.linalg.solve(poses[a],poses[b])
    local = item['points']@relative[:3,:3].T+relative[:3,3]
    normals = item['normals']@relative[:3,:3].T
    c = nodes[a]['surface'][scale]['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    image = local@k.T;uv = image[:,:2]/image[:,2,None]
    intersection = np.arange(len(local))
    for offset in ((-1,-1),(0,-1),(1,-1),(-1,0),(0,0),(1,0),(-1,1),(0,1),(1,1)):
        rays = np.column_stack([uv+offset,np.ones(len(uv))])@np.linalg.inv(k).T
        query = rays*local[:,2,None]
        ids,_,df,point = independent.interpolation.independent_sample(nodes[a]['surface'][scale],query)
        length = np.sqrt(np.einsum('ij,ij->i',df,df))
        require(np.isfinite(length).all() and np.all(length > 1e-12),'finite margin gradient')
        ok = np.sqrt(np.sum((query[ids]-point)**2,axis=1)) < (.12 if scale == 4 else .08)
        ok &= np.einsum('ij,ij->i',normals[ids],df/length[:,None]) > .8
        intersection = np.intersect1d(intersection,ids[ok],assume_unique=True)
    result = np.zeros(len(local),dtype=bool);result[intersection] = True;return result


def population(nodes,poses,scale):
    rows = []
    for item in original_population(nodes,poses,scale):
        keep = margin_mask(item,nodes,poses,scale)
        if int(keep.sum()) >= 200:
            selected = dict(item)
            for name in ('points','normals','source_ids','normalizer'):selected[name] = item[name][keep]
            rows.append(selected)
    return rows


independent.population = population
verify = independent.verify


def source_proof():
    def body(name):
        source = (ROOT/'scripts'/name).read_text()
        function = next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name == 'refine')
        return ast.get_source_segment(source,function)
    require(body('margin_depth_refinement.py') == body('continuous_depth_refinement.py'),'local solver body unchanged')
    for name in ('equation','energy','evaluate','metadata','search'):
        require(getattr(kernel,name) is getattr(kernel.parent,name),'unchanged continuous '+name)
    return True


def controls():
    proof = source_proof();independent.kernel = kernel
    inherited = independent.controls()
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5)
    z = np.full((32,32),2.);normals = np.tile([0.,0.,1.],(32,32,1))
    z[12,19] = 0.
    y,x = np.indices(z.shape);vertices = np.stack([(x-c['cx'])*z/c['fx'],(y-c['cy'])*z/c['fy'],z],axis=-1)
    layer = dict(depth=z,normals=normals,vertices=vertices,camera=c)
    uv = np.array([[10.25,12.25],[1.25,12.25],[20.25,12.25]])
    points = np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(3,2.)])
    item = dict(target=0,source=1,points=points,normals=np.tile([0.,0.,1.],(3,1)),source_ids=np.arange(3),normalizer=np.ones(3))
    nodes = [dict(timestamp=t,root=np.eye(4),surface={1:layer}) for t in (1.,1.03)];poses = np.array([np.eye(4),np.eye(4)])
    actual = kernel.margin_mask(item,nodes,poses,1);expected = margin_mask(item,nodes,poses,1)
    np.testing.assert_array_equal(actual,[True,False,False]);np.testing.assert_array_equal(actual,expected)
    require(len(kernel.parent.values(nodes[0],nodes[1],poses,0,1,points,item['normals'],1)[0]) == 3,'center-only validity misses border and adjacent hole')
    audit = load('margin_rendered_control','check-reciprocal-keyframes.py').auditor();camera = audit.rays.CAMERA
    depth = audit.rays.render(np.eye(4));truth = audit.increment(np.eye(4),np.array([.04,0,.002,0,.0015,0]))
    source = audit.rays.render(truth);tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered warm initialization');initial = np.asarray(warm['revised_roots'])
    raw = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = [dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(raw,(depth,source),strict=True)]
    report = kernel.refine(raw,initial,None);checked = verify(report,other,initial,None,audit)
    old = kernel.parent.population(raw,initial,1);new = kernel.population(raw,initial,1)
    require(sum(len(i['source_ids']) for i in new) < sum(len(i['source_ids']) for i in old),'real rendered stencil changes support')
    for a,b in zip(new,population(other,initial,1),strict=True):
        np.testing.assert_array_equal(a['source_ids'],b['source_ids'])
        require(np.isin(a['source_ids'],next(x['source_ids'] for x in old if x['target']==a['target'] and x['source']==a['source'])).all(),'stencil only removes initial sources')
    rejected = []
    bad = copy.deepcopy(report);bad['frozen_population'] = kernel.parent.metadata(old)
    try:verify(bad,other,initial,None,audit)
    except (ValueError,AssertionError):rejected.append('center-only population substituted for margin selection')
    else:raise ValueError('missing initial margin policy not detected')
    return dict(analytic_pass=True,source_proof=proof,inherited_continuous_controls=inherited,
        additional_rendered_reported_systems=checked,hand_counted_probe_mask=actual.tolist(),
        initial_center_qualified_fixture_samples=3,initial_margin_qualified_fixture_samples=1,
        rendered_parent_source_occurrences=sum(len(i['source_ids']) for i in old),
        rendered_margin_source_occurrences=sum(len(i['source_ids']) for i in new),
        rejected_margin_corruptions=rejected,stencil_is_not_a_continuous_domain_guarantee=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('margin_depth_refinement.py','check-margin-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
