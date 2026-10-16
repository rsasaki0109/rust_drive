#!/usr/bin/env python3
"""Independent cubic support stencil and unchanged private objective verification."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import cubic_margin_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


private = load('cubic_margin_private_oracle','check-cubic-depth-refinement.py')
center_population = private.population


def sampling_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];relative = np.linalg.solve(poses[a],poses[b])
    q = np.column_stack([item['points'],np.ones(len(item['points']))])@relative.T
    c = nodes[a]['surface'][scale]['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    image = q[:,:3]@k.T;uv = image[:,:2]/image[:,2,None];kept = np.arange(len(q))
    for offset in ((-1,-1),(0,-1),(1,-1),(-1,0),(0,0),(1,0),(-1,1),(0,1),(1,1)):
        rays = np.column_stack([uv+offset,np.ones(len(q))])@np.linalg.inv(k).T
        ids,_,_,_ = private.independent_sample(nodes[a]['surface'][scale],rays*q[:,2,None])
        kept = np.intersect1d(kept,ids,assume_unique=True)
    mask = np.zeros(len(q),dtype=bool);mask[kept] = True;return mask


def population(nodes,poses,scale):
    result = []
    for item in center_population(nodes,poses,scale):
        indices = np.flatnonzero(sampling_mask(item,nodes,poses,scale))
        if len(indices)>=200:result.append(dict(item,**{key:item[key][indices] for key in ('points','normals','source_ids','normalizer')}))
    return result


private.population = private.fixed.population = population
verify,verify_checkpoint = private.verify,private.verify_checkpoint


def source_proof():
    require(kernel.private.population is kernel.population and kernel.refine.__globals__['population'] is kernel.population,'private solver initial population binding')
    require(verify.__globals__['population'] is population and verify_checkpoint.__globals__['population'] is population,'private verifier initial population binding')
    for name in ('values','evaluate','equation','energy','search','refine','snapshot','checkpoint'):
        require(getattr(kernel,name) is getattr(kernel.private,name),'unchanged private cubic function '+name)
    require(kernel.sample is kernel.previous.sample,'unchanged C1 sampler')
    return True


def controls():
    proof = source_proof();inherited = load('cubic_margin_original_controls','check-cubic-depth-refinement.py').controls()
    audit = load('cubic_margin_rendered','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA
    depth = audit.rays.render(identity);truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = [dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(nodes,(depth,source),strict=True)]
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    endpoints = sum(verify_checkpoint(kernel.checkpoint(report,nodes,None,cap),report,other,initial,None,audit,cap) for cap in (10,20,30))
    old = kernel.previous.population(nodes,initial,1);new = kernel.population(nodes,initial,1)
    require(sum(len(i['source_ids']) for i in new)<sum(len(i['source_ids']) for i in old),'measured initial margin changes support')
    rejected=[]
    for name,change in (('center-only population',lambda r:r.update(frozen_population=kernel.metadata(old))),
        ('false convergence',lambda r:r.update(continuous_converged=not r['continuous_converged'])),
        ('wrong final matrix',lambda r:r['final_equation'].update(g=(np.asarray(r['final_equation']['g'])+1).tolist()))):
        bad=copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected cubic margin corruption: '+name)
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    return dict(analytic_pass=True,source_proof=proof,inherited_cubic_controls=inherited,reported_systems=checked,checkpoint_systems=endpoints,
        rendered_continuous_converged=report['continuous_converged'],rendered_applied_corrections=report['applied_iterations'],rendered_guard=report['guard_failure'],
        original_rendered_sources=sum(len(i['source_ids']) for i in old),margin_rendered_sources=sum(len(i['source_ids']) for i in new),
        rejected_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=fresh(args.output);report=controls()
    report['source_sha256']={'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('cubic_margin_refinement.py','check-cubic-margin-refinement.py')}
    write(out/'summary.json',report)


if __name__=='__main__':main()
