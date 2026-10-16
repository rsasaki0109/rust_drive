#!/usr/bin/env python3
"""Independent center geometry and depth-field admission for a private solver."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import slope_admission_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


previous = load('slope_admission_old_controls','check-extended-slope-refinement.py')
private = load('slope_admission_private_verifier','check-extended-slope-refinement.py')
center = load('slope_admission_original_center','check-continuous-depth-refinement.py')
legacy = load('slope_admission_legacy_probes','check-sampling-depth-refinement.py')
prepare,compare_prepared = private.prepare,private.compare_prepared
independent_sample = private.previous.independent_sample


def sampling_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];relative = np.linalg.solve(poses[a],poses[b])
    q = np.column_stack([item['points'],np.ones(len(item['points']))])@relative.T
    c = nodes[a]['surface'][scale]['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    image = q[:,:3]@k.T;uv = image[:,:2]/image[:,2,None];kept = np.arange(len(q))
    for offset in ((-1,-1),(0,-1),(1,-1),(-1,0),(0,0),(1,0),(-1,1),(0,1),(1,1)):
        rays = np.column_stack([uv+offset,np.ones(len(q))])@np.linalg.inv(k).T
        ids,_,df,_ = independent_sample(nodes[a]['surface'][scale],rays*q[:,2,None])
        size = np.sqrt(np.einsum('ij,ij->i',df,df));require(np.isfinite(size).all() and np.all(size>1e-12),'finite sampling gradient')
        kept = np.intersect1d(kept,ids,assume_unique=True)
    mask = np.zeros(len(q),dtype=bool);mask[kept] = True;return mask


def population(nodes,poses,scale):
    rows = []
    for item in center.population(nodes,poses,scale):
        mask = sampling_mask(item,nodes,poses,scale)
        if int(mask.sum())>=200:rows.append(dict(item,**{key:item[key][mask] for key in ('points','normals','source_ids','normalizer')}))
    return rows


def admission_counts(nodes,poses,scale):
    result = []
    for b in range(1,len(nodes)):
        for a in range(b):
            item = center.independent_matches([dict(nodes[a],root=poses[a])],nodes[b],poses[b],scale,False)
            if item['count']<200:continue
            points,normals = item['points'][2:];ids,_,_,size = center.values(nodes,poses,a,b,points,normals,scale)
            row = dict(target=a,source=b,points=points[ids],normals=normals[ids],source_ids=item['pixel_ids'][ids,1],normalizer=size)
            old,new = legacy.sampling_mask(row,nodes,poses,scale),sampling_mask(row,nodes,poses,scale)
            result.append(dict(target=a,source=b,current_source=b==len(nodes)-1,raw_count=item['count'],
                bilinear_geometric_center=len(ids),old_bilinear_probe_count=int(old.sum()),new_model_probe_count=int(new.sum())))
    return result


private.population = population
verify,verify_checkpoint = private.verify,private.verify_checkpoint
check_metadata,system,objective,reference = private.check_metadata,private.system,private.objective,private.reference


def source_proof():
    def body(file,name):
        text = (ROOT/'scripts'/file).read_text();node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    require(body('slope_admission_refinement.py','sampling_mask')==body('sampling_depth_refinement.py','sampling_mask'),'same nine-probe body with a different sampler binding')
    require(body('slope_admission_refinement.py','population')==body('sampling_depth_refinement.py','population').replace('parent.population','center.population'),
        'same source slicing and minimum200 rule')
    for name in ('values','evaluate','equation','energy','search','refine','snapshot','checkpoint'):
        require(getattr(kernel,name) is getattr(kernel.private,name),'unchanged private extended-slope function '+name)
    require(kernel.refine.__globals__['population'] is kernel.population and kernel.snapshot.__globals__['population'] is kernel.population,'private fixed population binding')
    require(verify.__globals__['population'] is population and verify_checkpoint.__globals__['population'] is population,'private independent population binding')
    require(kernel.sample is kernel.previous.sample and kernel.prepare is kernel.previous.prepare,'unchanged field and sampler')
    require(kernel.previous.refine.__globals__['population'] is kernel.previous.population,'original solver remains isolated')
    require(previous.verify.__globals__['population'] is previous.population,'original independent control remains isolated')
    return True


def admission_controls():
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);yy,xx = np.indices((32,32));depth = np.full((32,32),2.)
    xyz = np.stack([(xx-c['cx'])*depth/c['fx'],(yy-c['cy'])*depth/c['fy'],depth],axis=2)
    normal = np.tile([0.,0.,1.],(32,32,1));normal[10,9] = 0.
    layer = dict(depth=depth,normals=normal,vertices=xyz,camera=c);poses = np.tile(np.eye(4),(2,1,1))
    point = np.array([[(10.25-c['cx'])*2/c['fx'],(11.25-c['cy'])*2/c['fy'],2.]])
    item = dict(target=0,source=1,points=point,normals=np.array([[0.,0.,1.]]),source_ids=np.array([77]),normalizer=np.array([.25]))
    def nodes(raw):return [dict(timestamp=1.+i*.03,root=np.eye(4),surface={1:raw}) for i in range(2)]
    actual,oracle = kernel.prepare(nodes(layer)),prepare(nodes(layer))
    before = kernel.legacy.sampling_mask(item,actual,poses,1);after = kernel.sampling_mask(item,actual,poses,1)
    np.testing.assert_array_equal(before,legacy.sampling_mask(item,oracle,poses,1));np.testing.assert_array_equal(after,sampling_mask(item,oracle,poses,1))
    require(not before[0] and after[0],'valid fitted depth probes survive absent unused normal')
    ids,_,_,_,_ = kernel.center.values(actual[0],actual[1],poses,0,1,point,item['normals'],1)
    require(len(ids)==1,'original center geometry still qualifies the authored point')
    bad = dict(layer,depth=layer['depth'].copy());bad['depth'][10,9] = 0.
    require(not kernel.sampling_mask(item,kernel.prepare(nodes(bad)),poses,1)[0] and not sampling_mask(item,prepare(nodes(bad)),poses,1)[0],
        'new probes still reject missing measured corner depth')
    bad = dict(layer,normals=layer['normals'].copy());bad['normals'][11,10] = 0.
    changed = kernel.prepare(nodes(bad));ids,_,_,_,_ = kernel.center.values(changed[0],changed[1],poses,0,1,point,item['normals'],1)
    require(len(ids)==0,'original center normal eligibility is preserved')
    ids,_,_,_,_ = kernel.center.values(actual[0],actual[1],poses,0,1,point,np.array([[1.,0.,0.]]),1)
    require(len(ids)==0,'original center source-normal agreement gate is preserved')
    return dict(unused_normal_model_domain_fixture_pass=True,missing_depth_still_rejected=True,
        center_normal_and_source_agreement_preserved=True,minimum200_rule_source_proof=True)


def controls():
    proof = source_proof();interpolation = previous.previous.support_controls();admission = admission_controls()
    audit = load('cubic_rendered_oracle','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA
    depth = audit.rays.render(identity);truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = prepare([dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(nodes,(depth,source),strict=True)])
    nodes = kernel.prepare(nodes);compare_prepared(nodes,other)
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    endpoints = sum(verify_checkpoint(kernel.checkpoint(report,nodes,None,cap),report,other,initial,None,audit,cap) for cap in (30,60,90))
    # Nonzero incoming prior with both measured endpoints moving: all12 axes.
    third = dict(timestamp=1.06,root=initial[1].copy(),surface=nodes[1]['surface']);raw3 = nodes+[third]
    ind3 = other+[dict(third,surface=other[1]['surface'])];poses3 = np.array([identity,initial[1],initial[1]])
    poses3[1] = audit.increment(poses3[1],np.array([.00073,-.00031,.00021,.00012,-.00014,.00009]))
    poses3[2] = audit.increment(poses3[2],np.array([-.00041,.00023,-.00011,-.00011,.00008,.00017]))
    prior = kernel.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    matches = kernel.population(raw3,poses3,1);expected = population(ind3,poses3,1);e = kernel.equation(matches,raw3,poses3,prior,1)
    audit.compare_equation(kernel.base.encode_equation(e),system(expected,ind3,poses3,prior.encode(),1,audit))
    numeric = []
    for node in (1,2):
        for axis in range(6):
            d = np.zeros(6);d[axis] = 1e-7;plus,minus = poses3.copy(),poses3.copy()
            plus[node] = audit.increment(poses3[node],d);minus[node] = audit.increment(poses3[node],-d)
            numeric.append((objective(expected,ind3,plus,prior.encode(),1,audit)-objective(expected,ind3,minus,prior.encode(),1,audit))/2e-7)
    np.testing.assert_allclose(numeric,e['g'],atol=3e-7,rtol=2e-6)
    rejected = []
    for name,change in (('false convergence',lambda r:r.update(continuous_converged=not r['continuous_converged'])),
        ('changed source identities',lambda r:r['frozen_population'][0].update(source_pixel_sha256='0'*64)),
        ('changed normalizer digest',lambda r:r['frozen_population'][0].update(normalizer_sha256='0'*64)),
        ('wrong final matrix',lambda r:r['final_equation'].update(g=(np.asarray(r['final_equation']['g'])+1).tolist()))):
        bad = copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected cubic corruption: '+name)
    guards = {}
    for name,z in (('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))):
        raw = [dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)]
        raw = kernel.prepare(raw)
        independent = prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw])
        q = kernel.refine(raw,np.array([identity,identity]),None);checked += verify(q,independent,np.array([identity,identity]),None,audit)
        require(q['termination']=='guard' and not q['trace'],'degenerate cubic images cannot fabricate information');guards[name] = q['guard_failure']
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    old = sum(len(i['source_ids']) for i in kernel.previous.population(nodes,initial,1));new = sum(len(i['source_ids']) for i in kernel.population(nodes,initial,1))
    return dict(analytic_pass=True,source_proof=proof,interpolation=interpolation,admission=admission,rendered_continuous_converged=report['continuous_converged'],
        rendered_applied_corrections=report['applied_iterations'],rendered_guard=report['guard_failure'],reported_systems=checked,checkpoint_systems=endpoints,
        prior_system_checks=1,objective_gradient_axes=12,original_rendered_sources=old,depth_slope_rendered_sources=new,
        degenerate_guards=guards,rejected_corruptions=rejected,input_roots_unchanged=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('slope_admission_refinement.py','check-slope-admission-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
