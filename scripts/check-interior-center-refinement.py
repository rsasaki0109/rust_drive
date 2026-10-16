#!/usr/bin/env python3
"""Independently audit two-pixel interior admission and bounded reselection."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import interior_center_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


previous = load('interior_center_original_audit','check-reselected-center-refinement.py')
private = load('interior_center_private_outer_audit','check-reselected-slope-refinement.py')
reference = load('interior_center_private_fixed_audit','check-fitted-center-refinement.py')


def sampling_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];relative = np.linalg.solve(poses[a],poses[b])
    local = np.column_stack([item['points'],np.ones(len(item['points']))])@relative.T
    q = local[:,:3];c = nodes[a]['surface'][scale]['camera']
    k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    image = q@k.T;inverse = np.linalg.inv(k);keep = np.ones(len(q),dtype=bool)
    for dv in range(-2,3):
        for du in range(-2,3):
            shifted = image.copy();shifted[:,:2] += np.array([du,dv])*q[:,2,None]
            ids,_,df,_ = reference.independent_sample(nodes[a]['surface'][scale],shifted@inverse.T)
            lengths = np.sqrt(np.einsum('ij,ij->i',df,df))
            require(np.isfinite(lengths).all() and np.all(lengths>1e-12),'finite independent interior gradient')
            keep &= np.isin(np.arange(len(q)),ids,assume_unique=True)
    return keep


def selection(nodes,poses,scale):
    seed,stages = previous.reference.selection(nodes,poses,scale);result = [];counts = {}
    for row in seed:
        keep = sampling_mask(row,nodes,poses,scale);n = int(np.count_nonzero(keep));counts[row['target'],row['source']] = n
        if n>=200:result.append(dict(row,**{key:row[key][keep] for key in ('points','normals','source_ids','normalizer')}))
    return result,[dict(s,interior_after_probes=counts.get((s['target'],s['source']),0),
        admitted_interior_count=counts.get((s['target'],s['source']),0) if counts.get((s['target'],s['source']),0)>=200 else 0) for s in stages]


def population(nodes,poses,scale):return selection(nodes,poses,scale)[0]


reference.population,reference.selection = population,selection
reference.verify.__globals__['population'] = population
reference.verify_checkpoint.__globals__['population'] = population
private.kernel,private.reference = kernel,reference
transition,compare_transition,outcome,verify,scheduler_controls = (
    private.transition,private.compare_transition,private.outcome,private.verify,private.scheduler_controls)


def source_proof():
    require(kernel.RADIUS_PIXELS==2 and kernel.ROUNDS==3 and kernel.INNER_CAP==30,'fixed interior and work bounds')
    for name in ('transition','stop','refine'):require(getattr(kernel,name) is getattr(kernel.private,name),'unchanged outer body '+name)
    for name in ('values','evaluate','equation','energy','search','refine','snapshot','checkpoint'):
        require(getattr(kernel.fixed,name) is getattr(kernel.fixed.private,name),'unchanged fixed solver body '+name)
    require(kernel.fixed.refine.__globals__['population'] is kernel.population,'one fixed interior population')
    require(kernel.fixed.sample is kernel.previous.fixed.sample and kernel.fixed.prepare is kernel.previous.fixed.prepare,'unchanged field and candidate domain')
    require(kernel.previous.fixed.population is not kernel.population,'old operational selection remains isolated')
    require(reference.verify.__globals__['population'] is population and verify.__globals__['reference'] is reference,'private independent population and outer bindings')
    require(previous.reference.verify.__globals__['population'] is previous.reference.population,'old independent selection remains isolated')
    return previous.source_proof()


def interior_controls():
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);yy,xx = np.indices((32,32));z = np.full((32,32),2.)
    q = np.array([[(12.3-c['cx'])*2/c['fx'],(12.4-c['cy'])*2/c['fy'],2.]])
    item = dict(target=0,source=1,points=q);poses = np.array([np.eye(4),np.eye(4)]);checks = []
    for missing in (False,True):
        depth = z.copy()
        if missing:depth[12,15]=0.
        xyz = np.stack([(xx-c['cx'])*depth/c['fx'],(yy-c['cy'])*depth/c['fy'],depth],axis=2)
        layer = dict(camera=c,depth=depth,vertices=xyz,normals=np.tile([0.,0.,1.],(32,32,1)))
        nodes = [dict(surface={1:layer}) for _ in range(2)]
        op,ind = kernel.fixed.prepare(nodes),reference.prepare(nodes)
        actual,expected = kernel.sampling_mask(item,op,poses,1),sampling_mask(item,ind,poses,1)
        np.testing.assert_array_equal(actual,expected);require(bool(actual[0])==(not missing),'two-pixel measured support control')
        require(kernel.previous.fixed.sampling_mask(item,op,poses,1)[0],'one-pixel control accepts both initial points')
        checks.append(bool(actual[0]))
    return dict(full_support_pass=checks[0],missing_outer_support_reject=not checks[1],original_one_pixel_accepts_both=True)


def identity_controls(audit,camera,depth):
    identity = np.eye(4);poses = np.array([identity,identity]);result = {};checked = 0
    for name,z in (('unquantized',depth),('quantized_2mm',np.round(depth/.002)*.002)):
        raw = kernel.fixed.prepare([dict(timestamp=t,root=identity.copy(),surface=kernel.fixed.base.pyramid(z,camera)) for t in (1.,1.03)])
        other = reference.prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw]);reference.compare_prepared(raw,other)
        actual = kernel.fixed.population(raw,poses,1);expected = reference.population(other,poses,1)
        reference.check_metadata(kernel.fixed.metadata(actual),expected)
        report = kernel.refine(raw,poses,None);checked += verify(report,other,poses,None,audit)
        require(report['association_rounds']==1 and not report['applied_iterations'] and report['selection_converged'],'identity passes both convergence checks without corrections')
        result[name] = dict(selected_sources=sum(len(r['source_ids']) for r in actual),
            source_metadata_agrees=True,association_rounds=1,applied_corrections=0,selection_converged=True)
    return result,checked


def controls():
    proof = source_proof();interior = interior_controls();inherited = previous.reference.controls();scheduling = scheduler_controls()
    audit = load('interior_center_rendered','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA
    depth = np.round(audit.rays.render(identity)/.002)*.002
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = np.round(audit.rays.render(truth)/.002)*.002
    tracker = kernel.fixed.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered warm start');initial = np.asarray(warm['revised_roots'])
    perturbation = np.array([.006,-.002,.001,.0008,-.0006,.0004])
    initial[1] = audit.increment(initial[1],perturbation)
    nodes = kernel.fixed.prepare([dict(timestamp=t,root=p.copy(),surface=kernel.fixed.base.pyramid(z,camera))
        for t,p,z in zip((1.,1.03),initial,(depth,source),strict=True)])
    other = reference.prepare([dict(n,surface=audit.pyramid(z,camera)) for n,z in zip(nodes,(depth,source),strict=True)])
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    require(report['association_rounds']>=2,'actual rendered reselection reaches multiple rounds')
    # The full independent inner audit rejects source/weight/matrix/search
    # corruption; outer controls additionally reject selection and scheduling.
    changes = [('false outer convergence',lambda r:r.update(selection_converged=not r['selection_converged'])),
        ('wrong round count',lambda r:r.update(association_rounds=1)),
        ('wrong total updates',lambda r:r.update(applied_iterations=r['applied_iterations']+1)),
        ('wrong fresh matrix',lambda r:r['rounds'][0]['reselected']['equation'].update(g=(np.asarray(r['rounds'][0]['reselected']['equation']['g'])+1).tolist())),
        ('wrong transition loss',lambda r:r['rounds'][0]['transition'][0].update(removed_count=999)),
        ('changed incoming round start',lambda r:r['rounds'][1]['batch']['initial_poses'][1][0].__setitem__(3,9.))]
    rejected = []
    for name,change in changes:
        bad = copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected outer corruption '+name)
    guards = {}
    for name,z in (('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))):
        raw = kernel.fixed.prepare([dict(timestamp=t,root=identity.copy(),surface=kernel.fixed.base.pyramid(z,camera)) for t in (1.,1.03)])
        ind = reference.prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw]);poses = np.array([identity,identity])
        result = kernel.refine(raw,poses,None);checked += verify(result,ind,poses,None,audit)
        require(result['association_rounds']==1 and not result['applied_iterations'],'no artificial outer progress for degenerate images')
        require(result['termination']=='inner_guard' and not result['selection_converged'],'degenerate data keep support guards')
        guards[name] = result['guard_failure']
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    prior = kernel.fixed.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    prior_before = prior.encode();poses3 = np.array([identity,initial[1],initial[1]])
    poses3[2] = audit.increment(poses3[2],np.array([-.0004,.0002,-.0001,-.0001,.00008,.00017]))
    third = dict(timestamp=1.06,root=poses3[2].copy(),surface=nodes[1]['surface'])
    three = nodes+[third];ind_three = other+[dict(third,surface=other[1]['surface'])]
    with_prior = kernel.refine(three,poses3,prior);checked += verify(with_prior,ind_three,poses3,prior_before,audit)
    require(prior.encode()==prior_before,'one original nonlinear prior remains unchanged across rounds')
    input_guards = []
    bad_origin = initial.copy();bad_origin[0,0,3]=.1
    new_prior = kernel.fixed.base.Prior(1.,[1.03],[identity],np.eye(6),np.zeros(6),[],1)
    for name,poses,bound_prior in (('pose count',initial[:1],None),('fixed origin',bad_origin,None),('new acquisition prior',initial,new_prior)):
        try:kernel.refine(nodes,poses,bound_prior)
        except ValueError:input_guards.append(name)
        else:raise ValueError('unguarded reselected input '+name)
    identity_cases,identity_systems = identity_controls(audit,camera,audit.rays.render(identity));checked += identity_systems
    return dict(analytic_pass=True,source_proof=proof,interior=interior,inherited_analytic_pass=inherited['analytic_pass'],inherited_prior_gradient_axes=inherited['objective_gradient_axes'],
        scheduler_cases=scheduling,rejected_corruptions=rejected,rendered_rounds=report['association_rounds'],
        rendered_applied_corrections=report['applied_iterations'],rendered_selection_converged=report['selection_converged'],
        rendered_depth_quantization_m=.002,reported_systems=checked,guards=guards,input_roots_unchanged=True,
        rendered_initial_left_perturbation=perturbation.tolist(),
        nonzero_prior_unchanged=True,nonzero_prior_rounds=with_prior['association_rounds'],
        input_guards=input_guards,identity_admission_agreement=True,identity_controls=identity_cases,rendered=report,rendered_with_prior=with_prior)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('interior_center_refinement.py','check-interior-center-refinement.py')}
    write(out/'summary.json',report)


if __name__=='__main__':main()
