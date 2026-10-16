#!/usr/bin/env python3
"""Independent spectral audit of extended fixed-source correction budgets."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import extended_slope_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


previous = load('extended_slope_original_verifier','check-one-sided-depth-refinement.py')
prepare,population = previous.prepare,previous.population
check_metadata,system,objective,reference = previous.check_metadata,previous.system,previous.objective,previous.reference
search = previous.verify.__globals__['search']
compare_search = previous.verify.__globals__['compare_search']
compare_prepared = previous.compare_prepared


def source_proof():
    def body(file,name):
        text = (ROOT/'scripts'/file).read_text();node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    for name in ('refine','snapshot','checkpoint'):
        old = body('one_sided_depth_refinement.py',name).replace('(10,20,30)','(10,20,30,60,90)')
        if name=='refine':old = old.replace('cap=30','cap=90')
        require(body('extended_slope_refinement.py',name)==old,'only fixed cap set/default changed '+name)
    for name in ('population','values','evaluate','equation','energy','search','prepare','sample','metadata','base','discrete'):
        require(getattr(kernel,name) is getattr(kernel.previous,name),'unchanged original dependency '+name)
    for name in ('verify','verify_checkpoint'):
        require(body('check-extended-slope-refinement.py',name)==body('check-normal-hermite-depth-refinement.py',name).replace('(10,20,30)','(10,20,30,60,90)'),
            'only independent cap admission changed '+name)
    require(search is previous.verify.__globals__['search'],'unchanged independent candidate search')
    return True


def verify(report,nodes,initial,prior,audit):
    require(type(report['cap']) is int and report['cap'] in (10,20,30,60,90) and report['scale'] in (4,2,1),'unchanged correction limit')
    poses = np.asarray(initial).copy();np.testing.assert_array_equal(report['initial_poses'],poses)
    scale = report['scale'];matches = population(nodes,poses,scale);check_metadata(report['frozen_population'],matches)
    np.testing.assert_allclose(report['initial_cost'],objective(matches,nodes,poses,prior,scale,audit),atol=1e-10,rtol=1e-8)
    require(report['applied_iterations'] == len(report['trace']) <= report['cap'],'honest bounded continuous corrections')
    checked = 0
    for item in report['trace']:
        np.testing.assert_array_equal(item['before'],poses)
        e = system(matches,nodes,poses,prior,scale,audit);audit.compare_equation(item['equation'],e);checked += 1
        reason,d,condition = reference.failure(e,len(nodes)-1,audit)
        require(reason is None and not reference.small(d),'no update after guard or convergence')
        np.testing.assert_allclose(item['delta'],d,atol=1e-9,rtol=1e-7)
        np.testing.assert_allclose(item['condition'],condition,atol=1e-6,rtol=1e-8)
        expected,receipt = search(matches,nodes,poses,prior,scale,d,e['g'],audit)
        require(expected is not None,'applied continuous update has valid Armijo step');compare_search(item['line_search'],receipt)
        np.testing.assert_allclose(item['after'],expected,atol=1e-10,rtol=0)
        np.testing.assert_array_equal(item['after'][0],np.eye(4));poses = np.asarray(item['after'])
    final = system(matches,nodes,poses,prior,scale,audit);audit.compare_equation(report['final_equation'],final)
    discrete = audit.independent_equation(nodes,poses,scale,prior);audit.compare_equation(report['final_discrete_equation'],discrete)
    reason,d,_ = reference.failure(final,len(nodes)-1,audit);converged = reason is None and reference.small(d)
    if report['guard_failure'] == 'line search':
        require(reason is None and not converged,'failed search follows admissible unconverged solve')
        expected,receipt = search(matches,nodes,poses,prior,scale,d,final['g'],audit)
        require(expected is None,'independently reproduced continuous search exhaustion');compare_search(report['failed_search'],receipt)
    else:
        require(report['guard_failure'] == reason and report['failed_search'] is None,'independent continuous stop guard')
        require(reason is not None or converged or len(report['trace']) == report['cap'],'no unexplained early continuous stop')
    require(report['continuous_converged'] == converged,'no false continuous convergence')
    termination = 'guard' if report['guard_failure'] is not None else ('converged' if converged else 'budget')
    require(report['termination'] == termination,'honest continuous termination')
    reference.check_correction(report['final_correction'],final,len(nodes)-1,audit)
    reference.check_correction(report['final_discrete_correction'],discrete,len(nodes)-1,audit)
    np.testing.assert_array_equal(report['final_poses'],poses)
    np.testing.assert_allclose(report['final_cost'],objective(matches,nodes,poses,prior,scale,audit),atol=1e-10,rtol=1e-8)
    return checked+2


def verify_checkpoint(actual,report,nodes,initial,prior,audit,cap):
    require(type(cap) is int and cap in (10,20,30,60,90) and actual['requested_cap'] == cap,'fixed checkpoint limits')
    used = min(cap,len(report['trace']));require(actual['actual_applied_corrections'] == used,'honest checkpoint applied count')
    require(actual['terminal_carried_forward'] == (used < cap),'explicit early terminal carry-forward')
    poses = np.asarray(initial if used == 0 else report['trace'][used-1]['after'])
    np.testing.assert_array_equal(actual['final_poses'],poses)
    matches = population(nodes,np.asarray(initial),report['scale'])
    e = system(matches,nodes,poses,prior,report['scale'],audit);reason,d,_ = reference.failure(e,len(nodes)-1,audit)
    converged = reason is None and reference.small(d)
    if reason is not None:termination,failure = 'guard',reason
    elif converged:termination,failure = 'converged',None
    elif used == cap:termination,failure = 'budget',None
    else:
        require(used < cap and report['guard_failure'] == 'line search','terminal search precedes requested endpoint')
        termination,failure = 'guard','line search'
    require(actual['termination'] == termination and actual['guard_failure'] == failure and actual['continuous_converged'] == converged,'independent checkpoint event order')
    audit.compare_equation(actual['final_equation'],e);reference.check_correction(actual['final_correction'],e,len(nodes)-1,audit)
    fresh = audit.independent_equation(nodes,poses,report['scale'],prior)
    audit.compare_equation(actual['final_discrete_equation'],fresh)
    reference.check_correction(actual['final_discrete_correction'],fresh,len(nodes)-1,audit)
    np.testing.assert_allclose(actual['final_cost'],objective(matches,nodes,poses,prior,report['scale'],audit),atol=1e-10,rtol=1e-8)
    return 2


def controls():
    proof = source_proof();interpolation = previous.support_controls()
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
    return dict(analytic_pass=True,source_proof=proof,interpolation=interpolation,rendered_continuous_converged=report['continuous_converged'],
        rendered_applied_corrections=report['applied_iterations'],rendered_guard=report['guard_failure'],reported_systems=checked,checkpoint_systems=endpoints,
        prior_system_checks=1,objective_gradient_axes=12,original_rendered_sources=old,depth_slope_rendered_sources=new,
        degenerate_guards=guards,rejected_corruptions=rejected,input_roots_unchanged=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('extended_slope_refinement.py','check-extended-slope-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
