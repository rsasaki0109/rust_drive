#!/usr/bin/env python3
"""Independent objective, spectral directions and bounded candidate selection."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import damped_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


previous = load('damped_previous_audit','check-deep-search-depth-refinement.py')
population,check_metadata,system,objective = previous.population,previous.check_metadata,previous.system,previous.objective
reference,verify_checkpoint = previous.reference,previous.verify_checkpoint
DAMPING = (1/1000,1/100,1/10,1,10,100,1000,10000)


def search(matches,nodes,poses,prior,scale,d,e,audit):
    incumbent,gn = previous.search(matches,nodes,poses,prior,scale,d,e['g'],audit)
    before = objective(matches,nodes,poses,prior,scale,audit);margin = max(1.,abs(before))*1e-12
    value = None if incumbent is None else gn['trials'][-1]['cost']
    choice = None if incumbent is None else 'gauss_newton';applied = None if incumbent is None else d.tolist()
    triggered = gn['accepted_alpha'] is None or gn['accepted_alpha'] < 1/16
    rows = []
    if triggered:
        for damping in DAMPING:
            row = dict(damping=damping,delta=None,condition=None,directional_derivative=None,cost=None,limit=None,guard_failure=None)
            matrix = np.array(e['h'],copy=True);diagonal = np.diag_indices(len(e['g']))
            matrix[diagonal] *= 1+damping
            try:
                direction,condition = audit.independent_solve(matrix,e['g'])
                row.update(delta=direction.tolist(),condition=condition)
                require(np.all(np.sqrt(np.sum(direction[:,:3]**2,axis=1)) <= .1)
                    and np.all(np.sqrt(np.sum(direction[:,3:]**2,axis=1)) <= .15),'increment')
                slope = float(np.dot(e['g'],direction.reshape(-1)))
                require(np.isfinite(slope) and slope <= 0,'nonascending damped proposal')
                row.update(directional_derivative=slope,limit=before+slope/10000+margin)
                candidate = np.asarray(poses).copy()
                for node,delta in enumerate(direction,1):candidate[node] = audit.increment(poses[node],delta)
                cost = objective(matches,nodes,candidate,prior,scale,audit);row['cost'] = cost
                eligible = cost <= row['limit'] and before-cost > margin
                if eligible and (value is None or value-cost > margin):
                    incumbent,value,choice,applied = candidate,cost,len(rows),direction.tolist()
            except ValueError as error:row['guard_failure'] = str(error)
            rows.append(row)
    return incumbent,dict(gauss_newton=gn,triggered=triggered,damped_trials=rows,selected=choice,applied_delta=applied,accepted_cost=value)


def compare_search(actual,wanted):
    require(set(actual) == set(wanted),'complete damped receipt schema')
    previous.compare_search(actual['gauss_newton'],wanted['gauss_newton'])
    require(actual['triggered'] == wanted['triggered'] and type(actual['selected']) is type(wanted['selected'])
        and actual['selected'] == wanted['selected'],'independent trigger and selected direction')
    require(len(actual['damped_trials']) == len(wanted['damped_trials']),'all bounded direction trials')
    for actual_row,expected in zip(actual['damped_trials'],wanted['damped_trials'],strict=True):
        require(set(actual_row) == set(expected) and actual_row['damping'] == expected['damping']
            and actual_row['guard_failure'] == expected['guard_failure'],'honest damping and candidate validity')
        for key in ('delta','condition','directional_derivative','cost','limit'):
            if expected[key] is None:require(actual_row[key] is None,'invalid candidate has no fabricated '+key)
            else:np.testing.assert_allclose(actual_row[key],expected[key],atol=1e-9 if key in ('delta','condition') else 2e-12,rtol=1e-7,err_msg=key)
    for key in ('applied_delta','accepted_cost'):
        if wanted[key] is None:require(actual[key] is None,'failed direction search '+key)
        else:np.testing.assert_allclose(actual[key],wanted[key],atol=1e-10,rtol=1e-7,err_msg=key)


def verify(report,nodes,initial,prior,audit):
    require(type(report['cap']) is int and report['cap'] in (10,20,30) and report['scale'] in (4,2,1),'unchanged correction limit')
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
        expected,receipt = search(matches,nodes,poses,prior,scale,d,e,audit)
        require(expected is not None,'applied continuous update has valid Armijo step');compare_search(item['line_search'],receipt)
        np.testing.assert_allclose(item['after'],expected,atol=1e-10,rtol=0)
        np.testing.assert_array_equal(item['after'][0],np.eye(4));poses = np.asarray(item['after'])
    final = system(matches,nodes,poses,prior,scale,audit);audit.compare_equation(report['final_equation'],final)
    discrete = audit.independent_equation(nodes,poses,scale,prior);audit.compare_equation(report['final_discrete_equation'],discrete)
    reason,d,_ = reference.failure(final,len(nodes)-1,audit);converged = reason is None and reference.small(d)
    if report['guard_failure'] == 'line search':
        require(reason is None and not converged,'failed search follows admissible unconverged solve')
        expected,receipt = search(matches,nodes,poses,prior,scale,d,final,audit)
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


def normalized(report):
    result = copy.deepcopy(report)
    for step in result['trace']:
        require(step['line_search']['selected'] == 'gauss_newton','normalize only unchanged GN choices')
        step['line_search'] = step['line_search']['gauss_newton']
    if result['failed_search'] is not None:result['failed_search'] = result['failed_search']['gauss_newton']
    return result


def source_proof():
    def body(file,function):
        text = (ROOT/'scripts'/file).read_text()
        node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name == function)
        return ast.get_source_segment(text,node)
    old = body('deep_search_depth_refinement.py','refine').replace("search(matches,nodes,poses,prior,scale,d,e['g'])",'search(matches,nodes,poses,prior,scale,d,e)')
    require(body('damped_depth_refinement.py','refine') == old,'only direction search dependency changes in refiner body')
    old = body('check-long-depth-refinement.py','verify').replace("scale,d,e['g'],audit)",'scale,d,e,audit)')
    old = old.replace("scale,d,final['g'],audit)",'scale,d,final,audit)')
    require(body('check-damped-depth-refinement.py','verify') == old,'unchanged verifier outside direction search dependency')
    for name in ('population','metadata','equation','energy','snapshot','checkpoint','base','discrete'):
        require(getattr(kernel,name) is getattr(kernel.previous,name),'unchanged original objective dependency '+name)
    require(tuple(DAMPING) == kernel.DAMPING,'fixed damping grid agreement')
    return True


def controls():
    proof = source_proof();inherited = load('damped_unmodified_controls','check-deep-search-depth-refinement.py').controls()
    audit = load('damped_rendered_oracle','check-reciprocal-keyframes.py').auditor()
    identity = np.eye(4);camera = audit.rays.CAMERA;depth = audit.rays.render(identity)
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = [dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(nodes,(depth,source),strict=True)]
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    require(normalized(report) == kernel.previous.refine(nodes,initial,None),'complete rendered GN control report')
    shifted = initial.copy();shifted[1] = audit.increment(shifted[1],np.array([.02,-.01,.005,.012,-.014,.009]))
    perturbed = kernel.refine(nodes,shifted,None);checked += verify(perturbed,other,shifted,None,audit)
    endpoint_checks = sum(verify_checkpoint(kernel.checkpoint(perturbed,nodes,None,cap),perturbed,other,shifted,None,audit,cap) for cap in (10,20,30))
    directions = sum(len(step['line_search']['damped_trials']) for q in (report,perturbed) for step in q['trace'])
    selected = sum(type(step['line_search']['selected']) is int for q in (report,perturbed) for step in q['trace'])
    # Measured degenerate images cannot be repaired by adding a diagonal: the
    # original proposal guard runs before any alternative-direction search.
    guards = {}
    for name,z in (('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))):
        raw = [dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)]
        independent = [dict(n,surface=audit.pyramid(z,camera)) for n in raw]
        result = kernel.refine(raw,np.array([identity,identity]),None);checked += verify(result,independent,np.array([identity,identity]),None,audit)
        require(result['termination'] == 'guard' and not result['trace'] and result['failed_search'] is None,'no damping before original admissibility')
        guards[name] = result['guard_failure']
    # An isolated authored quadratic has a closed-form optimum. This checks
    # candidate selection even when the rendered fits do not trigger it.
    # Private modules isolate cost substitution from all measured-data code.
    quadratic = load('damped_quadratic_operational','damped_depth_refinement.py')
    quadratic.previous = load('damped_quadratic_gn','deep_search_depth_refinement.py')
    oracle = load('damped_quadratic_oracle','check-damped-depth-refinement.py')
    h = np.eye(6)*10;h[0,1] = h[1,0] = 9;g = np.array([-.01,.009,0.,0.,0.,0.])
    def value(matches,nodes,poses,prior,scale):
        t = poses[1][:3,3];return float(1+g[:3]@t+500*(t@t))
    def independent_value(matches,nodes,poses,prior,scale,audit):
        t = poses[1][:3,3]
        return float(1+sum(coefficient*coordinate+500*coordinate**2 for coefficient,coordinate in zip((-.01,.009,0.),t,strict=True)))
    quadratic.energy = quadratic.previous.energy = value
    oracle.objective = oracle.previous.objective = independent_value
    poses = np.array([identity,identity]);d,_ = kernel.base.solve(h,g);d = d.reshape(1,6)
    candidate,receipt = quadratic.search([],[],poses,None,1,d,dict(h=h,g=g))
    expected,wanted = oracle.search([],[],poses,None,1,d,dict(h=h,g=g),audit);compare_search(receipt,wanted)
    require(receipt['triggered'] and type(receipt['selected']) is int and kernel.DAMPING[receipt['selected']] == 100,
        'closed-form quadratic chooses damping100')
    determinant = 1010**2-9**2
    hand = np.array([(1010*.01+9*.009)/determinant,(-9*.01-1010*.009)/determinant,0.])
    np.testing.assert_allclose(np.asarray(candidate)[1,:3,3],hand,atol=1e-15,rtol=0)
    np.testing.assert_allclose(candidate,expected,atol=1e-14,rtol=0)
    require(receipt['accepted_cost'] < receipt['gauss_newton']['trials'][-1]['cost']-1e-12,'quadratic direction improves on shortened GN')
    rejected = []
    for name,change in (('false selected direction',lambda r:r.update(selected='gauss_newton')),
        ('changed damping',lambda r:r['damped_trials'][0].update(damping=999)),
        ('false damped solve',lambda r:r['damped_trials'][0].update(delta=(np.asarray(r['damped_trials'][0]['delta'])+1).tolist())),
        ('truncated alternatives',lambda r:r['damped_trials'].pop()),
        ('false accepted cost',lambda r:r.update(accepted_cost=99.))):
        bad = copy.deepcopy(receipt);change(bad)
        try:compare_search(bad,receipt)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected damped search corruption: '+name)
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    return dict(analytic_pass=True,source_proof=proof,inherited_deep_controls=inherited,
        additional_rendered_reported_systems=checked,additional_checkpoint_systems=endpoint_checks,
        rendered_warm_converged=report['continuous_converged'],perturbed_converged=perturbed['continuous_converged'],
        perturbed_applied_corrections=perturbed['applied_iterations'],perturbed_guard=perturbed['guard_failure'],
        rendered_damped_solve_checks=directions,rendered_damped_choices=selected,original_degenerate_guards=guards,
        authored_quadratic_damped_solve_checks=8,authored_quadratic_selected_damping=100,
        authored_quadratic_is_not_measured_image_or_physical_accuracy_evidence=True,
        rejected_damped_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('damped_depth_refinement.py','check-damped-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
