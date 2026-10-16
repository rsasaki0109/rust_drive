#!/usr/bin/env python3
"""Independent batch scheduling, frozen updates and fresh-convergence audit."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path
import numpy as np

import alternating_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


reference = load('alternating_frozen_reference','check-fixed-depth-refinement.py')
independent_system = reference.independent_system
failure,small = reference.failure,reference.small
expected_search,compare_search = reference.expected_search,reference.compare_search
check_correction = reference.check_correction


def verify_batch(report,nodes,initial,prior,audit):
    objective = load('fixed_independent_objective','check-descent-keyframes.py')
    require(report['scale'] in (4,2,1) and type(report['cap']) is int and 1 <= report['cap'] <= 3,'fixed refinement scale and ten-correction bound')
    poses = np.asarray(initial).copy();np.testing.assert_array_equal(report['initial_poses'],poses)
    matches = objective.selection(nodes,poses,report['scale'])
    metadata = [{k:x[k] for k in ('target','source','count','pixel_sha256')} for x in matches]
    require(report['frozen_population'] == metadata,'single initial measured population')
    require(len(report['trace']) <= report['cap'] and report['applied_iterations'] == len(report['trace']),'bounded honest correction count')
    np.testing.assert_allclose(report['initial_fixed_cost'],objective.objective(matches,nodes,poses,prior,audit),atol=1e-10,rtol=1e-9)
    checked = 0
    for item in report['trace']:
        np.testing.assert_array_equal(item['before'],poses)
        e = independent_system(matches,nodes,poses,prior,audit,objective);audit.compare_equation(item['equation'],e)
        reason,d,condition = failure(e,len(nodes)-1,audit)
        require(reason is None and not small(d),'no correction after failure or fixed convergence')
        np.testing.assert_allclose(item['delta'],d,atol=1e-9,rtol=1e-7)
        np.testing.assert_allclose(item['condition'],condition,atol=1e-6,rtol=1e-8)
        expected,receipt = expected_search(matches,nodes,poses,prior,d,e['g'],audit,objective)
        require(expected is not None,'applied fixed correction has a valid multiplier')
        compare_search(item['line_search'],receipt)
        np.testing.assert_allclose(item['after'],expected,atol=1e-10,rtol=0)
        np.testing.assert_array_equal(item['after'][0],np.eye(4));poses = np.asarray(item['after']).copy();checked += 1
    fixed = independent_system(matches,nodes,poses,prior,audit,objective)
    fresh = audit.independent_equation(nodes,poses,report['scale'],prior)
    audit.compare_equation(report['final_fixed_equation'],fixed);audit.compare_equation(report['final_fresh_equation'],fresh)
    reason,d,_ = failure(fixed,len(nodes)-1,audit);converged = reason is None and small(d)
    if report['guard_failure'] == 'line search':
        require(reason is None and not converged,'search failure follows admissible unconverged solve')
        after,_ = expected_search(matches,nodes,poses,prior,d,fixed['g'],audit,objective)
        require(after is None,'independently reproduce exhausted frozen search')
    else:
        require(report['guard_failure'] == reason,'independently reproduced frozen termination')
        require(reason is not None or converged or len(report['trace']) == report['cap'],'no unexplained early frozen stop')
    require(report['fixed_converged'] == converged,'no false fixed convergence')
    check_correction(report['final_fixed_correction'],fixed,len(nodes)-1,audit)
    check_correction(report['final_fresh_correction'],fresh,len(nodes)-1,audit)
    np.testing.assert_array_equal(report['final_poses'],poses)
    np.testing.assert_allclose(report['final_fixed_cost'],objective.objective(matches,nodes,poses,prior,audit),atol=1e-10,rtol=1e-9)
    new_matches = objective.selection(nodes,poses,report['scale'])
    np.testing.assert_allclose(report['final_fresh_cost'],objective.objective(new_matches,nodes,poses,prior,audit),atol=1e-10,rtol=1e-9)
    return checked+2


def verify(report,nodes,initial,prior,audit):
    require(report['total_cap'] == 10 and report['inner_cap'] == 3,'independent alternating correction limits')
    require(report['scale'] in (4,2,1) and 1 <= len(report['batches']) <= 10,'bounded alternating batches/scale')
    poses = np.asarray(initial).copy();np.testing.assert_array_equal(report['initial_poses'],poses)
    checked = applied = 0;failure_reason = None;converged = False;finished = False
    for index,item in enumerate(report['batches']):
        require(not finished,'no batch after convergence, failure or budget exhaustion')
        require(item['cap'] == min(3,10-applied) and item['scale'] == report['scale'],'prescribed remaining-budget batch size')
        np.testing.assert_array_equal(item['initial_poses'],poses)
        checked += verify_batch(item,nodes,poses,prior,audit)
        poses = np.asarray(item['final_poses']);applied += len(item['trace'])
        require(applied <= 10,'ten applied corrections across all batches')
        failure_reason = item['guard_failure'] or item['final_fresh_correction']['guard_failure']
        converged = failure_reason is None and item['final_fresh_correction']['converged']
        if failure_reason is None and not converged and not item['trace']:failure_reason = 'no progress'
        finished = failure_reason is not None or converged or applied == 10
    require(finished,'no unexplained early alternating stop')
    require(report['applied_iterations'] == applied and report['association_rounds'] == len(report['batches']),'honest total corrections and rounds')
    require(report['guard_failure'] == failure_reason and report['fresh_converged'] == converged,'honest global fresh convergence/failure')
    expected = 'guard' if failure_reason is not None else ('converged' if converged else 'budget')
    require(report['termination'] == expected,'independently checked termination cause')
    last = report['batches'][-1]
    for key in ('final_poses','final_fresh_equation','final_fresh_correction','final_fresh_cost'):
        require(report[key] == last[key],'global final state is last batch output')
    np.testing.assert_array_equal(report['final_poses'],poses)
    return checked


def source_proof():
    def body(file,function):
        source = (ROOT/'scripts'/file).read_text()
        node = next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name == function)
        return ast.get_source_segment(source,node)
    old = body('fixed_depth_refinement.py','refine')
    expected = old.replace('def refine(nodes,initial,prior,scale=1):','def batch(nodes,initial,prior,scale=1,cap=3):')
    expected = expected.replace('At most ten corrections, no re-association, and no tracker state commit.','One bounded frozen batch, no re-association or tracker state commit.')
    expected = expected.replace('    require(2 <= len(nodes)',"    require(isinstance(cap,int) and not isinstance(cap,bool) and 1 <= cap <= 3,'one to three corrections per batch')\n    require(2 <= len(nodes)",1)
    expected = expected.replace('range(11)','range(cap+1)').replace('iteration == 10','iteration == cap').replace('cap=10,initial_poses','cap=cap,initial_poses')
    require(body('alternating_depth_refinement.py','batch') == expected,'batch changes only bounded correction cap')
    old = body('check-fixed-depth-refinement.py','verify').replace('def verify(','def verify_batch(',1)
    old = old.replace("report['cap'] == 10","type(report['cap']) is int and 1 <= report['cap'] <= 3")
    old = old.replace("len(report['trace']) <= 10","len(report['trace']) <= report['cap']")
    old = old.replace("len(report['trace']) == 10","len(report['trace']) == report['cap']")
    require(body('check-alternating-depth-refinement.py','verify_batch') == old,'independent batch audit changes only bound')
    return True


def controls():
    proof = source_proof();inherited = reference.controls()
    audit = load('alternating_raw_audit','check-reciprocal-keyframes.py').auditor()
    identity = np.eye(4);camera = audit.rays.CAMERA;depth = audit.rays.render(identity)
    nodes = [dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(depth,camera)) for t in (1.,1.03)]
    independent = [dict(n,surface=audit.pyramid(depth,camera)) for n in nodes]
    poses = np.array([identity,identity]);same = kernel.refine(nodes,poses,None)
    checked = verify(same,independent,poses,None,audit)
    require(same['fresh_converged'] and not same['applied_iterations'] and same['association_rounds'] == 1,'measured identity stops without updates')
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'sensor-only rendered warm start')
    initial = np.asarray(warm['revised_roots'])
    initial[1] = audit.increment(initial[1],np.array([.02,-.01,.005,.012,-.014,.009]))
    nodes[1]['surface'] = kernel.base.pyramid(source,camera);independent[1]['surface'] = audit.pyramid(source,camera)
    moved = kernel.refine(nodes,initial,None);checked += verify(moved,independent,initial,None,audit)
    require(moved['association_rounds'] >= 2,'multiple real rendered association rounds exercised')
    single = reference.kernel.refine(nodes,initial,None)
    single_checked = reference.verify(single,independent,initial,None,audit)
    require(single['fixed_converged'] and not single['final_fresh_correction']['converged']
        and moved['fresh_converged'],'same rendered start needs fresh associations for convergence')
    estimated = np.asarray(moved['final_poses'])[-1]
    position_error = float(np.linalg.norm(estimated[:3,3]-truth[:3,3]))
    rotation_error = float(np.arccos(np.clip((np.trace(truth[:3,:3].T@estimated[:3,:3])-1)/2,-1,1)))
    require(position_error <= .004 and rotation_error <= .004,'rendered alternating optical motion accuracy')
    measured_guards = {}
    for name,z in [('missing support',np.zeros_like(depth)),('plane',np.full_like(depth,2.))]:
        raw = [dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)]
        expected = [dict(n,surface=audit.pyramid(z,camera)) for n in raw]
        result = kernel.refine(raw,poses,None);checked += verify(result,expected,poses,None,audit)
        require(result['termination'] == 'guard' and not result['applied_iterations'] and not result['fresh_converged'],'measured alternating guard stops without updates')
        measured_guards[name] = result['guard_failure']
    # Isolated scheduler fixtures check worst-case work and no-progress
    # bookkeeping only; the rendered cases above verify real matrix updates.
    original_batch = kernel.batch;caps = []
    def fixture(nodes,initial,prior,scale,cap):
        caps.append(cap);after = np.asarray(initial).copy();after[1,0,3] += .001*cap
        return dict(applied_iterations=cap,final_poses=after.tolist(),guard_failure=None,
            final_fresh_correction=dict(converged=False,guard_failure=None),final_fresh_equation={},final_fresh_cost=0.)
    try:
        kernel.batch = fixture;worst = kernel.refine(nodes,poses,None)
        require(caps == [3,3,3,1] and worst['applied_iterations'] == 10 and worst['termination'] == 'budget','independent worst-case ten-correction schedule')
        def no_progress(nodes,initial,prior,scale,cap):
            r = fixture(nodes,initial,prior,scale,cap);r['applied_iterations'] = 0;r['final_poses'] = np.asarray(initial).tolist();return r
        kernel.batch = no_progress;stalled = kernel.refine(nodes,poses,None)
        require(stalled['guard_failure'] == 'no progress' and not stalled['fresh_converged'] and stalled['association_rounds'] == 1,'zero progress is never fresh convergence')
    finally:kernel.batch = original_batch
    # A completely coherent shortened report must still fail the stopping rule.
    def early(r):
        r['batches'].pop();last = r['batches'][-1]
        r['applied_iterations'] = sum(x['applied_iterations'] for x in r['batches'])
        r['association_rounds'] = len(r['batches'])
        for key in ('final_poses','final_fresh_equation','final_fresh_correction','final_fresh_cost'):r[key] = last[key]
    rejected = []
    changes = [('wrong total budget',lambda r:r.update(total_cap=11)),
        ('wrong inner budget',lambda r:r.update(inner_cap=4)),
        ('wrong remaining batch cap',lambda r:r['batches'][0].update(cap=2)),
        ('unjustified early stop',early),
        ('stale batch anchor',lambda r:r['batches'][1].update(initial_poses=r['initial_poses'])),
        ('false fresh convergence',lambda r:r.update(fresh_converged=not r['fresh_converged'])),
        ('wrong new population',lambda r:r['batches'][1]['frozen_population'][0].update(pixel_sha256='0'*64)),
        ('wrong applied multiplier',lambda r:r['batches'][0]['trace'][0]['line_search'].update(accepted_alpha=.5)),
        ('wrong final fresh system',lambda r:r['batches'][-1]['final_fresh_equation'].update(g=(np.asarray(r['batches'][-1]['final_fresh_equation']['g'])+1).tolist()))]
    for name,change in changes:
        bad = copy.deepcopy(moved);change(bad)
        try:verify(bad,independent,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected alternating corruption: '+name)
    extra = copy.deepcopy(same);extra['batches'].append(copy.deepcopy(extra['batches'][0]));extra['association_rounds'] += 1
    try:verify(extra,independent,poses,None,audit)
    except (ValueError,AssertionError):rejected.append('batch after fresh convergence')
    else:raise ValueError('continued after fresh convergence')
    for node in nodes:np.testing.assert_array_equal(node['root'],identity)
    return dict(analytic_pass=True,source_proof=proof,independent_reported_systems=checked,
        identity_fresh_converged_without_updates=True,rendered_association_rounds=moved['association_rounds'],
        rendered_applied_iterations=moved['applied_iterations'],rendered_fresh_converged=moved['fresh_converged'],
        same_start_single_fixed_fresh_converged=single['final_fresh_correction']['converged'],
        rendered_position_error_m=position_error,rendered_rotation_error_rad=rotation_error,
        separately_checked_single_fixed_systems=single_checked,
        measured_guard_failures=measured_guards,
        isolated_worst_case_schedule=[3,3,3,1],no_progress_rejected=True,fixtures_are_not_image_evidence=True,
        rejected_corruptions=rejected,input_roots_unchanged=True,inherited_fixed_controls=inherited)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in
        ('alternating_depth_refinement.py','check-alternating-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
