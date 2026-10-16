#!/usr/bin/env python3
"""Independent thirty-correction chaining, endpoint and inherited raw-system audit."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import budget_depth_refinement as kernel
import depth_correspondence_oracle as oracle
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


reference = load('budget_alternating_audit','check-alternating-depth-refinement.py')
compare = load('budget_turnover_audit','check-depth-correspondence.py').compare


def shape(report,initial):
    require(type(report['total_cap']) is int and report['total_cap'] == 30
        and type(report['segment_cap']) is int and report['segment_cap'] == 10,'fixed diagnostic correction budgets')
    require(report['scale'] in (4,2,1) and 1 <= len(report['segments']) <= 3,'one to three local segments')
    require(len(report['endpoints']) == len(report['segments']),'one diagnostic per returned segment')
    np.testing.assert_array_equal(report['initial_poses'],initial)
    poses = np.asarray(initial);applied = rounds = 0;finished = False
    for item in report['segments']:
        require(not finished,'no segment after fresh convergence or guard')
        require(item['total_cap'] == 10 and item['inner_cap'] == 3 and item['scale'] == report['scale'],'unchanged local schedule')
        np.testing.assert_array_equal(item['initial_poses'],poses)
        require(type(item['applied_iterations']) is int and 0 <= item['applied_iterations'] <= 10,'bounded segment corrections')
        expected = 'guard' if item['guard_failure'] is not None else ('converged' if item['fresh_converged'] else 'budget')
        require(item['termination'] == expected,'honest segment termination')
        if expected == 'budget':require(item['applied_iterations'] == 10,'budget stop consumes ten corrections')
        require(not item['fresh_converged'] or item['guard_failure'] is None,'guard cannot imply convergence')
        finished = expected != 'budget';poses = np.asarray(item['final_poses'])
        applied += item['applied_iterations'];rounds += item['association_rounds']
    require(finished or len(report['segments']) == 3,'no unexplained early extension stop')
    require(report['applied_iterations'] == applied <= 30 and report['association_rounds'] == rounds,'honest extended totals')
    for key in ('fresh_converged','guard_failure','termination','final_poses','final_fresh_equation','final_fresh_correction','final_fresh_cost'):
        require(report[key] == report['segments'][-1][key],'extended output equals last segment')


def verify_endpoint(actual,segment,nodes,prior,audit):
    batch = segment['batches'][-1];before = np.asarray(batch['initial_poses']);after = np.asarray(batch['final_poses'])
    expected = [];index = 0
    for source in range(1,len(nodes)):
        for target in range(source):
            old = oracle.independent_matches([dict(nodes[target],root=before[target])],nodes[source],before[source],segment['scale'],False)
            new = oracle.independent_matches([dict(nodes[target],root=after[target])],nodes[source],after[source],segment['scale'],False)
            item = oracle.transition(old,new,before[target],before[source],after[target],after[source])
            require(index < len(actual['edges']),'complete endpoint edges')
            compare(actual['edges'][index],dict(item,target=target,source=source));index += 1;expected.append(item)
    require(len(actual['edges']) == index,'no extra endpoint edges')
    wanted = oracle.aggregate(expected,oracle.prior_value(prior,nodes,before,audit),oracle.prior_value(prior,nodes,after,audit))
    compare(actual['totals'],wanted)
    # The system is reconstructed again here rather than trusting report deltas.
    system = audit.independent_equation(nodes,after,segment['scale'],prior)
    reason,delta,_ = reference.failure(system,len(nodes)-1,audit)
    for key,part in [('max_translation_proposal_m',slice(0,3)),('max_rotation_proposal_rad',slice(3,6))]:
        if reason is not None:require(actual[key] is None,'guard has no admissible proposal')
        else:np.testing.assert_allclose(actual[key],np.sqrt(np.einsum('ij,ij->i',delta[:,part],delta[:,part])).max(),atol=1e-10,rtol=1e-7)
    require(set(actual) == {'edges','totals','max_translation_proposal_m','max_rotation_proposal_rad'},'complete endpoint diagnostic schema')
    return index


def verify(report,nodes,initial,prior,audit):
    shape(report,initial);systems = transitions = 0;poses = np.asarray(initial)
    for segment,endpoint in zip(report['segments'],report['endpoints'],strict=True):
        systems += reference.verify(segment,nodes,poses,prior,audit)
        transitions += verify_endpoint(endpoint,segment,nodes,prior,audit)
        poses = np.asarray(segment['final_poses'])
    return dict(reported_systems=systems,additional_endpoint_system_checks=len(report['segments']),pair_transitions=transitions)


def controls():
    reference.source_proof()
    audit = load('budget_rendered_raw','check-reciprocal-keyframes.py').auditor()
    identity = np.eye(4);camera = audit.rays.CAMERA;depth = audit.rays.render(identity)
    nodes = [dict(timestamp=t,root=identity.copy(),surface=kernel.local.base.pyramid(depth,camera)) for t in (1.,1.03)]
    independent = [dict(n,surface=audit.pyramid(depth,camera)) for n in nodes];poses = np.array([identity,identity])
    same = kernel.refine(nodes,poses,None);counts = [verify(same,independent,poses,None,audit)]
    require(same['fresh_converged'] and len(same['segments']) == 1 and same['applied_iterations'] == 0,'identity never extends')
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.local.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization')
    initial = np.asarray(warm['revised_roots']);initial[1] = audit.increment(initial[1],np.array([.02,-.01,.005,.012,-.014,.009]))
    nodes[1]['surface'] = kernel.local.base.pyramid(source,camera);independent[1]['surface'] = audit.pyramid(source,camera)
    moved = kernel.refine(nodes,initial,None);counts.append(verify(moved,independent,initial,None,audit))
    require(moved['fresh_converged'] and moved['applied_iterations'] == 3 and len(moved['segments']) == 1,'rendered convergence never extends')
    for name,z in [('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))]:
        raw = [dict(timestamp=t,root=identity.copy(),surface=kernel.local.base.pyramid(z,camera)) for t in (1.,1.03)]
        check = [dict(n,surface=audit.pyramid(z,camera)) for n in raw]
        result = kernel.refine(raw,poses,None);counts.append(verify(result,check,poses,None,audit))
        require(result['termination'] == 'guard' and len(result['segments']) == 1 and result['applied_iterations'] == 0,name+' never extends')
    rejected = []
    mutations = [('wrong extension cap',lambda r:r.update(total_cap=31)),
        ('wrong segment cap',lambda r:r.update(segment_cap=11)),
        ('wrong total corrections',lambda r:r.update(applied_iterations=4)),
        ('stale first anchor',lambda r:r['segments'][0].update(initial_poses=poses.tolist())),
        ('false global convergence',lambda r:r.update(fresh_converged=False)),
        ('wrong fresh proposal metric',lambda r:r['endpoints'][0].update(max_translation_proposal_m=.02)),
        ('wrong turnover count',lambda r:r['endpoints'][0]['edges'][0].update(stable_pairs=-1)),
        ('wrong cost partition',lambda r:r['endpoints'][0]['totals'].update(association_delta=1.))]
    for name,change in mutations:
        bad = copy.deepcopy(moved);change(bad)
        try:verify(bad,independent,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected extended refinement corruption: '+name)
    bad = copy.deepcopy(same);bad['segments'] *= 2;bad['endpoints'] *= 2
    try:shape(bad,poses)
    except (ValueError,AssertionError):rejected.append('segment after convergence')
    else:raise ValueError('continued after rendered convergence')
    # Isolated outer-scheduler fixtures: no image-derived matrix evidence.
    saved_refine,saved_endpoint = kernel.local.refine,kernel.endpoint
    calls = []
    def fixture(nodes,initial,prior,scale):
        calls.append(np.asarray(initial).copy());after = np.asarray(initial).copy();after[1,0,3] += .001
        return dict(total_cap=10,inner_cap=3,scale=scale,initial_poses=np.asarray(initial).tolist(),
            applied_iterations=10,association_rounds=4,fresh_converged=False,guard_failure=None,termination='budget',
            final_poses=after.tolist(),final_fresh_equation={},final_fresh_correction={},final_fresh_cost=0.)
    try:
        kernel.local.refine = fixture;kernel.endpoint = lambda *args:{}
        worst = kernel.refine(nodes,poses,None);shape(worst,poses)
        require(len(calls) == 3 and worst['applied_iterations'] == 30,'three bounded diagnostic segments')
        stale = copy.deepcopy(worst);stale['segments'][1]['initial_poses'] = poses.tolist()
        try:shape(stale,poses)
        except (ValueError,AssertionError):rejected.append('stale later segment anchor')
        else:raise ValueError('stale later segment anchor allowed')
        bad = copy.deepcopy(worst);bad['segments'].pop();bad['endpoints'].pop()
        last = bad['segments'][-1]
        for key in ('fresh_converged','guard_failure','termination','final_poses','final_fresh_equation','final_fresh_correction','final_fresh_cost'):bad[key] = last[key]
        bad['applied_iterations'] = 20;bad['association_rounds'] = 8
        try:shape(bad,poses)
        except (ValueError,AssertionError):rejected.append('coherent unexplained early stop')
        else:raise ValueError('early budget stop allowed')
        for reason in ('converged','guard'):
            calls.clear()
            def stopping(nodes,initial,prior,scale):
                r = fixture(nodes,initial,prior,scale)
                if len(calls) == 2:r.update(applied_iterations=1,fresh_converged=reason == 'converged',guard_failure='injected' if reason == 'guard' else None,termination=reason)
                return r
            kernel.local.refine = stopping;result = kernel.refine(nodes,poses,None);shape(result,poses)
            require(len(calls) == 2 and result['applied_iterations'] == 11,'stop extension at '+reason)
    finally:kernel.local.refine,kernel.endpoint = saved_refine,saved_endpoint
    for node in nodes:np.testing.assert_array_equal(node['root'],identity)
    return dict(analytic_pass=True,inherited_source_proof=True,
        independently_checked_reported_systems=sum(c['reported_systems'] for c in counts),
        additional_endpoint_system_checks=sum(c['additional_endpoint_system_checks'] for c in counts),
        independently_checked_pair_transitions=sum(c['pair_transitions'] for c in counts),
        rendered_fresh_converged=True,rendered_applied_corrections=3,rendered_segments=1,
        isolated_worst_case_applied_corrections=30,isolated_later_guard_and_convergence_stop=True,
        fixtures_are_not_image_evidence=True,rejected_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in
        ('budget_depth_refinement.py','check-budget-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
