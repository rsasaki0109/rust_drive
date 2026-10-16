#!/usr/bin/env python3
"""Independent fixed-population cap expansion and checkpoint event ordering."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import long_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


sampling = load('long_sampling_oracle','check-sampling-depth-refinement.py')
independent = sampling.independent
population = sampling.population
check_metadata,system,objective = independent.check_metadata,independent.system,independent.objective
search,compare_search,reference = independent.search,independent.compare_search,independent.reference


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
    require(type(cap) is int and cap in (10,20,30) and actual['requested_cap'] == cap,'fixed checkpoint limits')
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


def source_proof():
    def body(name,function):
        text = (ROOT/'scripts'/name).read_text()
        node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name == function)
        return ast.get_source_segment(text,node)
    expected = body('sampling_depth_refinement.py','refine').replace('def refine(nodes,initial,prior,scale=1):',
        "def refine(nodes,initial,prior,scale=1,cap=30):\n    require(type(cap) is int and cap in (10,20,30),'fixed diagnostic correction cap')")
    expected = expected.replace('range(11)','range(cap+1)').replace('iteration == 10','iteration == cap').replace('scale=scale,cap=10','scale=scale,cap=cap')
    require(body('long_depth_refinement.py','refine') == expected,'only bounded correction cap changes in solver')
    expected = body('check-continuous-depth-refinement.py','verify')
    expected = expected.replace("report['cap'] == 10","type(report['cap']) is int and report['cap'] in (10,20,30)")
    expected = expected.replace("len(report['trace']) <= 10","len(report['trace']) <= report['cap']")
    expected = expected.replace("len(report['trace']) == 10","len(report['trace']) == report['cap']")
    require(body('check-long-depth-refinement.py','verify') == expected,'only cap changes in independent fixed-objective verifier')
    return True


def controls():
    proof = source_proof();inherited = sampling.controls()
    audit = load('long_rendered_oracle','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA
    depth = audit.rays.render(identity);truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = [dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(nodes,(depth,source),strict=True)]
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    checkpoints = [kernel.checkpoint(report,nodes,None,cap) for cap in (10,20,30)]
    endpoint_checks = sum(verify_checkpoint(cp,report,other,initial,None,audit,cap) for cap,cp in zip((10,20,30),checkpoints,strict=True))
    old = sampling.kernel.refine(nodes,initial,None)
    require(kernel.snapshot(report,nodes,None,10) == old,'complete shorter report matches frozen ten-correction solver')
    rejected = []
    for name,change in [('wrong extended cap',lambda r:r.update(cap=31)),('false long convergence',lambda r:r.update(continuous_converged=False)),
        ('changed frozen normalizer',lambda r:r['frozen_population'][0].update(normalizer_sha256='0'*64))]:
        bad = copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected long-refinement corruption: '+name)
    for name,change in [('wrong checkpoint count',lambda r:r.update(actual_applied_corrections=10)),
        ('false carry-forward',lambda r:r.update(terminal_carried_forward=False)),
        ('false checkpoint budget',lambda r:r.update(termination='budget')),
        ('wrong checkpoint matrix',lambda r:r['final_equation'].update(g=(np.asarray(r['final_equation']['g'])+1).tolist()))]:
        bad = copy.deepcopy(checkpoints[0]);change(bad)
        try:verify_checkpoint(bad,report,other,initial,None,audit,10)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected checkpoint corruption: '+name)
    # Isolated event-order checks, not image-derived matrix evidence: an
    # exhausted next search at update10 cannot change a cap10 budget stop.
    admissible = dict(guard_failure=None,converged=False)
    require(kernel.stop(admissible,10,10,'line search') == ('budget',None),'cap precedes next search failure')
    require(kernel.stop(admissible,10,20,'line search') == ('guard','line search'),'same search precedes later cap')
    require(kernel.stop(dict(guard_failure='condition',converged=False),10,10,'condition') == ('guard','condition'),'proposal guard precedes cap')
    require(kernel.stop(dict(guard_failure=None,converged=True),10,10,None) == ('converged',None),'convergence precedes cap')
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    return dict(analytic_pass=True,source_proof=proof,inherited_sampling_controls=inherited,
        additional_rendered_reported_systems=checked,additional_checkpoint_systems=endpoint_checks,
        rendered_continuous_converged=report['continuous_converged'],rendered_applied_corrections=report['applied_iterations'],
        complete_ten_correction_control_parity=True,event_order_controls_are_not_image_evidence=True,
        cap_precedes_next_search_failure=True,proposal_guard_and_convergence_precede_cap=True,
        rejected_long_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('long_depth_refinement.py','check-long-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
