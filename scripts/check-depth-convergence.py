#!/usr/bin/env python3
"""Independent reconstruction of bounded depth-fit continuation certificates."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

from projective_depth_common import ROOT, fresh, require, sha, write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def verify(report,nodes,saved,prior,audit):
    """Use raw SVD surfaces, spectral solves and the independent Rodrigues map."""
    guard = load('continuation_spectral_guards','check-selective-keyframes.py')
    trace,metrics = report['trace'],report['steps']
    require(len(trace) <= 10 and len(metrics) == len(trace)+1,'ten-step diagnostic budget and fresh final system')
    require(report['additional_applied_iterations'] == len(trace),'honest applied-iteration count')
    poses = np.asarray(saved).copy()
    np.testing.assert_array_equal(report['initial_poses'],poses)
    for i,metric in enumerate(metrics):
        e = audit.independent_equation(nodes,poses,1,prior)
        reason,d,condition = guard.failure(audit,e,len(nodes)-1)
        require(metric['iteration'] == i and metric['count'] == e['count'],'metric iteration and support')
        require(metric['guard_failure'] == reason,'independently reproduced continuation guard')
        np.testing.assert_allclose(metric['raw_rms_m'],e['rms_m'],atol=1e-10,rtol=0)
        if condition is None:require(metric['condition'] is None,'no fabricated diagnostic condition')
        else:np.testing.assert_allclose(metric['condition'],condition,atol=1e-6,rtol=1e-8)
        if d is None:require(metric['proposed_step'] is None,'no fabricated diagnostic correction')
        else:
            expected = [float(np.linalg.norm(d[:,:3],axis=1).max()),float(np.linalg.norm(d[:,3:],axis=1).max())]
            actual = metric['proposed_step']
            np.testing.assert_allclose([actual['max_translation_m'],actual['max_rotation_rad']],expected,atol=1e-9,rtol=1e-7)
        displacement = np.linalg.norm(poses[:,:3,3]-np.asarray(saved)[:,:3,3],axis=1).max()
        np.testing.assert_allclose(metric['max_node_translation_from_saved_m'],displacement,atol=1e-11,rtol=0)
        converged = reason is None and all(np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-5 for x in d)
        if i < len(trace):
            require(reason is None and not converged,'no update after failure or convergence')
            item = trace[i];require(item['scale'] == 1 and 'skip_reason' not in item,'finest-only diagnostic continuation')
            np.testing.assert_array_equal(item['before'],poses)
            poses = audit.audit_step(nodes,prior,item)
        else:
            audit.compare_equation(report['final_equation'],e)
            require(reason is not None or converged or i == 10,'no unexplained early termination')
            require(report['guard_failure'] == reason and report['converged'] == converged,'honest termination reason')
            require(report['final_raw_residual_gate'] == (e['rms_m'] <= .03),'fresh final residual gate')
    require(report['initial']['count'] == metrics[0]['count'],'initial diagnostic support')
    np.testing.assert_allclose(report['initial']['raw_rms_m'],metrics[0]['raw_rms_m'],atol=0,rtol=0)
    np.testing.assert_array_equal(report['final_poses'],poses)
    np.testing.assert_array_equal(poses[0],np.eye(4))
    return len(metrics)


def controls():
    diagnostic = load('continuation_under_test','diagnose-depth-convergence.py')
    audit = load('continuation_raw_audit','check-reciprocal-keyframes.py').auditor()
    camera = audit.rays.CAMERA;depth = audit.rays.render(np.eye(4))
    saved = np.array([np.eye(4),np.eye(4)])
    nodes = [dict(timestamp=t,root=p.copy(),surface=diagnostic.kernel.pyramid(depth,camera)) for t,p in zip((1.,1.03),saved)]
    independent = [dict(n,surface=audit.pyramid(depth,camera)) for n in nodes]
    identity = diagnostic.continue_fine(nodes,saved,None)
    checked = verify(identity,independent,saved,None,audit)
    require(identity['converged'] and not identity['trace'],'measured identity stops before an unnecessary correction')
    empty = [dict(n,surface=diagnostic.kernel.pyramid(np.zeros_like(depth),camera)) for n in nodes]
    empty_independent = [dict(n,surface=audit.pyramid(np.zeros_like(depth),camera)) for n in nodes]
    missing = diagnostic.continue_fine(empty,saved,None)
    checked += verify(missing,empty_independent,saved,None,audit)
    require(missing['guard_failure'] == 'new frame needs measured support' and not missing['trace'],'missing finest support stops immediately')
    # Injected algebra isolates budget/guard bookkeeping. It does not claim
    # these systems come from image measurements or verify raw correspondence.
    original_equation,original_independent = diagnostic.kernel.equation,audit.independent_equation
    original_step = audit.audit_step
    def system(nodes,poses,scale,prior,incident=None):
        return dict(h=np.eye(6),g=np.array([-.0002,0,0,0,0,0]),edges=[dict(target=0,source=1,count=200,rms_m=.002)],
            count=200,rms_m=.002,prior_delta=[])
    def algebra_step(nodes,prior,item):
        e = audit.independent_equation(nodes,np.asarray(item['before']),1,prior)
        audit.compare_equation(item['equation'],e)
        d,condition = audit.independent_solve(e['h'],e['g'])
        np.testing.assert_allclose(item['delta'],d,atol=1e-12,rtol=0)
        np.testing.assert_allclose(item['condition'],condition,atol=1e-12,rtol=0)
        result = np.asarray(item['before']).copy();result[1] = audit.increment(result[1],d[0])
        np.testing.assert_array_equal(item['after'],result)
        return result
    rejected = [];guards = {};budget = None
    try:
        diagnostic.kernel.equation = system;audit.independent_equation = system;audit.audit_step = algebra_step
        budget = diagnostic.continue_fine(nodes,saved,None);checked += verify(budget,nodes,saved,None,audit)
        require(len(budget['trace']) == 10 and not budget['converged'],'nonconvergent algebra stops exactly at the budget')
        def truncate(r):
            r['trace'].pop();r['steps'].pop();r['additional_applied_iterations'] = 9
            r['final_poses'] = r['trace'][-1]['after']
        changes = [
            ('extra iteration',lambda r:r['trace'].append(copy.deepcopy(r['trace'][-1]))),
            ('unjustified early stop',truncate),
            ('false convergence',lambda r:r.update(converged=True)),
            ('coarse masquerading as fine',lambda r:r['trace'][0].update(scale=2)),
            ('false residual metric',lambda r:r['steps'][-1].update(raw_rms_m=.02)),
            ('scaled final system',lambda r:r['final_equation'].update(h=(np.eye(6)*2).tolist())),
        ]
        for name,change in changes:
            bad = copy.deepcopy(budget);change(bad)
            try:verify(bad,nodes,saved,None,audit)
            except (ValueError,AssertionError):rejected.append(name)
            else:raise ValueError('undetected continuation corruption: '+name)
        for name in ('zero/nonfinite diagonal','condition','increment','late missing support','high final RMS'):
            def altered(nodes,poses,scale,prior,incident=None):
                e = system(nodes,poses,scale,prior)
                if name == 'zero/nonfinite diagonal':e['h'][0,0] = 0
                elif name == 'condition':e['h'][0,1] = e['h'][1,0] = .9999999
                elif name == 'increment':e['g'][0] = -.11
                elif name == 'late missing support' and poses[1,0,3] >= .0003:e['edges'] = [];e['count'] = 0
                elif name == 'high final RMS':e['rms_m'] = .04
                return e
            diagnostic.kernel.equation = altered;audit.independent_equation = altered
            result = diagnostic.continue_fine(nodes,saved,None);checked += verify(result,nodes,saved,None,audit)
            if name == 'late missing support':require(len(result['trace']) == 2 and result['guard_failure'] == 'new frame needs measured support','late failure leaves the failed correction unapplied')
            elif name == 'high final RMS':require(not result['final_raw_residual_gate'],'high final RMS stays a failed gate')
            else:require(not result['trace'] and result['guard_failure'] == name,'inadmissible first proposal is not applied')
            guards[name] = dict(applied_iterations=len(result['trace']),guard_failure=result['guard_failure'],final_raw_residual_gate=result['final_raw_residual_gate'])
    finally:
        diagnostic.kernel.equation = original_equation;audit.independent_equation = original_independent;audit.audit_step = original_step
    np.testing.assert_array_equal(saved,[np.eye(4),np.eye(4)])
    for n in nodes:np.testing.assert_array_equal(n['root'],np.eye(4))
    return dict(analytic_pass=True,independent_metric_systems=checked,measured_identity_converges_without_update=True,
        measured_missing_support_rejected=True,algebra_budget=10,algebra_guards=guards,rejected_corruptions=rejected,
        input_poses_and_roots_unchanged=True,raw_oracle_shared='frozen reciprocal pixel oracle and SVD normal reconstruction',
        injected_algebra_is_not_raw_image_evidence=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    names = ('check-depth-convergence.py','diagnose-depth-convergence.py')
    report['source_sha256'] = {'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in names}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
