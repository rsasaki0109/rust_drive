#!/usr/bin/env python3
"""Evaluate conditional damped directions with the same ten fixed objectives."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import damped_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)
CAPS = (10,20,30)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('damped_recorded_inputs','evaluate-alternating-depth-refinement.py')


def statistics(rows,controls):
    reports = [r['refinement'] for r in rows];old = [r['refinement'] for r in controls]
    receipts = [s['line_search'] for q in reports for s in q['trace']]
    all_receipts = receipts+[q['failed_search'] for q in reports if q['failed_search'] is not None]
    directions = [t for s in all_receipts for t in s['damped_trials']]
    gn = [t for s in all_receipts for t in s['gauss_newton']['trials']]
    count = sum(q['continuous_converged'] for q in reports);fresh = sum(q['final_discrete_correction']['converged'] for q in reports)
    guards = sum(q['guard_failure'] is not None for q in reports)
    fresh_guards = sum(q['final_discrete_correction']['guard_failure'] is not None for q in reports)
    rms = sum(q['final_discrete_equation']['rms_m'] <= .03 for q in reports)
    median = float(np.median([q['final_discrete_equation']['rms_m'] for q in reports]))
    control_median = float(np.median([q['final_discrete_equation']['rms_m'] for q in old]))
    eligible = count == len(rows) and fresh > sum(q['final_discrete_correction']['converged'] for q in old)
    eligible = eligible and guards == fresh_guards == 0 and rms == len(rows) and median <= control_median
    checkpoints = {str(cap):dict(continuous_converged=sum(r['checkpoints'][i]['continuous_converged'] for r in rows),
        budget_exhaustions=sum(r['checkpoints'][i]['termination'] == 'budget' for r in rows)) for i,cap in enumerate(CAPS)}
    return dict(samples=len(rows),continuous_converged=count,original_discrete_fresh_converged=fresh,
        control_continuous_converged=sum(q['continuous_converged'] for q in old),local_guard_failures=guards,
        original_discrete_guard_failures=fresh_guards,original_discrete_rms_gate_passes=rms,
        median_original_discrete_rms_m=median,control_median_original_discrete_rms_m=control_median,
        budget_exhaustions=sum(q['termination'] == 'budget' for q in reports),
        total_applied_corrections=sum(q['applied_iterations'] for q in reports),
        unchanged_complete_control_reports=sum(r['complete_control_report_equal'] for r in rows),
        exact_control_prefix_corrections=sum(r['exact_control_prefix_corrections'] for r in rows),
        triggered_searches=sum(s['triggered'] for s in all_receipts),damped_solve_checks=sum(t['delta'] is not None for t in directions),
        selected_damped_directions=sum(type(s['selected']) is int for s in receipts),
        selected_gauss_newton_directions=sum(s['selected'] == 'gauss_newton' for s in receipts),
        original_gn_candidate_trials=len(gn),original_gn_invalid_candidates=sum(t['guard_failure'] is not None for t in gn),
        original_gn_valid_nonpassing_candidates=sum(t['guard_failure'] is None and t['cost'] > t['limit'] for t in gn),
        original_gn_passing_candidates=sum(t['guard_failure'] is None and t['cost'] <= t['limit'] for t in gn),
        damped_candidate_trials=len(directions),damped_invalid_candidates=sum(t['guard_failure'] is not None for t in directions),
        accepted_cost_increases=sum(s['accepted_cost'] > s['gauss_newton']['before_cost'] for s in receipts),
        checkpoints=checkpoints,eligible_for_followup_full_trajectory=bool(eligible))


def recorded(case,source):
    saved = source['recorded'][case];require([r['source_index'] for r in saved['samples']] == list(SAMPLES),'same ten viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body)
    require(sha(body) == saved['input_freeze_sha256'],'same original freeze')
    inventory = json.loads((parent/'files.json').read_text())
    require(saved['sensor_ledger_sha256'] == inventory[prefix+'sensor-only.json.gz']['sha256'],'same original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body) == freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']}
    suffix = case.replace('-','_');audit = load('damped_raw_'+suffix,'check-reciprocal-keyframes.py').auditor()
    verifier = load('damped_verifier_'+suffix,'check-damped-depth-refinement.py')
    old_verifier = load('damped_control_'+suffix,'check-deep-search-depth-refinement.py')
    surfaces = {};rows = [];systems = controls = endpoints = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original PNG')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original PNG digest')
            image = Image.open(io.BytesIO(raw));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = kernel.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera'])
        return surfaces[stamp]
    for sample in saved['samples']:
        control = sample['refinement'];initial = np.asarray(control['initial_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        controls += old_verifier.verify(control,other,initial,prior,audit)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,other,initial,prior,audit)
        require(report['initial_poses'] == control['initial_poses'] and report['frozen_population'] == control['frozen_population']
            and report['initial_cost'] == control['initial_cost'],'same original objective and normalizers')
        prefix_count = 0
        for step,old in zip(report['trace'],control['trace']):
            if type(step['line_search']['selected']) is int:break
            require(dict(step,line_search=step['line_search']['gauss_newton']) == old,'exact old state/matrix/pose prefix before selected alternative')
            prefix_count += 1
        changed = any(type(s['line_search']['selected']) is int for s in report['trace'])
        equal = not changed and verifier.normalized(report) == control
        if not changed:require(equal,'complete old terminal report when no alternative selected')
        checkpoints = [kernel.checkpoint(report,nodes,op_prior,cap) for cap in CAPS]
        for cap,cp in zip(CAPS,checkpoints,strict=True):endpoints += verifier.verify_checkpoint(cp,report,other,initial,prior,audit,cap)
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_report_sha256=sha(json.dumps(control,sort_keys=True).encode()),exact_control_prefix_corrections=prefix_count,
            complete_control_report_equal=equal,refinement=report,checkpoints=checkpoints))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'updates',report['applied_iterations'],'continuous/discrete',report['continuous_converged'],
            report['final_discrete_correction']['converged'],'stop',report['termination'],report['guard_failure'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_systems=systems,independently_rechecked_control_systems=controls,
        independently_checked_checkpoint_systems=endpoints,statistics=statistics(rows,saved['samples']),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/deep-search-depth-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free local input')
    freeze = json.loads((ROOT/'assets/damped-depth-refinement-v1/source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes()) == digest,'frozen source '+name)
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda c:recorded(c,source),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes()) == digest,'unchanged source '+name)
    write(out/'summary.json',dict(data_role='already viewed development; conditional damped-direction comparison',sample_indices=list(SAMPLES),
        checkpoint_caps=list(CAPS),input_deep_search_sha256=sha(body),real_local_pose_fitting=True,new_trajectory_fitting=False,
        numeric_ground_truth_parsed=False,physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        eligible_for_followup_full_trajectory=all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values()),
        recorded=reports,source_sha256=freeze))


if __name__ == '__main__':main()
