#!/usr/bin/env python3
"""Compare twenty-one versus eight search candidates on ten frozen local fits."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import deep_search_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)
CAPS = (10,20,30)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('deep_search_recorded_inputs','evaluate-alternating-depth-refinement.py')


def statistics(rows,controls):
    reports = [r['refinement'] for r in rows];before = [r['refinement'] for r in controls]
    accepted = [s['line_search'] for q in reports for s in q['trace']]
    trials = [t for s in accepted for t in s['trials']]
    trials += [t for q in reports if q['failed_search'] is not None for t in q['failed_search']['trials']]
    old_median = float(np.median([q['final_discrete_equation']['rms_m'] for q in before]))
    new_median = float(np.median([q['final_discrete_equation']['rms_m'] for q in reports]))
    continuous = sum(q['continuous_converged'] for q in reports)
    fresh = sum(q['final_discrete_correction']['converged'] for q in reports)
    guards = sum(q['guard_failure'] is not None for q in reports)
    fresh_guards = sum(q['final_discrete_correction']['guard_failure'] is not None for q in reports)
    rms = sum(q['final_discrete_equation']['rms_m'] <= .03 for q in reports)
    eligible = continuous == len(rows) and fresh > sum(q['final_discrete_correction']['converged'] for q in before)
    eligible = eligible and guards == fresh_guards == 0 and rms == len(rows) and new_median <= old_median
    checkpoints = {}
    for index,cap in enumerate(CAPS):
        values = [row['checkpoints'][index] for row in rows]
        checkpoints[str(cap)] = dict(continuous_converged=sum(q['continuous_converged'] for q in values),
            original_discrete_fresh_converged=sum(q['final_discrete_correction']['converged'] for q in values),
            local_guards=sum(q['guard_failure'] is not None for q in values),budget_exhaustions=sum(q['termination'] == 'budget' for q in values),
            total_applied_corrections=sum(q['actual_applied_corrections'] for q in values),
            terminal_carried_forward=sum(q['terminal_carried_forward'] for q in values))
    return dict(samples=len(rows),continuous_converged=continuous,original_discrete_fresh_converged=fresh,
        control_continuous_converged=sum(q['continuous_converged'] for q in before),local_guard_failures=guards,
        original_discrete_guard_failures=fresh_guards,original_discrete_rms_gate_passes=rms,
        median_original_discrete_rms_m=new_median,control_median_original_discrete_rms_m=old_median,
        budget_exhaustions=sum(q['termination'] == 'budget' for q in reports),
        total_applied_corrections=sum(q['applied_iterations'] for q in reports),
        old_control_corrections=sum(q['applied_iterations'] for q in before),
        complete_control_trace_prefixes=sum(r['complete_control_trace_prefix_equal'] for r in rows),
        unchanged_nonsearch_terminal_reports=sum(r['unchanged_nonsearch_terminal_report'] is True for r in rows),
        continued_old_search_failures=sum(r['continued_old_search_failure'] for r in rows),
        accepted_steps_below_old_search_bound=sum(s['accepted_alpha'] < 1/128 for s in accepted),
        shortened_steps=sum(s['accepted_alpha'] < 1 for s in accepted),
        accepted_cost_increases=sum(s['trials'][-1]['cost'] > s['before_cost'] for s in accepted),
        accepted_steps_without_strict_cost_decrease=sum(s['trials'][-1]['cost'] >= s['before_cost'] for s in accepted),
        accepted_steps_requiring_existing_slack=sum(s['trials'][-1]['cost'] > s['before_cost']+.0001*s['accepted_alpha']*s['directional_derivative'] for s in accepted),
        invalid_candidate_trials=sum(t['guard_failure'] is not None for t in trials),
        valid_nonpassing_candidate_trials=sum(t['guard_failure'] is None and t['cost'] > t['limit'] for t in trials),
        valid_passing_candidate_trials=sum(t['guard_failure'] is None and t['cost'] <= t['limit'] for t in trials),
        checkpoints=checkpoints,eligible_for_followup_full_trajectory=bool(eligible))


def recorded(case,source,predictions):
    saved = source['recorded'][case]
    require([s['source_index'] for s in saved['samples']] == list(SAMPLES),'same ten original viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body)
    require(sha(body) == saved['input_freeze_sha256'],'same original freeze')
    inventory = json.loads((parent/'files.json').read_text())
    require(saved['sensor_ledger_sha256'] == inventory[prefix+'sensor-only.json.gz']['sha256'],'same sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'],'same image manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']}
    suffix = case.replace('-','_');audit = load('deep_raw_'+suffix,'check-reciprocal-keyframes.py').auditor()
    verifier = load('deep_verifier_'+suffix,'check-deep-search-depth-refinement.py')
    old_verifier = load('deep_old_control_'+suffix,'check-long-depth-refinement.py')
    surfaces = {};rows = [];systems = old_systems = endpoints = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original depth PNG')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original depth hash')
            image = Image.open(io.BytesIO(raw));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = kernel.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera'])
        return surfaces[stamp]
    for sample in saved['samples']:
        control = sample['refinement'];initial = np.asarray(control['initial_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        old_systems += old_verifier.verify(control,other,initial,prior,audit)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,other,initial,prior,audit)
        require(report['initial_poses'] == control['initial_poses'] and report['frozen_population'] == control['frozen_population']
            and report['initial_cost'] == control['initial_cost'],'same complete initial objective')
        used = len(control['trace']);require(report['trace'][:used] == control['trace'],'complete old trace prefix equality')
        continued = control['guard_failure'] == 'line search'
        if continued:
            require(len(report['trace']) > used,'deeper search continues original failed search')
            step = report['trace'][used];receipt = step['line_search'];old = control['failed_search']
            require(step['before'] == control['final_poses'] and step['equation'] == control['final_equation']
                and step['delta'] == control['final_correction']['delta'],'identical state and full proposal at old failure')
            require(receipt['trials'][:8] == old['trials'] and receipt['before_cost'] == old['before_cost']
                and receipt['directional_derivative'] == old['directional_derivative'],'complete failed eight-candidate prefix')
            require(case == 'tum-fr1-room-temporal' and receipt['accepted_alpha'] == predictions[sample['source_index']],
                'first continuation matches frozen signed-probe prediction')
        else:require(report == control,'complete unchanged non-search terminal report')
        checkpoints = [kernel.checkpoint(report,nodes,op_prior,cap) for cap in CAPS]
        for cap,cp in zip(CAPS,checkpoints,strict=True):endpoints += verifier.verify_checkpoint(cp,report,other,initial,prior,audit,cap)
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_report_sha256=sha(json.dumps(control,sort_keys=True).encode()),complete_control_trace_prefix_equal=True,
            control_trace_length=used,continued_old_search_failure=continued,
            unchanged_nonsearch_terminal_report=None if continued else True,refinement=report,checkpoints=checkpoints))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'updates',report['applied_iterations'],'continuous/discrete',report['continuous_converged'],
            report['final_discrete_correction']['converged'],'stop',report['termination'],report['guard_failure'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_deep_systems=systems,independently_rechecked_control_systems=old_systems,
        independently_checked_checkpoint_systems=endpoints,statistics=statistics(rows,saved['samples']),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/long-depth-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free local input')
    probes = common.archived(ROOT/'assets/depth-search-diagnostic-v1','statistics.json')
    predictions = {r['source_index']:r['first_positive_armijo_alpha'] for r in json.loads(probes)['samples']}
    freeze = json.loads((ROOT/'assets/deep-search-depth-refinement-v1/source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes()) == digest,'frozen source '+name)
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda c:recorded(c,source,predictions),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes()) == digest,'unchanged source '+name)
    write(out/'summary.json',dict(data_role='already viewed development; bounded search-depth comparison',sample_indices=list(SAMPLES),
        checkpoint_caps=list(CAPS),candidate_multipliers_per_search=21,input_long_sha256=sha(body),input_probe_statistics_sha256=sha(probes),
        real_local_pose_fitting=True,new_trajectory_fitting=False,numeric_ground_truth_parsed=False,physical_accuracy_scored=False,
        tracker_state_commits=False,default_changed=False,
        eligible_for_followup_full_trajectory=all(r['statistics']['eligible_for_followup_full_trajectory'] for r in reports.values()),
        recorded=reports,source_sha256=freeze))


if __name__ == '__main__':main()
