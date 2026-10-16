#!/usr/bin/env python3
"""Compare three bounded continuous reselection rounds on ten viewed windows."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import reselected_slope_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('reselected_recorded_inputs','evaluate-alternating-depth-refinement.py')


def statistics(rows,controls):
    reports = [r['refinement'] for r in rows];old = [r['refinement'] for r in controls]
    batches = [r['batch'] for q in reports for r in q['rounds']]
    receipts = [t['line_search'] for b in batches for t in b['trace']]
    searches = receipts+[b['failed_search'] for b in batches if b['failed_search'] is not None]
    trials = [t for search in searches for t in search['trials']]
    median = float(np.median([q['final_original_discrete_equation']['rms_m'] for q in reports]))
    control_median = float(np.median([q['final_discrete_equation']['rms_m'] for q in old]))
    selection_converged = sum(q['selection_converged'] for q in reports)
    discrete_converged = sum(q['final_original_discrete_correction']['converged'] for q in reports)
    guards = sum(q['guard_failure'] is not None for q in reports)
    fresh_guards = sum(q['final_original_discrete_correction']['guard_failure'] is not None for q in reports)
    rms_passes = sum(q['final_original_discrete_equation']['rms_m']<=.03 for q in reports)
    transitions = [t for row in rows for t in row['initial_to_terminal_reselection']]
    old_count = sum(t['old_count'] for t in transitions);new_count = sum(t['new_count'] for t in transitions)
    shared = sum(t['shared_count'] for t in transitions)
    eligible = selection_converged==len(rows) and discrete_converged>sum(q['final_discrete_correction']['converged'] for q in old)
    eligible = eligible and guards==fresh_guards==0 and rms_passes==len(rows) and median<=control_median
    return dict(samples=len(rows),selection_converged=selection_converged,final_fixed_converged=sum(q['final_fixed_converged'] for q in reports),
        terminal_same_model_fresh_converged=sum(q['final_reselected_correction']['converged'] for q in reports),
        original_discrete_fresh_converged=discrete_converged,control_fixed_converged=sum(q['continuous_converged'] for q in old),
        control_original_discrete_fresh_converged=sum(q['final_discrete_correction']['converged'] for q in old),
        total_association_rounds=len(batches),total_applied_corrections=sum(q['applied_iterations'] for q in reports),
        inner_guard_stops=sum(q['termination']=='inner_guard' for q in reports),selection_guard_stops=sum(q['termination']=='selection_guard' for q in reports),
        inner_budget_stops=sum(q['termination']=='inner_budget' for q in reports),round_budget_stops=sum(q['termination']=='round_budget' for q in reports),
        original_discrete_guard_failures=fresh_guards,original_discrete_rms_gate_passes=rms_passes,
        median_original_discrete_rms_m=median,control_median_original_discrete_rms_m=control_median,
        initial_source_occurrences=old_count,terminal_reselected_source_occurrences=new_count,retained_initial_source_occurrences=shared,
        removed_initial_source_occurrences=old_count-shared,added_terminal_source_occurrences=new_count-shared,
        initial_source_retention=None if not old_count else shared/old_count,
        within_round_invalid_trials=sum(t['guard_failure'] is not None for t in trials),
        within_round_valid_nonpassing_trials=sum(t['guard_failure'] is None and t['cost']>t['limit'] for t in trials),
        within_round_accepted_trials=len(receipts),within_round_shortened_steps=sum(s['accepted_alpha']<1 for s in receipts),
        within_round_cost_increases=sum(s['trials'][-1]['cost']>s['before_cost'] for s in receipts),
        between_round_population_cost_increases=sum(r['reselected']['cost']>r['batch']['final_cost'] for q in reports for r in q['rounds']),
        eligible_for_followup_full_trajectory=bool(eligible))


def recorded(case,source):
    saved = source['recorded'][case];require([s['source_index'] for s in saved['samples']]==list(SAMPLES),'same ten viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body);require(sha(body)==saved['input_freeze_sha256'],'original input freeze')
    inventory = json.loads((parent/'files.json').read_text());require(saved['sensor_ledger_sha256']==inventory[prefix+'sensor-only.json.gz']['sha256'],'original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body)==freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']};suffix = case.replace('-','_')
    audit = load('reselected_raw_'+suffix,'check-reciprocal-keyframes.py').auditor()
    verifier = load('reselected_verify_'+suffix,'check-reselected-slope-refinement.py');surfaces = {};rows = [];systems = controls = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];receipt = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0<path.stat().st_size<=4*1024*1024,'bounded original PNG')
            body = path.read_bytes();require(sha(body)==receipt['sha256'] and len(body)==receipt['bytes'],'original PNG digest')
            image = Image.open(io.BytesIO(body));require(image.size==(640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth image')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.fixed.prepare([dict(surface=kernel.fixed.base.pyramid(depth,freeze['camera']))])[0]['surface'],
                verifier.reference.prepare([dict(surface=audit.pyramid(depth,freeze['camera']))])[0]['surface'])
        return surfaces[stamp]
    for sample in saved['samples']:
        control = sample['refinement'];initial = np.asarray(control['initial_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
        prior_before = None if op_prior is None else json.dumps(op_prior.encode(),sort_keys=True)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        slopes = verifier.reference.compare_prepared(nodes,other)
        controls += verifier.reference.verify(control,other,initial,prior,audit)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,other,initial,prior,audit)
        require(report['rounds'][0]['batch']==dict(control,cap=30),'first full round exactly reproduces the prior terminal fit within30')
        for previous,next_round in zip(report['rounds'],report['rounds'][1:]):
            require(previous['reselected']['population']==next_round['batch']['frozen_population'],'reselection becomes the next fixed population without another state change')
        terminal = np.asarray(report['final_poses'])
        selection = kernel.transition(kernel.fixed.population(nodes,initial,1),kernel.fixed.population(nodes,terminal,1))
        expected = verifier.transition(verifier.reference.population(other,initial,1),verifier.reference.population(other,terminal,1))
        verifier.compare_transition(selection,expected)
        require(prior_before==(None if op_prior is None else json.dumps(op_prior.encode(),sort_keys=True)),'incoming nonlinear prior remains unchanged across all rounds')
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_report_sha256=sha(json.dumps(control,sort_keys=True).encode()),first_round_matches_control_except_cap=True,
            depth_slope_fields=slopes,initial_to_terminal_reselection=selection,incoming_prior_unchanged=True,refinement=report))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'rounds',report['association_rounds'],'updates',report['applied_iterations'],
            'selection/discrete',report['selection_converged'],report['final_original_discrete_correction']['converged'],
            'stop',report['termination'],report['guard_failure'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_systems=systems,independently_rechecked_control_systems=controls,statistics=statistics(rows,saved['samples']),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/slope-admission-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free original local input')
    freeze = json.loads((ROOT/'assets/reselected-slope-refinement-v1/source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'frozen source '+name)
    analytic = load('reselected_recorded_analytic','check-reselected-slope-refinement.py').controls()
    analytic['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('reselected_slope_refinement.py','check-reselected-slope-refinement.py')}
    write(out/'analytic.json',analytic)
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda case:recorded(case,source),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'unchanged source '+name)
    numerical_progression = all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values())
    unresolved = sum(not r['resolved'] for r in analytic['unresolved_identity_boundaries'].values())
    write(out/'summary.json',dict(data_role='already viewed development; three bounded continuous reselection rounds',sample_indices=list(SAMPLES),
        max_rounds=3,inner_cap=30,max_corrections=90,input_slope_admission_sha256=sha(body),real_local_pose_fitting=True,
        new_trajectory_fitting=False,numeric_ground_truth_parsed=False,physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        numerical_progression_rule_pass=numerical_progression,unresolved_analytic_domain_cases=unresolved,
        eligible_for_followup_full_trajectory=numerical_progression and unresolved==0,recorded=reports,source_sha256=freeze))


if __name__=='__main__':main()
