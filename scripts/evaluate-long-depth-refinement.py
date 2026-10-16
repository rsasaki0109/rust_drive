#!/usr/bin/env python3
"""One fixed-source solve through thirty corrections with exact 10/20/30 endpoints."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import long_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)
CAPS = (10,20,30)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('long_recorded_common','evaluate-alternating-depth-refinement.py')


def statistics(rows):
    checkpoints = {}
    for index,cap in enumerate(CAPS):
        values = [r['checkpoints'][index] for r in rows]
        checkpoints[str(cap)] = dict(samples=len(values),continuous_converged=sum(v['continuous_converged'] for v in values),
            original_discrete_fresh_converged=sum(v['final_discrete_correction']['converged'] for v in values),
            local_guard_failures=sum(v['guard_failure'] is not None for v in values),
            original_discrete_guard_failures=sum(v['final_discrete_correction']['guard_failure'] is not None for v in values),
            budget_exhaustions=sum(v['termination'] == 'budget' for v in values),
            applied_corrections=sum(v['actual_applied_corrections'] for v in values),
            terminal_carried_forward=sum(v['terminal_carried_forward'] for v in values),
            original_discrete_rms_gate_passes=sum(v['final_discrete_equation']['rms_m'] <= .03 for v in values),
            median_original_discrete_rms_m=float(np.median([v['final_discrete_equation']['rms_m'] for v in values])))
    reports = [r['refinement'] for r in rows]
    trials = [t for r in reports for s in r['trace'] for t in s['line_search']['trials']]
    trials += [t for r in reports if r['failed_search'] is not None for t in r['failed_search']['trials']]
    final,control = checkpoints['30'],checkpoints['10']
    eligible = (final['continuous_converged'] == len(rows)
        and final['original_discrete_fresh_converged'] > control['original_discrete_fresh_converged']
        and final['local_guard_failures'] == final['original_discrete_guard_failures'] == 0
        and final['original_discrete_rms_gate_passes'] == len(rows)
        and final['median_original_discrete_rms_m'] <= control['median_original_discrete_rms_m'])
    return dict(checkpoints=checkpoints,eligible_for_followup_full_trajectory=eligible,
        exact_ten_correction_control_matches=sum(r['complete_ten_correction_control_equal'] for r in rows),
        total_applied_corrections=sum(r['applied_iterations'] for r in reports),
        shortened_steps=sum(s['line_search']['accepted_alpha'] < 1 for r in reports for s in r['trace']),
        invalid_candidate_trials=sum(t['guard_failure'] is not None for t in trials),
        valid_nonpassing_trials=sum(t['guard_failure'] is None and t['cost'] > t['limit'] for t in trials))


def recorded(case,source):
    saved = source['recorded'][case]
    require([s['source_index'] for s in saved['samples']] == list(SAMPLES),'same ten viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body)
    require(sha(body) == saved['input_freeze_sha256'],'same original input freeze')
    inventory = json.loads((parent/'files.json').read_text())
    require(saved['sensor_ledger_sha256'] == inventory[prefix+'sensor-only.json.gz']['sha256'],'same original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'],'same original image manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']}
    suffix = case.replace('-','_');audit = load('long_raw_'+suffix,'check-reciprocal-keyframes.py').auditor()
    verifier = load('long_verifier_'+suffix,'check-long-depth-refinement.py')
    control_verifier = load('long_control_'+suffix,'check-sampling-depth-refinement.py')
    surfaces = {};rows = [];systems = controls = endpoints = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original image')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original depth digest')
            image = Image.open(io.BytesIO(raw));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = kernel.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera'])
        return surfaces[stamp]
    for sample in saved['samples']:
        control = sample['refinement'];initial = np.asarray(control['initial_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        independent = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        controls += control_verifier.verify(control,independent,initial,prior,audit)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,independent,initial,prior,audit)
        require(kernel.snapshot(report,nodes,op_prior,10) == control,'complete ten-correction report matches frozen control')
        checkpoints = [kernel.checkpoint(report,nodes,op_prior,cap) for cap in CAPS]
        for cap,endpoint in zip(CAPS,checkpoints,strict=True):endpoints += verifier.verify_checkpoint(endpoint,report,independent,initial,prior,audit,cap)
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_refinement=control,complete_ten_correction_control_equal=True,refinement=report,checkpoints=checkpoints))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'checkpoints',[(v['requested_cap'],v['actual_applied_corrections'],v['termination']) for v in checkpoints],
            'continuous/discrete',report['continuous_converged'],report['final_discrete_correction']['converged'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_long_systems=systems,independently_rechecked_control_systems=controls,
        independently_checked_checkpoint_systems=endpoints,statistics=statistics(rows),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/sampling-depth-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free local input')
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda c:recorded(c,source),CASES),strict=True))
    names = set(json.loads((ROOT/'assets/sampling-depth-refinement-v1/source-freeze.json').read_text()))
    names |= {'scripts/'+n for n in ('long_depth_refinement.py','check-long-depth-refinement.py','evaluate-long-depth-refinement.py')}
    names.add('assets/long-depth-refinement-v1/design.json')
    write(out/'summary.json',dict(data_role='already viewed development; fixed-source budget comparison only',sample_indices=list(SAMPLES),
        checkpoint_caps=list(CAPS),input_sampling_sha256=sha(body),real_local_pose_fitting=True,new_trajectory_fitting=False,
        numeric_ground_truth_parsed=False,physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        eligible_for_followup_full_trajectory=all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values()),
        recorded=reports,source_sha256={n:sha((ROOT/n).read_bytes()) for n in sorted(names)}))


if __name__ == '__main__':main()
