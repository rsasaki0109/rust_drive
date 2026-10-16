#!/usr/bin/env python3
"""Paired sampling-domain comparison with both frozen continuous predecessors."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import sampling_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('sampling_recorded_common','evaluate-alternating-depth-refinement.py')


def statistics(rows):
    reports = [r['refinement'] for r in rows];controls = [r['margin_control'] for r in rows];centers = [r['center_control'] for r in rows]
    trials = [t for r in reports for s in r['trace'] for t in s['line_search']['trials']]
    trials += [t for r in reports if r['failed_search'] is not None for t in r['failed_search']['trials']]
    result = dict(samples=len(rows),continuous_converged=sum(r['continuous_converged'] for r in reports),
        margin_control_converged=sum(r['continuous_converged'] for r in controls),center_control_converged=sum(r['continuous_converged'] for r in centers),
        original_discrete_fresh_converged=sum(r['final_discrete_correction']['converged'] for r in reports),
        control_discrete_fresh_converged=sum(r['final_discrete_correction']['converged'] for r in controls),
        local_guard_failures=sum(r['guard_failure'] is not None for r in reports),
        margin_control_guard_failures=sum(r['guard_failure'] is not None for r in controls),
        center_control_guard_failures=sum(r['guard_failure'] is not None for r in centers),
        original_discrete_guard_failures=sum(r['final_discrete_correction']['guard_failure'] is not None for r in reports),
        budget_exhaustions=sum(r['termination'] == 'budget' for r in reports),
        applied_corrections=sum(r['applied_iterations'] for r in reports),
        margin_control_corrections=sum(r['applied_iterations'] for r in controls),center_control_corrections=sum(r['applied_iterations'] for r in centers),
        invalid_candidate_trials=sum(t['guard_failure'] is not None for t in trials),
        valid_nonpassing_trials=sum(t['guard_failure'] is None and t['cost'] > t['limit'] for t in trials),
        shortened_steps=sum(s['line_search']['accepted_alpha'] < 1 for r in reports for s in r['trace']),
        initial_source_occurrences=sum(m['count'] for r in reports for m in r['frozen_population']),
        margin_control_source_occurrences=sum(m['count'] for r in controls for m in r['frozen_population']),
        center_control_source_occurrences=sum(m['count'] for r in centers for m in r['frozen_population']),
        initial_factor_occurrences=sum(len(r['frozen_population']) for r in reports),
        original_discrete_rms_gate_passes=sum(r['final_discrete_equation']['rms_m'] <= .03 for r in reports),
        median_original_discrete_rms_m=float(np.median([r['final_discrete_equation']['rms_m'] for r in reports])),
        median_control_discrete_rms_m=float(np.median([r['final_discrete_equation']['rms_m'] for r in controls])))
    result['eligible_for_followup_full_trajectory'] = (result['continuous_converged'] == len(rows)
        and result['original_discrete_fresh_converged'] > result['control_discrete_fresh_converged']
        and result['local_guard_failures'] == result['original_discrete_guard_failures'] == 0
        and result['original_discrete_rms_gate_passes'] == len(rows)
        and result['median_original_discrete_rms_m'] <= result['median_control_discrete_rms_m'])
    return result


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
    suffix = case.replace('-','_');audit = load('sampling_raw_'+suffix,'check-reciprocal-keyframes.py').auditor()
    verifier = load('sampling_verifier_'+suffix,'check-sampling-depth-refinement.py')
    margin_verifier = load('sampling_margin_control_'+suffix,'check-margin-depth-refinement.py')
    center_verifier = load('sampling_center_control_'+suffix,'check-continuous-depth-refinement.py')
    surfaces = {};rows = [];systems = margins = centers = 0
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
        margin,center = sample['refinement'],sample['control_refinement'];initial = np.asarray(margin['initial_poses']);prior = sample['prior_before']
        require(center['initial_poses'] == margin['initial_poses'],'all three methods have the same start')
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        independent = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        margins += margin_verifier.verify(margin,independent,initial,prior,audit)
        centers += center_verifier.verify(center,independent,initial,prior,audit)
        report = kernel.refine(nodes,initial,common.prior_object(prior));systems += verifier.verify(report,independent,initial,prior,audit)
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            margin_control=margin,center_control=center,refinement=report))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'continuous/discrete',report['continuous_converged'],report['final_discrete_correction']['converged'],
            'corrections',report['applied_iterations'],'termination',report['termination'],report['guard_failure'],
            'sources',sum(m['count'] for m in margin['frozen_population']),'->',sum(m['count'] for m in report['frozen_population']),flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_new_systems=systems,independently_rechecked_margin_systems=margins,independently_rechecked_center_systems=centers,
        statistics=statistics(rows),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/margin-depth-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free local input')
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda c:recorded(c,source),CASES),strict=True))
    names = set(json.loads((ROOT/'assets/margin-depth-refinement-v1/source-freeze.json').read_text()))
    names |= {'scripts/'+n for n in ('sampling_depth_refinement.py','check-sampling-depth-refinement.py','evaluate-sampling-depth-refinement.py')}
    names.add('assets/sampling-depth-refinement-v1/design.json')
    write(out/'summary.json',dict(data_role='already viewed development; sampling-domain local comparison only',sample_indices=list(SAMPLES),
        input_margin_sha256=sha(body),real_local_pose_fitting=True,new_trajectory_fitting=False,numeric_ground_truth_parsed=False,
        physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        eligible_for_followup_full_trajectory=all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values()),
        recorded=reports,source_sha256={n:sha((ROOT/n).read_bytes()) for n in sorted(names)}))


if __name__ == '__main__':main()
