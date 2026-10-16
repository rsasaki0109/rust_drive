#!/usr/bin/env python3
"""Diagnose ten saved local fits through 10/20/30 unchanged correction checkpoints."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import budget_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('budget_recorded_common','evaluate-alternating-depth-refinement.py')


def checkpoint(report,index):
    chosen = min(index,len(report['segments'])-1);segment = report['segments'][chosen]
    return dict(requested_cap=10*(index+1),actual_applied_corrections=sum(s['applied_iterations'] for s in report['segments'][:chosen+1]),
        terminal_carried_forward=index > chosen,fresh_converged=segment['fresh_converged'],guard_failure=segment['guard_failure'],
        termination=segment['termination'],fresh_rms_m=segment['final_fresh_equation']['rms_m'],
        **{k:report['endpoints'][chosen][k] for k in ('max_translation_proposal_m','max_rotation_proposal_rad')})


def statistics(rows):
    result = {}
    for index in range(3):
        values = [r['checkpoints'][index] for r in rows]
        result[str(10*(index+1))] = dict(samples=len(values),fresh_converged=sum(v['fresh_converged'] for v in values),
            guards=sum(v['guard_failure'] is not None for v in values),budget_stops=sum(v['termination'] == 'budget' for v in values),
            applied_corrections=sum(v['actual_applied_corrections'] for v in values),
            fresh_rms_gate_passes=sum(v['fresh_rms_m'] <= .03 for v in values),
            median_fresh_rms_m=float(np.median([v['fresh_rms_m'] for v in values])),
            terminal_carried_forward=sum(v['terminal_carried_forward'] for v in values))
    eligible = (result['30']['fresh_converged'] == len(rows) and result['30']['guards'] == 0
        and result['30']['fresh_rms_gate_passes'] == len(rows) and result['30']['median_fresh_rms_m'] <= result['10']['median_fresh_rms_m'])
    return dict(checkpoints=result,eligible_for_followup_full_trajectory=eligible,
        total_association_rounds=sum(r['refinement']['association_rounds'] for r in rows),
        endpoint_pair_occurrences=sum(e['totals']['before_count'] for r in rows for e in r['refinement']['endpoints']),
        endpoint_stable_pair_occurrences=sum(e['totals']['stable_pairs'] for r in rows for e in r['refinement']['endpoints']),
        endpoint_retargeted_source_occurrences=sum(e['totals']['retargeted_sources'] for r in rows for e in r['refinement']['endpoints']))


def recorded(case,source):
    saved = source['recorded'][case]
    require([s['source_index'] for s in saved['samples']] == list(SAMPLES),'ten fixed development windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body)
    require(sha(body) == saved['input_freeze_sha256'],'original input freeze')
    inventory = json.loads((parent/'files.json').read_text())
    require(saved['sensor_ledger_sha256'] == inventory[prefix+'sensor-only.json.gz']['sha256'],'original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'],'same original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']}
    audit = load('budget_raw_'+case.replace('-','_'),'check-reciprocal-keyframes.py').auditor()
    checker = load('budget_verifier_'+case.replace('-','_'),'check-budget-depth-refinement.py')
    surfaces = {};rows = [];systems = repeated = transitions = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original depth PNG')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original image hash')
            image = Image.open(io.BytesIO(raw));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth image')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.local.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera']))
        return surfaces[stamp]
    for sample in saved['samples']:
        initial = np.asarray(sample['refinement']['initial_poses']);prior = sample['prior_before']
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        independent = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        report = kernel.refine(nodes,initial,common.prior_object(prior))
        require(report['segments'][0] == sample['refinement'],'first ten corrections exactly reproduce archived control')
        checked = checker.verify(report,independent,initial,prior,audit)
        systems += checked['reported_systems'];repeated += checked['additional_endpoint_system_checks'];transitions += checked['pair_transitions']
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            first_segment_matches_archived_control=True,refinement=report,checkpoints=[checkpoint(report,i) for i in range(3)]))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],json.dumps(rows[-1]['checkpoints']),flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_reported_systems=systems,additional_endpoint_system_checks=repeated,
        independently_checked_pair_transitions=transitions,statistics=statistics(rows),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/alternating-depth-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free local input')
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda c:recorded(c,source),CASES),strict=True))
    names = set(json.loads((ROOT/'assets/alternating-depth-refinement-v1/source-freeze.json').read_text()))
    names |= {'scripts/'+name for name in ('budget_depth_refinement.py','check-budget-depth-refinement.py','evaluate-budget-depth-refinement.py')}
    names.add('assets/budget-depth-refinement-v1/design.json')
    write(out/'summary.json',dict(data_role='already viewed development; budget diagnostic only',sample_indices=list(SAMPLES),
        input_alternating_sha256=sha(body),real_local_pose_fitting=True,new_trajectory_fitting=False,numeric_ground_truth_parsed=False,
        tracker_state_commits=False,default_changed=False,physical_accuracy_scored=False,
        eligible_for_followup_full_trajectory=all(r['statistics']['eligible_for_followup_full_trajectory'] for r in reports.values()),
        recorded=reports,source_sha256={name:sha((ROOT/name).read_bytes()) for name in sorted(names)}))


if __name__ == '__main__':main()
