#!/usr/bin/env python3
"""Frozen-pair refinement of ten saved fits; all sensor diagnostics without labels."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import fixed_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def archived(packet,name):
    expected = json.loads((packet/'files.json').read_text())[name];body = (packet/name).read_bytes()
    require(len(body) == expected['bytes'] and sha(body) == expected['sha256'],'archived refinement input '+name)
    return body


def prior_object(value):
    if value is None:return None
    return kernel.base.Prior(value['origin'],value['ids'],[np.asarray(p) for p in value['anchors']],
        np.asarray(value['h']),np.asarray(value['g']),list(map(tuple,value['consumed_pairs'])),value['generation'])


def stats(rows):
    return dict(samples=len(rows),fixed_converged=sum(r['refinement']['fixed_converged'] for r in rows),
        fresh_converged=sum(r['refinement']['final_fresh_correction']['converged'] for r in rows),
        control_fresh_converged=sum(r['control_fresh_correction']['converged'] for r in rows),
        frozen_guard_failures=sum(r['refinement']['guard_failure'] is not None for r in rows),
        fresh_guard_failures=sum(r['refinement']['final_fresh_correction']['guard_failure'] is not None for r in rows),
        fresh_rms_gate_passes=sum(r['refinement']['final_fresh_equation']['rms_m'] <= .03 for r in rows),
        new_applied_iterations=sum(r['refinement']['applied_iterations'] for r in rows),
        control_applied_iterations=10*len(rows),
        new_fresh_rms_m_median=float(np.median([r['refinement']['final_fresh_equation']['rms_m'] for r in rows])),
        control_fresh_rms_m_median=float(np.median([r['control_final_equation']['rms_m'] for r in rows])))


def recorded(case,source):
    saved = source['recorded'][case]
    require([s['source_index'] for s in saved['samples']] == list(SAMPLES),'all ten fixed-viewed sample indices')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = archived(parent,prefix+'freeze.json');freeze = json.loads(body)
    require(sha(body) == saved['input_freeze_sha256'],'same input freeze')
    inventory = json.loads((parent/'files.json').read_text())
    require(saved['sensor_ledger_sha256'] == inventory[prefix+'sensor-only.json.gz']['sha256'],'same input ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'],'same image manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']}
    audit = load('fixed_eval_audit_'+case.replace('-','_'),'check-reciprocal-keyframes.py').auditor()
    verifier = load('fixed_eval_verifier_'+case.replace('-','_'),'check-fixed-depth-refinement.py')
    surfaces = {};rows = [];systems = 0;control_systems = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original depth')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original image digest')
            image = Image.open(io.BytesIO(raw));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera']))
        return surfaces[stamp]
    for sample in saved['samples']:
        initial = np.asarray(sample['initial_poses']);prior = sample['prior_before'];op_prior = prior_object(prior)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        independent = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        initial_system = audit.independent_equation(independent,initial,1,prior)
        audit.compare_equation(sample['trace'][0]['equation'],initial_system);control_systems += 1
        report = kernel.refine(nodes,initial,op_prior)
        systems += verifier.verify(report,independent,initial,prior,audit)
        old_pose = np.asarray(sample['final_poses']);old = audit.independent_equation(independent,old_pose,1,prior)
        audit.compare_equation(sample['final_equation'],old);control_systems += 1
        reason,d,condition = verifier.failure(old,len(nodes)-1,audit)
        control_correction = dict(guard_failure=reason,converged=reason is None and verifier.small(d),
            delta=None if reason is not None else d.tolist(),condition=None if reason is not None else condition)
        for node,p in zip(nodes,initial):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_final_poses=sample['final_poses'],control_final_equation=sample['final_equation'],
            control_fresh_correction=control_correction,refinement=report))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'fixed/fresh convergence',report['fixed_converged'],report['final_fresh_correction']['converged'],
            'iterations',report['applied_iterations'],'guard',report['guard_failure'],flush=True)
    result = stats(rows)
    result['advance_to_full_trajectory'] = (result['fresh_converged'] > result['control_fresh_converged']
        and result['frozen_guard_failures'] == result['fresh_guard_failures'] == 0 and result['fresh_rms_gate_passes'] == 5
        and result['new_fresh_rms_m_median'] <= result['control_fresh_rms_m_median'])
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_new_systems=systems,independently_reconstructed_control_systems=control_systems,
        statistics=result,samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = archived(ROOT/'assets/depth-convergence-v1','summary.json');source = json.loads(body)
    require(not source['real_numeric_ground_truth_parsed'],'no numeric labels in diagnostic input')
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda case:recorded(case,source),CASES),strict=True))
    names = set(json.loads((ROOT/'assets/depth-correspondence-v1/source-freeze.json').read_text()))
    names |= {'scripts/'+name for name in ('fixed_depth_refinement.py','check-fixed-depth-refinement.py','evaluate-fixed-depth-refinement.py')}
    names.add('assets/fixed-depth-refinement-v1/design.json')
    advance = all(r['statistics']['advance_to_full_trajectory'] for r in reports.values())
    write(out/'summary.json',dict(data_role='already viewed development; local refinement only',sample_indices=list(SAMPLES),
        input_continuation_sha256=sha(body),real_local_pose_fitting=True,new_trajectory_fitting=False,
        numeric_ground_truth_parsed=False,tracker_state_commits=False,default_changed=False,
        advance_to_full_trajectory=advance,recorded=reports,
        source_sha256={name:sha((ROOT/name).read_bytes()) for name in sorted(names)}))


if __name__ == '__main__':main()
