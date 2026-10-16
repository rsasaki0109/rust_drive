#!/usr/bin/env python3
"""Evaluate a legacy-preserving fitted-center union on ten viewed windows."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import fitted_center_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)
CAPS = (30,60,90)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


original = load('fitted_center_original_statistics','evaluate-boundary-slope-refinement.py')
common = original.common


def statistics(rows,controls):
    result = original.statistics(rows,controls)
    result['independently_checked_initial_stage_pairs'] = sum(len(r['admission_stages']) for r in rows)
    result['new_fitted_metric_source_occurrences'] = sum(s['new_fitted_metric_count'] for r in rows for s in r['selection'])
    result['new_legacy_metric_source_occurrences'] = sum(s['new_legacy_metric_count'] for r in rows for s in r['selection'])
    return result


def selection_receipt(nodes,initial,stages):
    old = {(r['target'],r['source']):r for r in kernel.previous.population(nodes,initial,1)}
    new = {(r['target'],r['source']):r for r in kernel.population(nodes,initial,1)}
    require(set(old)<=set(new),'all original factors retained');by_pair = {(s['target'],s['source']):s for s in stages};result = []
    for pair,after in new.items():
        before = old.get(pair);ids = np.array([],dtype=int) if before is None else before['source_ids'];index = np.flatnonzero(np.isin(after['source_ids'],ids))
        require(len(index)==len(ids),'all original source identities retained')
        if before is not None:
            for key in ('source_ids','points','normals','normalizer'):np.testing.assert_array_equal(before[key],after[key][index])
        stage = by_pair[pair];new_fitted = stage['admitted_fitted_only_count'];new_legacy = stage['legacy_after_probes']-len(ids)
        require(new_legacy>=0 and len(after['source_ids'])==len(ids)+new_fitted+new_legacy,'separate added C1 metrics and restored legacy metrics')
        result.append(dict(target=pair[0],source=pair[1],old_count=len(ids),new_count=len(after['source_ids']),retained_old_count=len(ids),
            newly_admitted_count=len(after['source_ids'])-len(ids),new_fitted_metric_count=new_fitted,new_legacy_metric_count=new_legacy,
            source_values_and_normalizers_preserved=True))
    return result


def recorded(case,source):
    saved = source['recorded'][case];require([s['source_index'] for s in saved['samples']]==list(SAMPLES),'same ten viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body);require(sha(body)==saved['input_freeze_sha256'],'original input freeze')
    inventory = json.loads((parent/'files.json').read_text());require(saved['sensor_ledger_sha256']==inventory[prefix+'sensor-only.json.gz']['sha256'],'original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body)==freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']};suffix = case.replace('-','_')
    audit = load('fitted_center_raw_'+suffix,'check-reciprocal-keyframes.py').auditor();verifier = load('fitted_center_verify_'+suffix,'check-fitted-center-refinement.py')
    surfaces = {};rows = [];systems = controls = endpoints = fresh_systems = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];receipt = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0<path.stat().st_size<=4*1024*1024,'bounded original PNG')
            body = path.read_bytes();require(sha(body)==receipt['sha256'] and len(body)==receipt['bytes'],'original PNG digest')
            image = Image.open(io.BytesIO(body));require(image.size==(640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth image')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.prepare([dict(surface=kernel.base.pyramid(depth,freeze['camera']))])[0]['surface'],
                verifier.prepare([dict(surface=audit.pyramid(depth,freeze['camera']))])[0]['surface'])
        return surfaces[stamp]
    for sample in saved['samples']:
        control = sample['refinement'];initial = np.asarray(control['initial_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
        prior_before = None if op_prior is None else json.dumps(op_prior.encode(),sort_keys=True)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        fields = verifier.compare_prepared(nodes,other);controls += verifier.previous.verify(control,other,initial,prior,audit)
        selected,stages = kernel.selection(nodes,initial,1);wanted,expected = verifier.selection(other,initial,1)
        require(stages==expected,'independent raw/legacy/fitted/union/probe stage counts');verifier.check_metadata(kernel.metadata(selected),wanted)
        selection = selection_receipt(nodes,initial,stages)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,other,initial,prior,audit)
        checkpoints = [kernel.checkpoint(report,nodes,op_prior,cap) for cap in CAPS]
        for cap,cp in zip(CAPS,checkpoints,strict=True):endpoints += verifier.verify_checkpoint(cp,report,other,initial,prior,audit,cap)
        rejected_future = None
        if report['applied_iterations']>30:
            bad = copy.deepcopy(checkpoints[0]);bad['final_poses'] = report['final_poses'];require(bad['final_poses']!=checkpoints[0]['final_poses'],'future state differs from cap30')
            try:verifier.verify_checkpoint(bad,report,other,initial,prior,audit,30)
            except (ValueError,AssertionError):rejected_future = True
            else:raise ValueError('future terminal state accepted at cap30')
        terminal = np.asarray(report['final_poses']);after = kernel.population(nodes,terminal,1);ind_after = verifier.population(other,terminal,1)
        verifier.check_metadata(kernel.metadata(after),ind_after)
        e = kernel.equation(after,nodes,terminal,op_prior,1);independent = verifier.system(ind_after,other,terminal,prior,1,audit)
        audit.compare_equation(kernel.base.encode_equation(e),independent);fresh_systems += 1
        correction = kernel.discrete.correction(e,len(nodes)-1);verifier.reference.check_correction(correction,independent,len(nodes)-1,audit)
        require(prior_before==(None if op_prior is None else json.dumps(op_prior.encode(),sort_keys=True)),'incoming nonlinear prior unchanged')
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_report_sha256=sha(json.dumps(control,sort_keys=True).encode()),complete_control_report_equal=report==control,
            selection=selection,admission_stages=stages,depth_slope_fields=fields,refinement=report,checkpoints=checkpoints,
            cap30_future_state_substitution_rejected=rejected_future,terminal_reselected=dict(population=kernel.metadata(after),equation=kernel.base.encode_equation(e),correction=correction),incoming_prior_unchanged=True))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'sources',sum(p['count'] for p in report['frozen_population']),'updates',report['applied_iterations'],
            'continuous/discrete',report['continuous_converged'],report['final_discrete_correction']['converged'],'stop',report['termination'],report['guard_failure'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_systems=systems,independently_rechecked_control_systems=controls,independently_checked_checkpoint_systems=endpoints,
        independently_checked_reselected_systems=fresh_systems,statistics=statistics(rows,saved['samples']),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/boundary-slope-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free original local input')
    freeze = json.loads((ROOT/'assets/fitted-center-refinement-v1/source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'frozen source '+name)
    analytic = load('fitted_center_recorded_analytic','check-fitted-center-refinement.py').controls()
    analytic['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('fitted_center_refinement.py','check-fitted-center-refinement.py')}
    write(out/'analytic.json',analytic)
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda case:recorded(case,source),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'unchanged source '+name)
    write(out/'summary.json',dict(data_role='already viewed development; legacy-preserving fitted-center admission',sample_indices=list(SAMPLES),checkpoint_caps=list(CAPS),
        input_boundary_slope_sha256=sha(body),real_local_pose_fitting=True,new_trajectory_fitting=False,numeric_ground_truth_parsed=False,
        physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        eligible_for_followup_full_trajectory=all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values()),recorded=reports,source_sha256=freeze))


if __name__=='__main__':main()
