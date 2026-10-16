#!/usr/bin/env python3
"""Compare completed and original normal Hermite fixed-source local fits on ten viewed windows."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import local_normal_depth_refinement as kernel
import normal_hermite_depth_refinement as original
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)
CAPS = (10,20,30)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('cubic_recorded_inputs','evaluate-alternating-depth-refinement.py')


def patch_statistics(layer,y,x,radius):
    z = layer['depth'][y-radius:y+radius+1,x-radius:x+radius+1]
    p = layer['vertices'][y-radius:y+radius+1,x-radius:x+radius+1];center = layer['depth'][y,x]
    valid = (z>.3)&(z<5)&(abs(z-center)<.05+.01*center);points = p[valid]
    if len(points)<3:return dict(depth=z.tolist(),gated_count=len(points),eigenvalues_m2=None,plane_ratio=None)
    offset = points-layer['vertices'][y,x];mean = offset.mean(axis=0);centered = offset-mean
    eigen = np.linalg.eigvalsh(centered.T@centered/len(points))
    singular = np.linalg.svd(points-points.mean(axis=0),compute_uv=False)
    np.testing.assert_allclose(eigen,singular[::-1]**2/len(points),atol=2e-16,rtol=1e-8)
    return dict(depth=z.tolist(),gated_count=len(points),eigenvalues_m2=eigen.tolist(),plane_ratio=float(eigen[0]/eigen[1]) if eigen[1]>0 else None)


def terminal_support(report,nodes,other,verifier,audit):
    if report['guard_failure']!='line search':return None
    initial = np.asarray(report['initial_poses']);poses = np.asarray(report['final_poses']);direction = np.asarray(report['final_correction']['delta'])
    matches = original.population(nodes,initial,1);expected = verifier.parent.population(other,initial,1)
    require(original.metadata(matches)==report['frozen_population'],'same control diagnostic sources')
    result = []
    for trial in report['failed_search']['trials']:
        alpha = trial['alpha'];candidate = poses.copy();independent = poses.copy()
        for node,d in enumerate(direction,1):
            inc = np.eye(4);inc[:3,:3] = original.discrete.exp_rotation(alpha*d[3:]);inc[:3,3] = alpha*d[:3];candidate[node] = inc@poses[node]
            independent[node] = audit.increment(poses[node],alpha*d)
        old_lost = new_lost = 0;lost_sources = []
        for item,oracle in zip(matches,expected,strict=True):
            a,b = item['target'],item['source'];layer = nodes[a]['surface'][1];other_layer = other[a]['surface'][1]
            world = item['points']@candidate[b,:3,:3].T+candidate[b,:3,3];q = (world-candidate[a,:3,3])@candidate[a,:3,:3]
            relative = np.linalg.solve(independent[a],independent[b]);iq = (np.column_stack([oracle['points'],np.ones(len(oracle['points']))])@relative.T)[:,:3]
            old_ids = original.sample(layer,q)[0];new_ids = kernel.sample(layer,q)[0]
            np.testing.assert_array_equal(old_ids,verifier.parent.independent_sample(other_layer,iq)[0])
            np.testing.assert_array_equal(new_ids,verifier.independent_sample(other_layer,iq)[0])
            require(np.array_equal(item['source_ids'],oracle['source_ids']),'same diagnostic source order')
            missing = np.setdiff1d(np.arange(len(q)),old_ids);old_lost += len(missing);new_lost += len(q)-len(new_ids)
            for k in missing:
                c = layer['camera'];uv = np.array([c['fx']*q[k,0]/q[k,2]+c['cx'],c['fy']*q[k,1]/q[k,2]+c['cy']]);x,y = np.floor(uv).astype(int)
                require(2<=x<c['width']-3 and 2<=y<c['height']-3,'interior support diagnosis')
                corners = []
                for dy in (0,1):
                    for dx in (0,1):
                        yy,xx = y+dy,x+dx
                        if np.linalg.norm(layer['normals'][yy,xx])<=.5:
                            corners.append(dict(pixel=[int(xx),int(yy)],normal_added=bool(np.linalg.norm(layer['hermite_normals'][yy,xx])>.5),
                                patch3=patch_statistics(layer,yy,xx,1),patch5=patch_statistics(layer,yy,xx,2)))
                lost_sources.append(dict(target=a,source=b,source_pixel=int(item['source_ids'][k]),uv=uv.tolist(),cell=[int(x),int(y)],missing_normals=corners))
        require(old_lost>0 and trial['guard_failure']=='fixed source support','old failed candidate domain independently reproduced')
        result.append(dict(alpha=alpha,old_missing_sources=old_lost,new_missing_sources=new_lost,lost_sources=lost_sources))
    return result


def statistics(rows,controls):
    reports = [r['refinement'] for r in rows];old = [r['refinement'] for r in controls]
    searches = [s['line_search'] for q in reports for s in q['trace']]
    trials = [t for s in searches for t in s['trials']]+[t for q in reports if q['failed_search'] is not None for t in q['failed_search']['trials']]
    continuous = sum(q['continuous_converged'] for q in reports);fresh = sum(q['final_discrete_correction']['converged'] for q in reports)
    guards = sum(q['guard_failure'] is not None for q in reports);fresh_guards = sum(q['final_discrete_correction']['guard_failure'] is not None for q in reports)
    rms = sum(q['final_discrete_equation']['rms_m']<=.03 for q in reports)
    median = float(np.median([q['final_discrete_equation']['rms_m'] for q in reports]));old_median = float(np.median([q['final_discrete_equation']['rms_m'] for q in old]))
    before = sum(p['count'] for q in old for p in q['frozen_population']);after = sum(p['count'] for q in reports for p in q['frozen_population'])
    eligible = continuous==len(rows) and fresh>sum(q['final_discrete_correction']['converged'] for q in old)
    eligible = eligible and guards==fresh_guards==0 and rms==len(rows) and median<=old_median
    checkpoints = {str(cap):dict(continuous_converged=sum(r['checkpoints'][i]['continuous_converged'] for r in rows),
        original_discrete_fresh_converged=sum(r['checkpoints'][i]['final_discrete_correction']['converged'] for r in rows),
        local_guards=sum(r['checkpoints'][i]['guard_failure'] is not None for r in rows),budget_exhaustions=sum(r['checkpoints'][i]['termination']=='budget' for r in rows)) for i,cap in enumerate(CAPS)}
    return dict(samples=len(rows),continuous_converged=continuous,control_continuous_converged=sum(q['continuous_converged'] for q in old),
        original_discrete_fresh_converged=fresh,local_guard_failures=guards,original_discrete_guard_failures=fresh_guards,
        original_discrete_rms_gate_passes=rms,median_original_discrete_rms_m=median,control_median_original_discrete_rms_m=old_median,
        source_occurrences_before=before,source_occurrences_after=after,source_retention=after/before if before else None,
        factors_before=sum(len(q['frozen_population']) for q in old),factors_after=sum(len(q['frozen_population']) for q in reports),
        budget_exhaustions=sum(q['termination']=='budget' for q in reports),total_applied_corrections=sum(q['applied_iterations'] for q in reports),
        shortened_steps=sum(s['accepted_alpha']<1 for s in searches),invalid_candidate_trials=sum(t['guard_failure'] is not None for t in trials),
        valid_nonpassing_candidate_trials=sum(t['guard_failure'] is None and t['cost']>t['limit'] for t in trials),
        accepted_candidate_trials=sum(t['guard_failure'] is None and t['cost']<=t['limit'] for t in trials),
        accepted_cost_increases=sum(s['trials'][-1]['cost']>s['before_cost'] for s in searches),checkpoints=checkpoints,
        eligible_for_followup_full_trajectory=bool(eligible))


def selection_receipt(nodes,initial):
    before = kernel.previous.population(nodes,initial,1);after = kernel.population(nodes,initial,1)
    by_pair = {(i['target'],i['source']):i for i in after};result = []
    for item in before:
        key = item['target'],item['source'];kept = by_pair.get(key)
        if kept is not None:
            indices = np.flatnonzero(np.isin(item['source_ids'],kept['source_ids']))
            for field in ('source_ids','normalizer','points','normals'):np.testing.assert_array_equal(item[field][indices],kept[field])
        result.append(dict(target=key[0],source=key[1],old_count=len(item['source_ids']),new_count=0 if kept is None else len(kept['source_ids']),original_values_preserved=True))
    require(set(by_pair)<=set((i['target'],i['source']) for i in before),'no new normal-Hermite source factors')
    return result


def recorded(case,source):
    saved = source['recorded'][case];require([s['source_index'] for s in saved['samples']]==list(SAMPLES),'same ten viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body);require(sha(body)==saved['input_freeze_sha256'],'same original freeze')
    inventory = json.loads((parent/'files.json').read_text());require(saved['sensor_ledger_sha256']==inventory[prefix+'sensor-only.json.gz']['sha256'],'same sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body)==freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']};suffix = case.replace('-','_')
    audit = load('cubic_raw_'+suffix,'check-reciprocal-keyframes.py').auditor();verifier = load('cubic_verify_'+suffix,'check-local-normal-depth-refinement.py')
    old_verifier = load('cubic_control_'+suffix,'check-normal-hermite-depth-refinement.py');surfaces = {};rows = [];systems = controls = endpoints = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0<path.stat().st_size<=4*1024*1024,'bounded original PNG')
            raw = path.read_bytes();require(len(raw)==expected['bytes'] and sha(raw)==expected['sha256'],'original PNG digest')
            image = Image.open(io.BytesIO(raw));require(image.size==(640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.prepare([dict(surface=kernel.base.pyramid(depth,freeze['camera']))])[0]['surface'],
                verifier.prepare([dict(surface=audit.pyramid(depth,freeze['camera']))])[0]['surface'])
        return surfaces[stamp]
    for sample in saved['samples']:
        control = sample['refinement'];initial = np.asarray(control['initial_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        controls += old_verifier.verify(control,other,initial,prior,audit)
        selection = selection_receipt(nodes,initial)
        completion = verifier.compare_prepared(nodes,other)
        support = terminal_support(control,nodes,other,verifier,audit)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,other,initial,prior,audit)
        require(report['initial_poses']==control['initial_poses'],'same original poses')
        checkpoints = [kernel.checkpoint(report,nodes,op_prior,cap) for cap in CAPS]
        for cap,cp in zip(CAPS,checkpoints,strict=True):endpoints += verifier.verify_checkpoint(cp,report,other,initial,prior,audit,cap)
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_report_sha256=sha(json.dumps(control,sort_keys=True).encode()),selection=selection,normal_completion=completion,control_terminal_support=support,refinement=report,checkpoints=checkpoints))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'updates',report['applied_iterations'],'continuous/discrete',report['continuous_converged'],
            report['final_discrete_correction']['converged'],'stop',report['termination'],report['guard_failure'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_systems=systems,independently_rechecked_control_systems=controls,independently_checked_checkpoint_systems=endpoints,
        statistics=statistics(rows,saved['samples']),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/normal-hermite-depth-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free local input')
    freeze = json.loads((ROOT/'assets/local-normal-depth-refinement-v1/source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'frozen source '+name)
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda c:recorded(c,source),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'unchanged source '+name)
    write(out/'summary.json',dict(data_role='already viewed development; C1 local normal-support completion comparison',sample_indices=list(SAMPLES),
        checkpoint_caps=list(CAPS),input_normal_hermite_sha256=sha(body),real_local_pose_fitting=True,new_trajectory_fitting=False,numeric_ground_truth_parsed=False,
        physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        eligible_for_followup_full_trajectory=all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values()),recorded=reports,source_sha256=freeze))


if __name__ == '__main__':main()
