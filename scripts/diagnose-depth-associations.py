#!/usr/bin/env python3
"""Known noisy motion and fixed-pose recorded reciprocal association diagnostic."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from projective_depth_common import ROOT, fresh, require, sha, write
from reciprocal_depth_oracle import independent_raw
from selective_keyframe_depth import Tracker

SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def synthetic():
    audit = load('association_noise_control','check-selective-keyframes.py').auditor()
    reports = {}
    for name,velocity in [('stationary',0.),('moving',1.)]:
        rng = np.random.default_rng(891021);tracker = Tracker(audit.rays.CAMERA,'retain');rows = []
        for i in range(15):
            truth = audit.increment(np.eye(4),velocity*np.array([.04*i,0,.002*i,0,.0015*i,0]))
            depth = audit.rays.render(truth)+rng.normal(0,.002,(120,160))
            u = tracker.step(1.+i*.03,depth)
            row = dict(index=i,accepted=u['accepted'],lost=u['lost'],rejection=u.get('rejection'))
            if i and u['accepted']:
                pose = np.asarray(u['root'])
                row.update(position_error_m=float(np.linalg.norm(pose[:3,3]-truth[:3,3])),
                    rotation_error_rad=float(np.arccos(np.clip((np.trace(truth[:3,:3].T@pose[:3,:3])-1)/2,-1,1))))
            rows.append(row)
        errors = [r for r in rows[1:] if r['accepted']]
        reports[name] = dict(denominator=14,accepted=len(errors),lost=rows[-1]['lost'],
            max_position_error_m=max((r['position_error_m'] for r in errors),default=None),
            max_rotation_error_rad=max((r['rotation_error_rad'] for r in errors),default=None),frames=rows)
    return dict(seed=891021,axial_gaussian_sigma_m=.002,frames=15,truth_operational=False,reports=reports)


def recorded(case):
    packet = ROOT/'assets/selective-keyframe-depth-v1'
    manifest = json.loads((packet/'files.json').read_text());prefix = 'first/'+case+'-retain/'
    def archived(name):
        body = (packet/name).read_bytes();expected = manifest[name]
        require(len(body) == expected['bytes'] and sha(body) == expected['sha256'],'archived source '+name)
        return body
    freeze_body = archived(prefix+'freeze.json');freeze = json.loads(freeze_body)
    sensor_body = archived(prefix+'sensor-only.json.gz');sensor = json.loads(gzip.decompress(sensor_body))
    require(not sensor['ground_truth_parsed'],'sensor-only control')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'],'original manifest')
    frames = json.loads(body)['frames'];by_stamp = {r['depth_timestamp']:r for r in frames}
    parent = load('association_control_'+case.replace('-','_'),'check-selective-keyframes.py').auditor()
    trial = load('association_trial_'+case.replace('-','_'),'check-reciprocal-keyframes.py').auditor()
    classifier = load('association_classifier_'+case.replace('-','_'),'check-selective-keyframes.py')
    surfaces = {};prior = None;rows = []
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name]
            raw = (base/'raw'/name).read_bytes()
            require(0 < len(raw) <= 4*1024*1024 and len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'depth source '+name)
            depth = np.asarray(Image.open(io.BytesIO(raw)),dtype=float)/5000
            surfaces[stamp] = parent.pyramid(depth,freeze['camera'])
        return surfaces[stamp]
    for entry in sensor['frames']:
        u = entry['update']
        require(u['prior_before_sha256'] == parent.digest(prior),'frozen prior chain')
        if entry['source_index'] in SAMPLES:
            require(u['accepted'],'selected admitted control pose')
            poses = np.asarray(u['revised_roots'])
            nodes = [dict(timestamp=t,root=p,surface=surface(t)) for t,p in zip(u['nodes'],poses,strict=True)]
            old = parent.independent_equation(nodes,poses,1,prior);parent.compare_equation(u['final_equation'],old)
            new = trial.independent_equation(nodes,poses,1,prior)
            reason,_,condition = classifier.failure(trial,new,len(nodes)-1)
            forward = reciprocal = 0;pairs = []
            for source in range(1,len(nodes)):
                for target in range(source):
                    selection = independent_raw([nodes[target]],nodes[source],poses[source],1,False)['selection']
                    forward += selection['forward_count'];reciprocal += selection['reciprocal_count']
                    pairs.append(dict(target=target,source=source,**selection))
            rows.append(dict(source_index=entry['source_index'],all_pair_forward_count=forward,all_pair_reciprocal_count=reciprocal,
                original_admitted_count=old['count'],reciprocal_admitted_count=new['count'],
                original_raw_rms_m=old['rms_m'],reciprocal_raw_rms_m=new['rms_m'],
                reciprocal_guard_failure=reason,reciprocal_scaled_condition=condition,pairs=pairs))
            surfaces = {t:surfaces[t] for t in u['nodes']}
            print(case,entry['source_index'],'reciprocal/forward',reciprocal,forward,'guard',reason,flush=True)
        if u['accepted']:prior = u['prior_commit']
    require(len(rows) == len(SAMPLES),'all fixed sample indices')
    return dict(input_freeze_sha256=sha(freeze_body),sensor_ledger_sha256=sha(sensor_body),samples=rows,
        forward_count=sum(r['all_pair_forward_count'] for r in rows),reciprocal_count=sum(r['all_pair_reciprocal_count'] for r in rows),
        same_pose_reciprocal_iteration_admissible=sum(r['reciprocal_guard_failure'] is None for r in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);known = synthetic()
    cases = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(cases,pool.map(recorded,cases),strict=True))
    names = ('diagnose-depth-associations.py','reciprocal_depth_oracle.py','check-reciprocal-keyframes.py','reciprocal_depth_factors.py',
        'reciprocal_keyframe_depth.py','check-selective-keyframes.py','selective_keyframe_depth.py','check-patch-keyframes.py',
        'patch_keyframe_depth.py','patch_depth_normals.py','check-symmetric-keyframes.py','symmetric_keyframe_depth.py',
        'symmetric_depth_factors.py','check-marginalized-keyframes.py','marginalized_keyframe_depth.py',
        'multireference_rgbd_common.py','joint_keyframe_depth.py','projective_depth_raw.py','projective_depth_common.py','check-projective-depth.py')
    write(out/'summary.json',dict(data_role='already viewed development; diagnostic only',sample_indices=list(SAMPLES),
        real_pose_fitting=False,real_numeric_ground_truth_parsed=False,synthetic_pose_fitting=True,
        synthetic=known,recorded=reports,source_sha256={'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in names}))


if __name__ == '__main__':main()
