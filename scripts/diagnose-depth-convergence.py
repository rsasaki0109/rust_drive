#!/usr/bin/env python3
"""Guarded finest-level continuations of frozen reciprocal fits; no reference labels."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import reciprocal_keyframe_depth as kernel
from projective_depth_common import ROOT, exp_rotation, fresh, require, sha, write

SAMPLES = (120,160,200,240,279)
ADDITIONAL_CAP = 10


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def decode_prior(value):
    if value is None:return None
    return kernel.Prior(value['origin'],value['ids'],[np.asarray(p) for p in value['anchors']],
        np.asarray(value['h']),np.asarray(value['g']),list(map(tuple,value['consumed_pairs'])),value['generation'])


def norms(delta):
    return dict(max_translation_m=float(np.linalg.norm(delta[:,:3],axis=1).max()),
        max_rotation_rad=float(np.linalg.norm(delta[:,3:],axis=1).max()))


def continue_fine(nodes,saved,prior):
    """Diagnostic copies only: no cache, key, clock or marginal-prior commit."""
    require(2 <= len(nodes) <= 5,'bounded diagnostic window')
    poses = np.asarray(saved,dtype=float).copy();origin = poses.copy();op_prior = decode_prior(prior)
    require(poses.shape == (len(nodes),4,4),'diagnostic pose shape')
    for pose in poses:kernel.valid_pose(pose)
    require(np.array_equal(poses[0],np.eye(4)),'fixed diagnostic origin')
    steps = [];trace = [];converged = False;failure = None
    for iteration in range(ADDITIONAL_CAP+1):
        e = kernel.equation(nodes,poses,1,op_prior);d = condition = None
        try:
            require(any(x['source'] == len(nodes)-1 for x in e['edges']),'new frame needs measured support')
            d,condition = kernel.solve(e['h'],e['g']);d = d.reshape(-1,6)
            require(all(np.linalg.norm(x[:3]) <= .1 and np.linalg.norm(x[3:]) <= .15 for x in d),'increment')
        except ValueError as error:
            require(str(error) in ('new frame needs measured support','zero/nonfinite diagonal','condition','increment'),
                'recognized diagnostic guard failure')
            failure = str(error)
        steps.append(dict(iteration=iteration,count=e['count'],raw_rms_m=e['rms_m'],condition=condition,
            guard_failure=failure,proposed_step=None if d is None else norms(d),
            max_node_translation_from_saved_m=float(np.linalg.norm(poses[:,:3,3]-origin[:,:3,3],axis=1).max())))
        if failure is not None:break
        converged = all(np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-5 for x in d)
        if converged or iteration == ADDITIONAL_CAP:break
        before = poses.copy()
        for node,delta in enumerate(d,1):
            inc = np.eye(4);inc[:3,:3],inc[:3,3] = exp_rotation(delta[3:]),delta[:3]
            poses[node] = inc@poses[node];kernel.valid_pose(poses[node])
        trace.append(dict(scale=1,before=before.tolist(),equation=kernel.encode_equation(e),
            delta=d.tolist(),condition=condition,after=poses.tolist()))
    return dict(initial=dict(count=steps[0]['count'],raw_rms_m=steps[0]['raw_rms_m']),steps=steps,
        trace=trace,final_equation=kernel.encode_equation(e),initial_poses=origin.tolist(),final_poses=poses.tolist(),
        additional_applied_iterations=len(trace),converged=converged,guard_failure=failure,
        final_raw_residual_gate=steps[-1]['raw_rms_m'] <= .03)


def recorded(case):
    packet = ROOT/'assets/reciprocal-keyframe-depth-v1'
    manifest = json.loads((packet/'files.json').read_text());prefix = 'first/'+case+'-retain/'
    def archived(name):
        body = (packet/name).read_bytes();expected = manifest[name]
        require(len(body) == expected['bytes'] and sha(body) == expected['sha256'],'archived source '+name)
        return body
    freeze_body = archived(prefix+'freeze.json');freeze = json.loads(freeze_body)
    sensor_body = archived(prefix+'sensor-only.json.gz');sensor = json.loads(gzip.decompress(sensor_body))
    require(not sensor['ground_truth_parsed'],'sensor-only source')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'],'original manifest')
    by_stamp = {r['depth_timestamp']:r for r in json.loads(body)['frames']}
    audit = load('convergence_audit_'+case.replace('-','_'),'check-reciprocal-keyframes.py').auditor()
    verifier = load('convergence_verifier_'+case.replace('-','_'),'check-depth-convergence.py')
    surfaces = {};prior = None;rows = [];checked = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name]
            raw = (base/'raw'/name).read_bytes()
            require(0 < len(raw) <= 4*1024*1024 and len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'depth source '+name)
            image = Image.open(io.BytesIO(raw))
            require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera']))
        return surfaces[stamp]
    for entry in sensor['frames']:
        u = entry['update'];require(u['prior_before_sha256'] == audit.digest(prior),'frozen prior chain')
        if entry['source_index'] in SAMPLES:
            require(u['accepted'],'fixed accepted diagnostic sample')
            saved = np.asarray(u['revised_roots'])
            nodes = [dict(timestamp=t,root=p,surface=surface(t)[0]) for t,p in zip(u['nodes'],saved,strict=True)]
            independent_nodes = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
            audit.compare_equation(u['final_equation'],audit.independent_equation(independent_nodes,saved,1,prior))
            report = continue_fine(nodes,saved,prior)
            checked += verifier.verify(report,independent_nodes,saved,prior,audit)
            report.update(source_index=entry['source_index'],node_timestamps=u['nodes'],prior_before=prior)
            rows.append(report)
            print(case,entry['source_index'],'additional',report['additional_applied_iterations'],
                'converged',report['converged'],'rms',report['initial']['raw_rms_m'],
                report['steps'][-1]['raw_rms_m'],'failure',report['guard_failure'],flush=True)
            surfaces = {t:surfaces[t] for t in u['nodes']}
        if u['accepted']:prior = u['prior_commit']
    require(len(rows) == len(SAMPLES),'all fixed indices')
    return dict(input_freeze_sha256=sha(freeze_body),sensor_ledger_sha256=sha(sensor_body),
        independent_systems=checked,samples=rows,converged_samples=sum(r['converged'] for r in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    cases = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(cases,pool.map(recorded,cases),strict=True))
    baseline = json.loads((ROOT/'assets/reciprocal-keyframe-depth-v1/source-freeze.json').read_text())
    names = set(baseline)|{'scripts/diagnose-depth-convergence.py','scripts/check-depth-convergence.py'}
    write(out/'summary.json',dict(data_role='already viewed development; fixed-pose diagnostic only',
        sample_indices=list(SAMPLES),additional_cap=ADDITIONAL_CAP,real_pose_continuations=True,
        new_trajectory_fitting=False,prior_commits=False,real_numeric_ground_truth_parsed=False,
        residual_population_can_change=True,recorded=reports,
        source_sha256={name:sha((ROOT/name).read_bytes()) for name in sorted(names)}))


if __name__ == '__main__':main()
