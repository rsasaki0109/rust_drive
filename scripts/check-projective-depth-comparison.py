#!/usr/bin/env python3
"""Check complete native trajectories and enforce the default-adoption gate."""
import argparse
import gzip
import importlib.util
import io
import json
from pathlib import Path

import numpy as np

from projective_depth_common import ROOT,audit_sequence,pose,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
spec = importlib.util.spec_from_file_location('native_comparison_reference',ROOT/'scripts/check-opencv-rgbd.py')
reference = importlib.util.module_from_spec(spec);spec.loader.exec_module(reference)


def verify(run,case):
    folder = run/case;freeze = json.loads((folder/'freeze.json').read_text())
    ledger = json.loads((folder/'sensor-only.json').read_text());report = json.loads((folder/'results.json').read_text())
    require(not ledger['ground_truth_parsed'] and freeze['pose_resets']==0 and not freeze['ground_truth_operational'],'measurement-only continuous tracking')
    require(freeze['root_translation_gate_m']==freeze['root_rotation_gate_rad']==.1 and freeze['reference_bracket_max_s']==.02,'unchanged physical gates')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body)==freeze['manifest_sha256'],'original manifest')
    frames = json.loads(body)['frames'];require([r['source_index'] for r in frames]==list(range(100,280)),'all180 acquisitions')
    body = (base/'raw/groundtruth.txt').read_bytes();pin = freeze['inputs']['groundtruth.txt']
    require(sha(body)==pin['sha256'] and len(body)==pin['bytes'],'original reference bytes')
    labels = np.loadtxt(io.BytesIO(body),comments='#');require(labels.shape[1]==8 and len(labels)<=30000 and np.isfinite(labels).all() and np.all(np.diff(labels[:,0])>0),'bounded chronological reference')
    origin = np.linalg.inv(reference.truth_at(labels,frames[0]['depth_timestamp']))
    rows = report['frames'];require(len(rows)==len(ledger['frames'])==180,'complete pre-label and scored ledgers')
    steps = audit_sequence(ledger['frames'],[f['depth_timestamp'] for f in frames])
    accepted = accurate = 0;errors = []
    for index,(frame,saved,row) in enumerate(zip(frames,ledger['frames'],rows,strict=True)):
        require(saved['source_index']==row['source_index']==frame['source_index'] and saved['update']==row['update'],'operational outcomes retained before scoring')
        if not index:continue
        update = row['update']
        if update['accepted']:
            estimated = pose(update['first_from_current']);truth = origin@reference.truth_at(labels,frame['depth_timestamp'])
            translation = float(np.linalg.norm(estimated[:3,3]-truth[:3,3]));rotation = truth[:3,:3].T@estimated[:3,:3]
            skew = np.array([rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1]])
            angle = float(np.arctan2(np.linalg.norm(skew)/2,(np.trace(rotation)-1)/2))
            require(abs(row['position_error_m']-translation)<1e-9 and abs(row['rotation_error_rad']-angle)<2e-7,'recomputed translation and atan2 rotation errors')
            good = translation<=.1 and angle<=.1;require(row['accurate']==good,'physical accuracy classification')
            accepted += 1;accurate += good;errors.append((translation,angle))
    summary = dict(total_updates=179,accepted=accepted,root_accurate=accurate,lost=rows[-1]['update']['lost'],
        max_position_error_m=max((e[0] for e in errors),default=None),max_rotation_error_rad=max((e[1] for e in errors),default=None))
    original = report['summary']
    for key,value in summary.items():
        if key.startswith('max_'):require(abs(original[key]-value)<2e-7,'summary maximum '+key)
        else:require(original[key]==value,'summary count/state '+key)
    require(original['independently_checked_normal_steps']==steps and original['complete_accuracy_pass']==(accurate==179 and not summary['lost']),'honest complete accuracy and work')
    return summary,freeze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,required=True);parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--raw-audit',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();reference.controls();cases = {};passed = True
    raw = json.loads((args.raw_audit/'summary.json').read_text());require(raw['pass_raw_mathematics'],'complete independent raw audit')
    for case in CASES:
        before,old = verify(args.baseline,case);after,new = verify(args.candidate,case)
        original = ROOT/'assets/projective-depth-rust-v1'/case/'sensor-only.json.gz'
        pin = json.loads((original.parent.parent/'files.json').read_text())[str(original.relative_to(original.parent.parent))]
        body = original.read_bytes();require(len(body)==pin['bytes'] and sha(body)==pin['sha256'],'frozen legacy baseline')
        require(json.loads(gzip.decompress(body))==json.loads((args.baseline/case/'sensor-only.json').read_text()),'baseline reproduces all original native outcomes')
        require(old['inputs']==new['inputs'] and old['camera']==new['camera'] and old['manifest_sha256']==new['manifest_sha256'],'identical measurements and calibration')
        audit_freeze = json.loads((args.raw_audit/case/'freeze.json').read_text())
        require(audit_freeze['original_sensor_ledger_sha256']==sha((args.candidate/case/'sensor-only.json').read_bytes())
            and audit_freeze['original_freeze_sha256']==sha((args.candidate/case/'freeze.json').read_bytes()),'raw audit covers this candidate')
        improved = after['root_accurate']>before['root_accurate'] if case==CASES[0] else after['root_accurate']>=before['root_accurate']
        eligible = improved and after['accepted']==179 and not after['lost'] and all(after[k]<=before[k] for k in ('max_position_error_m','max_rotation_error_rad'))
        passed &= eligible;cases[case] = dict(baseline=before,candidate=after,default_adoption_pass=bool(eligible))
    write(args.output,dict(implementation_integrity_pass=True,cases=cases,default_adoption_pass=bool(passed),complete_physical_accuracy_pass=all(c['candidate']['root_accurate']==179 for c in cases.values()),
        shared_reference_interpolation=True,rotation_check='independent atan2 of relative rotation',viewed_development_data=True))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
