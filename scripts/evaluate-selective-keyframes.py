#!/usr/bin/env python3
"""Frozen coarse-skip retained comparison; all sensor work and audits before numeric labels."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from selective_keyframe_depth import Tracker
from projective_depth_common import CAMERA, ROOT, fresh, require, sha, write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')


def operational(case,mode,out):
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();manifest = json.loads(body)
    frames = manifest['frames'];inventory = {}
    for v in manifest['files']:
        name = v['file']
        require(Path(name).name == name and name not in inventory and 0 < v['bytes'] <= 4*1024*1024,'bounded unique input')
        inventory[name] = v
    def verified(name):
        path = base/'raw'/name
        require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 4*1024*1024,'bounded real source')
        b = path.read_bytes();require(len(b) == inventory[name]['bytes'] and sha(b) == inventory[name]['sha256'],'source digest '+name)
        return b
    for name in inventory:verified(name)
    require(len(frames) == 180 and all(b['depth_timestamp'] > a['depth_timestamp'] for a,b in zip(frames,frames[1:])),'all180 selected acquisitions')
    sources = [Path(__file__),*[ROOT/'scripts'/x for x in ('selective_keyframe_depth.py','check-selective-keyframes.py','diagnose-depth-levels.py','patch_keyframe_depth.py','patch_depth_normals.py','symmetric_keyframe_depth.py','symmetric_depth_factors.py','check-symmetric-keyframes.py','check-patch-keyframes.py',
        'marginalized_keyframe_depth.py','check-marginalized-keyframes.py',
        'joint_keyframe_depth.py','multireference_rgbd_common.py','projective_depth_raw.py','projective_depth_common.py',
        'check-projective-depth.py','check-opencv-rgbd.py','requirements-visual.txt')],ROOT/'assets/patch-keyframe-depth-v1/design.json',ROOT/'assets/selective-keyframe-depth-v1/design.json',ROOT/'assets/selective-keyframe-depth-v1/level-diagnostic.json']
    write(out/'freeze.json',dict(mode=mode,role='already viewed development; not held out',camera=CAMERA,
        manifest_sha256=sha(body),input_sha256=inventory,source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources},
        ground_truth_operational=False,rgb_decoded=False,updates=179,pose_resets=0,numpy_version=np.__version__,
        root_translation_gate_m=.1,root_rotation_gate_rad=.1,reference_bracket_max_s=.02,audit_workers=2))
    tracker = Tracker(CAMERA,mode);rows = []
    for frame in frames:
        image = Image.open(io.BytesIO(verified(frame['depth_file'])))
        require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth PNG')
        depth = np.asarray(image,dtype=float)/5000
        rows.append(dict(source_index=frame['source_index'],update=tracker.step(frame['depth_timestamp'],depth)))
        if len(rows)%30 == 0:print(case,mode,len(rows),flush=True)
    write(out/'sensor-only.json',dict(frames=rows,ground_truth_parsed=False))
    return frames,rows,verified


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);saved = {}
    for case in CASES:
        for mode in ('retain',):
            name = case+'-'+mode;folder = out/name;folder.mkdir();saved[name] = operational(case,mode,folder)
    spec = importlib.util.spec_from_file_location('selective_audit',ROOT/'scripts/check-selective-keyframes.py')
    wrapper = importlib.util.module_from_spec(spec);spec.loader.exec_module(wrapper)
    integrity = {}
    def audit_case(name):
        # Every worker gets a private audit module and callbacks. No operational
        # fitter state is modified, and all audits finish before reference labels.
        frames,rows,verified = saved[name];audit = wrapper.auditor()
        report = audit.recorded(rows,frames,verified,name.rsplit('-',1)[1],CAMERA,progress_label=name)
        write(out/name/'integrity.json',report);print(name,'raw/state integrity',json.dumps(report),flush=True)
        return report
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = {name:pool.submit(audit_case,name) for name in saved}
        for name,future in pending.items():integrity[name] = future.result()
    spec = importlib.util.spec_from_file_location('physical_reference',ROOT/'scripts/check-opencv-rgbd.py')
    reference = importlib.util.module_from_spec(spec);spec.loader.exec_module(reference);reference.controls();summaries = {}
    for name,(frames,rows,verified) in saved.items():
        labels = np.loadtxt(io.BytesIO(verified('groundtruth.txt')),comments='#')
        require(labels.ndim == 2 and labels.shape[1] == 8 and len(labels) <= 30000 and np.isfinite(labels).all()
                and np.all(np.diff(labels[:,0]) > 0) and np.all(np.linalg.norm(labels[:,4:],axis=1) > .5),'whole chronological physical reference')
        origin = np.linalg.inv(reference.truth_at(labels,frames[0]['depth_timestamp']));errors = []
        for frame,row in zip(frames[1:],rows[1:]):
            u = row['update']
            if not u['accepted']:continue
            estimated = np.asarray(u['root']);truth = origin@reference.truth_at(labels,frame['depth_timestamp'])
            p = float(np.linalg.norm(estimated[:3,3]-truth[:3,3]))
            a = float(np.arccos(np.clip((np.trace(truth[:3,:3].T@estimated[:3,:3])-1)/2,-1,1)))
            row.update(position_error_m=p,rotation_error_rad=a,accurate=p <= .1 and a <= .1);errors.append((p,a))
        summary = dict(total_updates=179,accepted=len(errors),root_accurate=sum(r.get('accurate',False) for r in rows),
            lost=rows[-1]['update']['lost'],max_position_error_m=max((x[0] for x in errors),default=None),
            max_rotation_error_rad=max((x[1] for x in errors),default=None),integrity=integrity[name])
        summary['complete_accuracy_pass'] = summary['root_accurate'] == 179 and not summary['lost']
        summaries[name] = summary;write(out/name/'results.json',dict(summary=summary,frames=rows));print(name,json.dumps(summary),flush=True)
    passed = all(s['complete_accuracy_pass'] for s in summaries.values())
    write(out/'comparison.json',dict(summaries=summaries,complete_accuracy_pass=passed,exit_code=0 if passed else 1))
    return 0 if passed else 1


if __name__ == '__main__':sys.exit(main())
