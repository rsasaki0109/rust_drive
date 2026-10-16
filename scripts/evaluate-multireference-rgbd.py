#!/usr/bin/env python3
"""Fixed four-branch viewed-data experiment; all sensor work precedes numeric labels."""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from multireference_rgbd_common import MultiClient, audit_sequence, compare_raw, raw_equation, raw_reference
from projective_depth_common import CAMERA, ROOT, fresh, pose, require, sha, write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
MODES = {'geometry':'geometry-only','phase_gated_rgbd':'phase-gated-rgbd'}


def operational(case, mode, binary, output):
    base = ROOT/'data'/case
    body = (base/'manifest.json').read_bytes()
    manifest = json.loads(body)
    frames = manifest['frames']
    require(len(frames) == 180 and all(b['depth_timestamp'] > a['depth_timestamp']
            for a,b in zip(frames,frames[1:])),'all 180 chronological acquisitions')
    inventory = {}
    for item in manifest['files']:
        name = item['file']
        require(Path(name).name == name and name not in inventory and 0 < item['bytes'] <= 4*1024*1024,
                'bounded source inventory')
        inventory[name] = item
    def verified(name):
        path = base/'raw'/name
        require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 4*1024*1024,
                'bounded source file')
        data = path.read_bytes()
        require(len(data) == inventory[name]['bytes'] and sha(data) == inventory[name]['sha256'],
                'input digest: '+name)
        return data
    for name in inventory:
        verified(name)
    sources = [Path(__file__),ROOT/'scripts/multireference_rgbd_common.py',
        ROOT/'scripts/projective_depth_common.py',ROOT/'scripts/projective_depth_raw.py',
        ROOT/'scripts/check-opencv-rgbd.py',ROOT/'scripts/check-multireference-rgbd.py',
        ROOT/'assets/multireference-rgbd-v1/design.json',
        ROOT/'assets/multireference-rgbd-v1/numeric-convention.json',
        *sorted((ROOT/'integrations/projective-depth/src').rglob('*.rs')),
        ROOT/'integrations/projective-depth/Cargo.toml',ROOT/'integrations/projective-depth/Cargo.lock']
    write(output/'freeze.json',dict(schema_version=1,dataset=case,mode=mode,
        role='viewed development comparison; not held out',manifest_sha256=sha(body),
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources},
        binary_sha256=sha(binary.read_bytes()),inputs=inventory,camera=CAMERA,
        ground_truth_operational=False,pose_resets=0,root_translation_gate_m=.1,
        root_rotation_gate_rad=.1,reference_bracket_max_s=.02,updates=179,
        rgb_decoded=mode == 'phase_gated_rgbd',numpy_version=np.__version__))
    client = MultiClient(binary,CAMERA,mode)
    rows = []
    retained,keys = {},[]
    raw_equations = 0
    try:
        for frame in frames:
            image = Image.open(io.BytesIO(verified(frame['depth_file'])))
            require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'measured axial Z PNG')
            depth = np.asarray(image,dtype=float)/5000
            rgb = None
            gray = None
            if mode == 'phase_gated_rgbd':
                color = Image.open(io.BytesIO(verified(frame['rgb_file'])))
                require(color.size == (640,480) and color.mode == 'RGB','associated original RGB PNG')
                color = np.asarray(color,dtype=np.uint32)
                gray = ((77*color[:,:,0]+150*color[:,:,1]+29*color[:,:,2]) >> 8).astype(np.uint8)
                rgb = dict(timestamp=frame['rgb_timestamp'],gray=gray.ravel().tolist())
            update = client.observation(frame['depth_timestamp'],depth,rgb)
            rows.append(dict(source_index=frame['source_index'],update=update))
            source = raw_reference(depth,gray,CAMERA,np.eye(4),frame['rgb_timestamp']-frame['depth_timestamp'])
            references = [retained[t] for t in update['reference_timestamps']]
            for step in update['iterations']:
                compare_raw(step['equation'],raw_equation(references,source,pose(step['before']),
                    step['scale'],update['photo_enabled']))
                raw_equations += 1
            if update['accepted']:
                stamp = frame['depth_timestamp']
                source['root'] = pose(update['first_from_current'])
                if retained:
                    compare_raw(update['final_equation'],raw_equation(references,source,source['root'],
                        1,update['photo_enabled']))
                    raw_equations += 1
                if not keys:
                    keys = [stamp]
                elif update['keyframe_replaced']:
                    keys = [keys[0],*keys[1:][-2:],stamp]
                retained[stamp] = source
                retained = {t:retained[t] for t in {*keys,stamp}}
            if len(rows)%30 == 0:
                print(case,mode,len(rows),flush=True)
        client.finish()
    except BaseException:
        client.abort()
        raise
    write(output/'sensor-only.json',dict(frames=rows,ground_truth_parsed=False))
    checked = audit_sequence(rows,frames,mode)
    return frames,rows,verified,checked,raw_equations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--prototype-root',type=Path,default=ROOT/'artifacts/multireference-rgbd-dev/first')
    args = parser.parse_args()
    output = fresh(args.output)
    saved = {}
    for case in CASES:
        for mode in MODES:
            name = case+'-'+MODES[mode]
            folder = output/name
            folder.mkdir()
            saved[name] = operational(case,mode,args.binary.absolute(),folder)
    spec = importlib.util.spec_from_file_location('independent_reference',ROOT/'scripts/check-opencv-rgbd.py')
    reference = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reference)
    reference.controls()
    summaries = {}
    for name,(frames,rows,verified,checked,raw_equations) in saved.items():
        labels = np.loadtxt(io.BytesIO(verified('groundtruth.txt')),comments='#')
        require(labels.ndim == 2 and labels.shape[1] == 8 and len(labels) <= 30000
                and np.isfinite(labels).all() and np.all(np.diff(labels[:,0]) > 0)
                and np.all(np.linalg.norm(labels[:,4:],axis=1) > .5),'whole chronological numeric reference')
        origin = np.linalg.inv(reference.truth_at(labels,frames[0]['depth_timestamp']))
        proto = args.prototype_root/name/'sensor-only.json'
        prototype = json.loads(proto.read_bytes())['frames']
        errors,parity = [],[]
        for frame,row,other in zip(frames[1:],rows[1:],prototype,strict=True):
            u = row['update']
            compare = dict(source_index=frame['source_index'],same_acceptance=u['accepted'] == other['accepted'])
            if u['accepted']:
                estimate = pose(u['first_from_current'])
                truth = origin@reference.truth_at(labels,frame['depth_timestamp'])
                p = float(np.linalg.norm(estimate[:3,3]-truth[:3,3]))
                a = float(np.arccos(np.clip((np.trace(truth[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
                row.update(position_error_m=p,rotation_error_rad=a,accurate=p <= .1 and a <= .1)
                errors.append((p,a))
                if other['accepted']:
                    previous = np.asarray(other['root'])
                    compare.update(translation_disagreement_m=float(np.linalg.norm(estimate[:3,3]-previous[:3,3])),
                        rotation_disagreement_rad=float(np.arccos(np.clip((np.trace(previous[:3,:3].T@estimate[:3,:3])-1)/2,-1,1))))
            parity.append(compare)
        summary = dict(total_updates=179,accepted=len(errors),root_accurate=sum(r.get('accurate',False) for r in rows),
            lost=rows[-1]['update']['lost'],max_position_error_m=max((e[0] for e in errors),default=None),
            max_rotation_error_rad=max((e[1] for e in errors),default=None),independently_checked_normal_steps=checked,
            independently_reconstructed_raw_equations=raw_equations,
            same_prototype_acceptance=all(p['same_acceptance'] for p in parity),
            max_prototype_translation_disagreement_m=max((p.get('translation_disagreement_m',0) for p in parity)),
            max_prototype_rotation_disagreement_rad=max((p.get('rotation_disagreement_rad',0) for p in parity)))
        summary['complete_accuracy_pass'] = summary['root_accurate'] == 179 and not summary['lost']
        write(output/name/'results.json',dict(summary=summary,frames=rows,prototype_parity=parity,
            prototype_sensor_ledger_sha256=sha(proto.read_bytes()),
            audit_scope='all reported raw geometric and combined equations, solves/increments and reference/clock state'))
        summaries[name] = summary
        print(name,json.dumps(summary),flush=True)
    passed = all(s['complete_accuracy_pass'] for s in summaries.values())
    write(output/'comparison.json',dict(summaries=summaries,complete_accuracy_pass=passed,exit_code=0 if passed else 1))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
