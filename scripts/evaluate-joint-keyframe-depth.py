#!/usr/bin/env python3
"""Viewed fixed/joint keyframe comparison; all4 operational branches before label scoring."""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from projective_depth_common import CAMERA, ROOT, fresh, require, sha, write
from projective_depth_raw import pyramid
from joint_keyframe_depth import fit


def operational(case,mode,out):
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();m = json.loads(body)
    frames = m['frames']; inventory = {}
    for v in m['files']:
        name = v['file']
        require(Path(name).name == name and name not in inventory and 0 < v['bytes'] <= 4*1024*1024,'bounded unique source inventory')
        inventory[name] = v
    def verified(name):
        path = base/'raw'/name
        require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 4*1024*1024,'bounded real input')
        b = path.read_bytes();pin = inventory[name]
        require(len(b) == pin['bytes'] and sha(b) == pin['sha256'],'source digest '+name)
        return b
    for name in inventory:verified(name)
    require(len(frames) == 180 and all(b['depth_timestamp'] > a['depth_timestamp'] for a,b in zip(frames,frames[1:])),
            'all chronological original acquisitions')
    sources = [Path(__file__),ROOT/'scripts/joint_keyframe_depth.py',ROOT/'scripts/check-joint-keyframe-depth.py',ROOT/'scripts/multireference_rgbd_common.py',ROOT/'scripts/projective_depth_raw.py',ROOT/'scripts/projective_depth_common.py',
               ROOT/'scripts/check-opencv-rgbd.py',ROOT/'assets/joint-keyframe-depth-v1/design.json']
    write(out/'freeze.json',dict(mode=mode,manifest_sha256=sha(body),inputs=inventory,
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources},
        data_role='viewed development, not held out',ground_truth_operational=False,numpy_version=np.__version__))
    keys = [];latest = None;last = None;lost = False;rows = []
    for frame in frames:
        stamp = frame['depth_timestamp'];row = dict(source_index=frame['source_index'],timestamp=stamp,accepted=False)
        if last is not None and stamp-last > .2:lost = True
        try:
            if lost:raise ValueError('accepted pose expiry latched')
            image = Image.open(io.BytesIO(verified(frame['depth_file'])))
            require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'bounded axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000.
            current = dict(timestamp=stamp,surface=pyramid(depth,CAMERA),root=np.eye(4) if latest is None else latest['root'].copy())
            if latest is None:
                require(np.sum(np.sum(current['surface'][4]['normals']**2,axis=-1) > .25) >= 200,'initial support')
                keys = [current];latest = current;last = stamp
                row.update(accepted=True,root=np.eye(4).tolist(),nodes=[stamp],trace=[])
            else:
                nodes = list(keys)
                if latest not in nodes:nodes.append(latest)
                nodes.append(current);require(len(nodes) <= 5,'bounded window')
                row['nodes'] = [n['timestamp'] for n in nodes]
                row['trace'] = []
                roots,trace,final = fit(nodes,mode,row['trace'])
                relative = np.linalg.inv(latest['root'])@roots[-1]
                if np.linalg.norm(relative[:3,3]) > .5:raise ValueError('accepted displacement')
                delta = np.linalg.inv(roots[nodes.index(keys[-1])])@roots[-1]
                replace = np.linalg.norm(delta[:3,3]) > .1 or np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)) > .1
                # Atomic commit: pose arrays were detached during all fitting/admission.
                for node,root in zip(nodes,roots):node['root'] = root
                if replace:keys = [keys[0],*keys[1:][-1:],current]
                latest = current;last = stamp
                row.update(accepted=True,root=roots[-1].tolist(),nodes=[n['timestamp'] for n in nodes],trace=trace,
                    final_equation=final,keyframe_replaced=bool(replace),revised_roots=[p.tolist() for p in roots])
        except ValueError as error:row['rejection'] = str(error)
        row.update(lost=lost,last_accepted_timestamp=last)
        rows.append(row)
        if len(rows)%30 == 0:print(case,mode,len(rows),flush=True)
    write(out/'sensor-only.json',dict(frames=rows,ground_truth_parsed=False))
    return frames,rows,verified


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = fresh(args.output)
    saved = {}
    for case in ['tum-fr1-room-temporal','tum-fr1-desk2-independent']:
        for mode in ['fixed','joint']:
            name = case+'-'+mode;out = output/name;out.mkdir();saved[name] = operational(case,mode,out)
    audit_spec = importlib.util.spec_from_file_location('joint_raw_audit',ROOT/'scripts/check-joint-keyframe-depth.py')
    audit = importlib.util.module_from_spec(audit_spec);audit_spec.loader.exec_module(audit)
    integrity = {}
    for name,(frames,rows,verified) in saved.items():
        integrity[name] = audit.recorded(rows,frames,verified,name.rsplit('-',1)[1],CAMERA)
        write(output/name/'integrity.json',integrity[name])
        print(name,'raw/state integrity',json.dumps(integrity[name]),flush=True)
    spec = importlib.util.spec_from_file_location('physical_reference',ROOT/'scripts/check-opencv-rgbd.py')
    ref = importlib.util.module_from_spec(spec);spec.loader.exec_module(ref);ref.controls();summaries = {}
    for name,(frames,rows,verified) in saved.items():
        labels = np.loadtxt(io.BytesIO(verified('groundtruth.txt')),comments='#')
        require(labels.ndim == 2 and labels.shape[1] == 8 and np.isfinite(labels).all() and np.all(np.diff(labels[:,0]) > 0),'whole chronological reference')
        origin = np.linalg.inv(ref.truth_at(labels,frames[0]['depth_timestamp']));errors = []
        for frame,row in zip(frames[1:],rows[1:]):
            if not row['accepted']:continue
            estimate = np.asarray(row['root']);truth = origin@ref.truth_at(labels,frame['depth_timestamp'])
            p = float(np.linalg.norm(estimate[:3,3]-truth[:3,3]));a = float(np.arccos(np.clip((np.trace(truth[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
            row.update(position_error_m=p,rotation_error_rad=a,accurate=p <= .1 and a <= .1);errors.append((p,a))
        summary = dict(total_updates=179,accepted=len(errors),root_accurate=sum(r.get('accurate',False) for r in rows),
            lost=rows[-1]['lost'],max_position_error_m=max((x[0] for x in errors),default=None),
            max_rotation_error_rad=max((x[1] for x in errors),default=None),integrity=integrity[name])
        summaries[name] = summary;write(output/name/'results.json',dict(summary=summary,frames=rows));print(name,json.dumps(summary),flush=True)
    passed = all(s['root_accurate'] == 179 and not s['lost'] for s in summaries.values())
    write(output/'comparison.json',dict(summaries=summaries,complete_accuracy_pass=passed,exit_code=0 if passed else 1))
    return 0 if passed else 1


if __name__ == '__main__':sys.exit(main())
