#!/usr/bin/env python3
"""Offline raw-pair objective probes on viewed recordings, never runtime truth input."""
import argparse
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
from projective_depth_common import CAMERA, ROOT, fresh, require, sha, write
from projective_depth_raw import normal_equation, pyramid

CASES=('tum-fr1-room-temporal','tum-fr1-desk2-independent','tum-fr1-rpy-heldout')
POLICIES=('target_plane','symmetric_planes','symmetric_midpoint')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=fresh(args.output)
    sources=(Path(__file__),ROOT/'scripts/projective_depth_common.py',
             ROOT/'scripts/projective_depth_raw.py',ROOT/'scripts/check-opencv-rgbd.py')
    source_pins={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources}
    write(out/'freeze.json',dict(source_sha256=source_pins,cases=CASES,
        selected_frame_offsets=list(range(1,180,12)),scales=[4,2,1],policies=POLICIES,
        data_role='viewed offline diagnosis',ground_truth_operational=False,
        reference_pose_probe=True,no_trajectory_accuracy_or_sensor_calibration_claim=True))
    spec=importlib.util.spec_from_file_location('reference',ROOT/'scripts/check-opencv-rgbd.py')
    reference=importlib.util.module_from_spec(spec);spec.loader.exec_module(reference);reference.controls()
    results={}
    for case in CASES:
        base=ROOT/'data'/case;manifest_body=(base/'manifest.json').read_bytes();manifest=json.loads(manifest_body)
        inventory={p['file']:p for p in manifest['files']}
        def read(name):
            p=base/'raw'/name;require(p.is_file() and not p.is_symlink(),'regular qualified input')
            b=p.read_bytes();pin=inventory[name]
            require(len(b)==pin['bytes'] and sha(b)==pin['sha256'],'qualified original bytes')
            return b
        labels=np.loadtxt(io.BytesIO(read('groundtruth.txt')),comments='#')
        require(labels.ndim==2 and labels.shape[1]==8 and np.isfinite(labels).all()
                and np.all(np.diff(labels[:,0])>0),'chronological finite offline reference')
        rows=[]
        for i in range(1,180,12):
            a,b=manifest['frames'][i-1:i+1]
            def depth(f):
                im=Image.open(io.BytesIO(read(f['depth_file'])))
                require(im.size==(640,480) and im.mode in ('I;16','I;16B','I'),'original axial depth')
                return np.asarray(im,dtype=float)/5000.
            old,new=pyramid(depth(a),CAMERA),pyramid(depth(b),CAMERA)
            expected=np.linalg.inv(reference.truth_at(labels,a['depth_timestamp']))@reference.truth_at(labels,b['depth_timestamp'])
            row=dict(source_index=b['source_index'],timestamp=b['depth_timestamp'],directions={})
            for direction,target,source,transform in (('forward',old,new,expected),('reverse',new,old,np.linalg.inv(expected))):
                equations={}
                for scale in (4,2,1):
                    for policy in POLICIES:
                        e=normal_equation(target[scale],source[scale],transform,scale,residual_policy=policy)
                        eig=np.linalg.eigvalsh(e['h']);condition=None if eig[0]<=0 else float(eig[-1]/eig[0])
                        correction=None
                        if condition is not None and condition<=1e6 and e['count']>=200:
                            correction=np.linalg.solve(e['h'],-e['g']).tolist()
                        equations[str(scale)+'/'+policy]=dict(count=e['count'],rms_m=e['rms_m'],
                            physical_condition=condition,correction_at_reference_pose=correction)
                row['directions'][direction]=equations
            rows.append(row)
        summary={}
        for policy in POLICIES:
            values=[r['directions'][d]['1/'+policy]['correction_at_reference_pose'] for r in rows for d in ('forward','reverse')]
            valid=np.array([v for v in values if v is not None])
            require(len(valid)>0,'nonempty diagnosis')
            summary[policy]=dict(probes=len(values),observable=len(valid),
                median_position_correction_m=float(np.median(np.linalg.norm(valid[:,:3],axis=1))),
                median_rotation_correction_rad=float(np.median(np.linalg.norm(valid[:,3:],axis=1))),
                maximum_rotation_correction_rad=float(np.max(np.linalg.norm(valid[:,3:],axis=1))))
        write(out/(case+'.json'),dict(manifest_sha256=sha(manifest_body),summary=summary,probes=rows,
            ground_truth_used_only_for_offline_objective_probe=True))
        results[case]=summary;print(case,json.dumps(summary),flush=True)
    require(all(sha((ROOT/p).read_bytes())==s for p,s in source_pins.items()),'unchanged diagnostic sources')
    write(out/'summary.json',dict(cases=results,scope='raw consecutive-pair local objective bias; no fused-model or complete runtime-trajectory claim'))


if __name__=='__main__':main()
