#!/usr/bin/env python3
"""Frozen immutable-depth-map trial; score references after all native tracking."""
import argparse
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from projective_depth_common import CAMERA,ROOT,Client,fresh,pose,require,sha,write

DESIGN_SHA='6d8e07da2129516cf8b00d401560d09c7594f28531ab41c9af5ba95b3d58c41a'
CASES=('tum-fr1-room-temporal','tum-fr1-desk2-independent','tum-fr1-rpy-heldout')


def metadata(case):
    base=ROOT/'data'/case
    manifest=json.loads((base/'manifest.json').read_bytes())
    require(len(manifest['frames'])==180,'all180 original acquisitions')
    inventory={}
    for pin in manifest['files']:
        name=pin['file']
        require(Path(name).name==name and name not in inventory
                and 0<pin['bytes']<=4*1024*1024,'bounded unique source')
        inventory[name]=pin
    require(all(b['depth_timestamp']>a['depth_timestamp'] for a,b in zip(manifest['frames'],manifest['frames'][1:])),
            'chronological acquisitions')
    return base,manifest,inventory


def verified(base,inventory,name):
    path=base/'raw'/name
    require(path.is_file() and not path.is_symlink() and path.stat().st_size<=4*1024*1024,'bounded regular input')
    body=path.read_bytes();pin=inventory[name]
    require(len(body)==pin['bytes'] and sha(body)==pin['sha256'],'original input pin')
    return body


def decode(base,inventory,frame):
    image=Image.open(io.BytesIO(verified(base,inventory,frame['depth_file'])))
    require(image.size==(640,480) and image.mode in ('I;16','I;16B','I'),'original registered depth')
    return np.asarray(image,dtype=float)/5000.


def score(case,rows,base,manifest,inventory):
    spec=importlib.util.spec_from_file_location('fixed_map_reference',ROOT/'scripts/check-opencv-rgbd.py')
    oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)
    oracle.controls()
    labels=np.loadtxt(io.BytesIO(verified(base,inventory,'groundtruth.txt')),comments='#')
    require(labels.ndim==2 and labels.shape[1]==8 and len(labels)<=30000 and np.isfinite(labels).all()
            and np.all(np.diff(labels[:,0])>0) and np.all(np.linalg.norm(labels[:,4:],axis=1)>.5),'whole numeric mocap')
    frames=manifest['frames'];origin=None
    last=None;map_time=None;lost=False;accepted=accurate=0;errors=[]
    for index,(frame,row) in enumerate(zip(frames,rows)):
        update=row['update'];stamp=frame['depth_timestamp']
        require(update['timestamp']==stamp and row['source_index']==frame['source_index'],'acquisition identity')
        if last is not None:lost|=stamp-last>.2
        require(update['lost']==lost and not(lost and update['accepted']),'expiry cannot be hidden')
        if update['accepted']:
            if map_time is None:
                map_time=stamp
                origin=np.linalg.inv(oracle.truth_at(labels,map_time))
            last=stamp
            estimate=pose(update['first_from_current'])
            reference=origin@oracle.truth_at(labels,stamp)
            position=float(np.linalg.norm(estimate[:3,3]-reference[:3,3]))
            rotation=float(np.arccos(np.clip((np.trace(reference[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
            good=position<=.1 and rotation<=.1
            row.update(position_error_m=position,rotation_error_rad=rotation,accurate=good)
            if index:
                accepted+=1;accurate+=good;errors.append((position,rotation))
            else:np.testing.assert_array_equal(estimate,np.eye(4))
        else:
            require(update['first_from_current'] is None and update['final_equation'] is None,'no rejected pose')
        require(update['last_accepted_timestamp']==last and update['map_timestamp']==map_time
                and update['initialized']==(map_time is not None),'immutable map/accepted clock')
    return dict(dataset=case,total_updates=179,accepted=accepted,root_accurate=accurate,lost=lost,
                max_position_error_m=max((a for a,b in errors),default=None),
                max_rotation_error_rad=max((b for a,b in errors),default=None),
                complete_accuracy_pass=accurate==179 and not lost,pose_resets=0,map_replacements=0)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--design',type=Path,default=ROOT/'artifacts/projective-fixed-map-v1/design.json')
    args=parser.parse_args()
    require(sha(args.design.read_bytes())==DESIGN_SHA,'fixed pretrial design')
    output=fresh(args.output)
    sources=[Path(__file__),ROOT/'scripts/projective_depth_common.py',ROOT/'scripts/projective_depth_raw.py',
             ROOT/'scripts/check-opencv-rgbd.py',ROOT/'scripts/projective_information_raw.py',
             ROOT/'scripts/check-projective-fixed-map.py',
             *sorted((ROOT/'integrations/projective-depth/src').rglob('*.rs')),
             ROOT/'integrations/projective-depth/Cargo.toml',ROOT/'integrations/projective-depth/Cargo.lock']
    source_pins={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources}
    binary_sha=sha(args.binary.read_bytes())
    records=[]
    for case in CASES:
        directory=fresh(output/case)
        base,manifest,inventory=metadata(case)
        for name in inventory:verified(base,inventory,name) # Reference bytes only.
        freeze=dict(schema_version=1,dataset=case,data_role='previously viewed fixed-map regression',
                    design_sha256=DESIGN_SHA,source_sha256=source_pins,binary_sha256=binary_sha,camera=CAMERA,
                    manifest_sha256=sha((base/'manifest.json').read_bytes()),inputs=inventory,
                    ground_truth_operational=False,ground_truth_parsed=False,
                    selected_frames=180,total_updates=179,root_position_gate_m=.1,root_rotation_gate_rad=.1,
                    pose_resets=0,map_replacements=0)
        write(directory/'freeze.json',freeze)
        client=Client(args.binary,CAMERA);rows=[]
        try:
            for frame in manifest['frames']:
                update=client.frame(frame['depth_timestamp'],decode(base,inventory,frame))
                rows.append(dict(source_index=frame['source_index'],update=update))
            client.finish()
        except BaseException:
            client.abort();raise
        write(directory/'sensor-only.json',dict(ground_truth_parsed=False,frames=rows))
        records.append((case,directory,rows,base,manifest,inventory))
        print(case,'native acquisitions complete; numeric reference not parsed',flush=True)
    write(output/'before-reference-parsing.json',dict(ground_truth_parsed=False,
          sensor_report_sha256={case:sha((directory/'sensor-only.json').read_bytes())
                                for case,directory,*_ in records}))
    summaries=[]
    for case,directory,rows,base,manifest,inventory in records:
        summary=score(case,rows,base,manifest,inventory)
        write(directory/'results.json',dict(schema_version=1,summary=summary,frames=rows))
        summaries.append(summary);print(json.dumps(summary),flush=True)
    require(sha(args.binary.read_bytes())==binary_sha and
            all(sha((ROOT/p).read_bytes())==pin for p,pin in source_pins.items()),'frozen sources/binary unchanged')
    passed=all(s['complete_accuracy_pass'] for s in summaries)
    write(output/'summary.json',dict(schema_version=1,cases=summaries,complete_accuracy_pass=passed,
                                   goal_30_percent_complete=False))
    return 0 if passed else 1


if __name__=='__main__':
    raise SystemExit(main())
