#!/usr/bin/env python3
"""Exercise explicit-clock depth supervision against clean estimator sequences."""
import argparse
import gzip
import importlib.util
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image
from projective_depth_common import CAMERA, ROOT, Client, audit_iterations, audit_sequence, fresh, require, sha, write


def tick(client, now, event):
    return client.send(dict(kind='Tick',now=now,event=event))


def event(stamp, depth):
    return dict(kind='Depth',timestamp=stamp,depth_m=depth.ravel().tolist())


def check(result, status, last, estimate=None):
    require(result['status'] == status,'expected temporal status '+status)
    require(result['last_accepted_timestamp'] == last,'accepted acquisition clock cannot be renewed by ticks')
    if last is not None:
        require(abs(result['accepted_age_s']-(result['now']-last)) <= 1e-12,'age uses acquisition time')
    if status == 'fresh':
        require(result['first_from_last_accepted'] == estimate,'cached pose belongs to last accepted depth')
    else:
        require(result['first_from_last_accepted'] is None,'held/lost pose is unavailable')


def analytic(binary, base_binary, output):
    spec = importlib.util.spec_from_file_location('scene',ROOT/'scripts/check-projective-depth.py')
    scene = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scene)
    depth = scene.render(np.eye(4))
    guard,clean = Client(binary,scene.CAMERA),Client(base_binary,scene.CAMERA)
    rows = []
    try:
        first = tick(guard,1.,event(1.,depth))
        expected = clean.frame(1.,depth)
        require(first['measurement'] == expected,'initial estimator parity')
        pose = expected['first_from_current']
        check(first,'fresh',1.,pose);rows.append(first)
        failed = tick(guard,1.03,dict(kind='Failed'))
        check(failed,'hold',1.);rows.append(failed)
        old = tick(guard,1.04,event(1.02,depth))
        check(old,'hold',1.);rows.append(old)
        require(old['measurement'] is None and old['fault_epoch'] == 1.03
                and old['reason'] == 'before_fault_epoch','backlog is quarantined without refitting')
        recovered = tick(guard,1.061,event(1.06,depth))
        expected = clean.frame(1.06,depth)
        require(recovered['measurement'] == expected,'backlog did not change the later fit or map')
        check(recovered,'fresh',1.06,expected['first_from_current']);rows.append(recovered)
        audit_iterations(expected)
        silence = tick(guard,1.261,dict(kind='Idle'))
        check(silence,'lost',1.06);rows.append(silence)
        require(silence['measurement'] is None,'silence is not a fabricated depth observation')
        retry = tick(guard,1.27,event(1.27,depth))
        check(retry,'lost',1.06);rows.append(retry)
        require(retry['measurement'] is None,'terminal loss cannot reinitialize')
        guard.finish();clean.finish()
    except BaseException:
        guard.abort();clean.abort();raise
    write(output/'analytic-sequence.json',dict(frames=rows))
    config = json.dumps(dict(kind='Configure',camera=scene.CAMERA))+'\n'
    idle = lambda now:json.dumps(dict(kind='Tick',now=now,event=dict(kind='Idle')))+'\n'
    malformed = {
        'tick before configuration':idle(1.),
        'duplicate configuration':config+config,
        'unknown event field':config+json.dumps(dict(kind='Tick',now=1.,event=dict(kind='Idle',truth=[])))+'\n',
        'unknown failure field':config+json.dumps(dict(kind='Tick',now=1.,event=dict(kind='Failed',truth=[])))+'\n',
        'unknown depth field':config+json.dumps(dict(kind='Tick',now=1.,
                    event=dict(kind='Depth',timestamp=1.,depth_m=[],truth=[])))+'\n',
        'invalid receipt clock':config+idle(-1.),
        'duplicate receipt clock':config+idle(1.)+idle(1.),
        'regressing receipt clock':config+idle(1.)+idle(.9),
        'missing final newline':config.rstrip('\n'),
        'no tick':config,
        'oversized request':' '*(8*1024*1024+1)+'\n',
        'tick cap':config+''.join(idle(i*.01) for i in range(1001)),
        'frame cap':config+''.join(json.dumps(dict(kind='Tick',now=i*.01,
                    event=dict(kind='Depth',timestamp=i*.01,depth_m=[])))+'\n' for i in range(201)),
    }
    for name,body in malformed.items():
        result = subprocess.run([str(binary)],input=body,text=True,capture_output=True,timeout=30)
        require(result.returncode == 2 and bool(result.stderr),'protocol rejection: '+name)
    return dict(analytic_sequence_pass=True,clean_estimator_parity=True,rejected_protocol_cases=list(malformed))


def recorded(case, binary, base_binary, output):
    source = ROOT/'data'/case
    manifest_body = (source/'manifest.json').read_bytes()
    manifest = json.loads(manifest_body)
    frames = manifest['frames']
    require(len(frames) == 180 and all(b['depth_timestamp'] > a['depth_timestamp'] for a,b in zip(frames,frames[1:])),
            'complete monotonic viewed recording')
    inventory = {v['file']:v for v in manifest['files']}
    require(len(inventory) == len(manifest['files']),'unique source inventory')
    def verified(name):
        require(Path(name).name == name,'plain source name')
        path = source/'raw'/name
        require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 4*1024*1024,'bounded source')
        body = path.read_bytes()
        require(len(body) == inventory[name]['bytes'] and sha(body) == inventory[name]['sha256'],'source binding')
        return body
    for frame in frames:
        verified(frame['depth_file'])
    previous_path = ROOT/'assets/projective-depth-rust-v1'/case/'sensor-only.json.gz'
    previous_body = previous_path.read_bytes()
    previous = json.loads(gzip.decompress(previous_body))['frames']
    freeze_sources = [Path(__file__),ROOT/'scripts/projective_depth_common.py',
        ROOT/'assets/projective-depth-supervision-v1/design.json',
        ROOT/'integrations/projective-depth/Cargo.toml',ROOT/'integrations/projective-depth/Cargo.lock',
        *sorted((ROOT/'integrations/projective-depth/src').rglob('*.rs'))]
    write(output/'freeze.json',dict(dataset=case,data_role='viewed supervision regression',ground_truth_decoded=False,
        sources={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in freeze_sources},
        manifest_sha256=sha(manifest_body),prior_sensor_ledger_sha256=sha(previous_body),
        supervised_binary_sha256=sha(binary.read_bytes()),base_binary_sha256=sha(base_binary.read_bytes()),
        depth_inputs={f['depth_file']:inventory[f['depth_file']]['sha256'] for f in frames}))
    def depth(frame):
        image = Image.open(io.BytesIO(verified(frame['depth_file'])))
        require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original depth PNG')
        return np.asarray(image,dtype=float)/5000.
    guard = Client(binary,CAMERA)
    healthy = []
    steps = 0
    try:
        for frame,prior in zip(frames,previous):
            stamp = frame['depth_timestamp']
            require(prior['source_index'] == frame['source_index'],'fixed frame identity')
            result = tick(guard,stamp+.001,event(stamp,depth(frame)))
            require(result['measurement'] == prior['update'],'healthy supervisor preserves frozen estimator outcome')
            require(result['measurement']['accepted'],'known accepted geometric regression')
            estimate = prior['update']['first_from_current']
            check(result,'fresh',stamp,estimate)
            steps += audit_iterations(result['measurement'])
            healthy.append(dict(source_index=frame['source_index'],result=result))
            idle = tick(guard,stamp+.01,dict(kind='Idle'))
            check(idle,'fresh',stamp,estimate)
            require(idle['measurement'] is None,'idle cannot fabricate an update')
            healthy.append(dict(source_index=frame['source_index'],result=idle))
        last = frames[-1]['depth_timestamp']
        expired = tick(guard,last+.201,dict(kind='Idle'))
        check(expired,'lost',last)
        require(expired['measurement'] is None,'silence loss without acquisition')
        healthy.append(dict(source_index=None,result=expired))
        guard.finish()
    except BaseException:
        guard.abort();raise
    write(output/'healthy.json',dict(frames=healthy,matched_frozen_acquisitions=180))

    guard,clean = Client(binary,CAMERA),Client(base_binary,CAMERA)
    faulted,clean_rows,clean_stamps = [],[],[]
    last = None
    recovered_offsets = []
    try:
        for index,frame in enumerate(frames[:20]):
            stamp = frame['depth_timestamp']
            if index == 5:
                result = tick(guard,stamp+.001,event(stamp,np.array([])))
                check(result,'hold',last)
                require(result['measurement'] is not None and not result['measurement']['accepted'],
                        'malformed depth is an actual rejected estimator observation')
            elif index == 11:
                failed = tick(guard,stamp+.001,dict(kind='Failed'))
                check(failed,'hold',last)
                faulted.append(dict(source_index=frame['source_index'],result=failed))
                result = tick(guard,stamp+.002,event(stamp,depth(frame)))
                check(result,'hold',last)
                require(result['measurement'] is None and result['fault_epoch'] == stamp+.001
                        and result['reason'] == 'before_fault_epoch','pre-fault backlog cannot recover')
            else:
                raw = depth(frame)
                result = tick(guard,stamp+.001,event(stamp,raw))
                expected = clean.frame(stamp,raw)
                require(expected['accepted'],'clean filtered sequence accepts the regression frame')
                require(result['measurement'] == expected,'faults did not change model, reference or later fits')
                last = stamp
                check(result,'fresh',stamp,expected['first_from_current'])
                clean_rows.append(dict(source_index=frame['source_index'],update=expected))
                clean_stamps.append(stamp)
                if index in (6,12):
                    recovered_offsets.append(index)
            faulted.append(dict(source_index=frame['source_index'],result=result))
        expiry = tick(guard,last+.201,dict(kind='Idle'))
        check(expiry,'lost',last)
        faulted.append(dict(source_index=None,result=expiry))
        retry = frames[27]
        require(retry['depth_timestamp'] > last+.201,'real later acquisition follows silence expiry')
        result = tick(guard,retry['depth_timestamp']+.001,event(retry['depth_timestamp'],depth(retry)))
        check(result,'lost',last)
        require(result['measurement'] is None,'later genuine depth cannot silently reset the lost map')
        faulted.append(dict(source_index=retry['source_index'],result=result))
        clean_steps = audit_sequence(clean_rows,clean_stamps)
        guard.finish();clean.finish()
    except BaseException:
        guard.abort();clean.abort();raise
    write(output/'faulted.json',dict(frames=faulted,clean_filtered_estimator=clean_rows,
                                   recovered_offsets=recovered_offsets))
    return dict(healthy_acquisitions_matching_frozen_estimator=180,healthy_ticks=len(healthy),
        healthy_normal_steps=steps,fault_prefix_source_acquisitions=20,clean_filtered_acquisitions=len(clean_rows),
        faulted_ticks=len(faulted),clean_filtered_normal_steps=clean_steps,recovered_offsets=recovered_offsets,
        malformed_frame_excluded_from_fusion=True,pre_fault_backlog_excluded_from_estimation=True,
        expired_without_measurement=True,later_real_acquisition_cannot_revive=True,ground_truth_decoded=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--base-binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--analytic-only',action='store_true')
    args = parser.parse_args()
    output = fresh(args.output)
    binary,base_binary = args.binary.absolute(),args.base_binary.absolute()
    summaries = dict(analytic=analytic(binary,base_binary,output),recorded={})
    if not args.analytic_only:
        for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
            folder = output/case
            folder.mkdir()
            summaries['recorded'][case] = recorded(case,binary,base_binary,folder)
            print(case,json.dumps(summaries['recorded'][case]),flush=True)
    write(output/'summary.json',dict(pass_temporal_supervision=True,**summaries,
        physical_accuracy_claim=False,driving_integration=False,subjective_maturity_percent=20))


if __name__ == '__main__':
    main()
