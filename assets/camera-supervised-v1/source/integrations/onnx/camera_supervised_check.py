#!/usr/bin/env python3
"""Audit bounded asynchronous camera work, original guard outputs and real clocks."""
import argparse
import ast
import copy
from decimal import Decimal
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

import camera_latest_check as latest
nano = latest.nano
ROOT, HERE = latest.ROOT, latest.HERE
require, digest, load, write, fresh, native = latest.require, latest.digest, latest.load, latest.write, latest.fresh, latest.native
freshness, tracking = latest.freshness, latest.tracking
NEW = ['integrations/onnx/src/camera_supervised.rs', 'integrations/onnx/src/bin/rustdriving-camera-supervised.rs', 'integrations/onnx/profile-supervised.json']
ORDER = ['latest-first', 'supervised-first', 'supervised-repeat', 'latest-repeat',
         'fault-latest-first', 'fault-supervised-first', 'fault-supervised-repeat', 'fault-latest-repeat']


def expected():
    return dict(nano.expected('runtime'), **{name:digest(ROOT/name) for name in NEW})


def slots_for(source):
    origin = float(source['events'][0]['timestamp_s'])
    return [dict(event_index=i, source_timestamp_text=e['timestamp_s'], source_timestamp_s=float(e['timestamp_s']),
        release_offset_s=float(e['timestamp_s'])-origin, source_image_sha256=e['input'].get('sha256')) for i,e in enumerate(source['events'])]


def precheck(previous, acquired, now):
    accepted = previous['last_accepted_acquired_s'] if previous else None
    barrier = previous['fault_epoch_s'] if previous else None
    if accepted is not None and (barrier is None or accepted > barrier) and now > accepted+.2:
        barrier = accepted+.2
    return ('before_fault' if barrier is not None and acquired <= barrier else
            'out_of_order' if accepted is not None and acquired-accepted < 1e-6 else
            'stale_sample' if now > acquired+.2 else 'accepted')


def audit(report, source, inspection, references=None):
    require(report['schema_version'] == 1 and report['algorithm'] == 'yolox_nano_supervised_replay_v1', 'distinct supervised producer')
    require(report['compiled_sources'] == expected() and report['model_sha256'] == nano.MODEL_SHA, 'complete compiled source and model binding')
    require(report['policy'] == freshness.POLICY and report['control_permission'] is False, 'original recovery and age policy')
    require((report['poll_period_s'],report['in_flight_capacity'],report['pending_image_capacity']) == (.01,1,1), 'bounded work and declared poll target')
    require(report['clock'] == 'std::time::Instant_supervised_replay', 'measured clock identity')
    require(inspection['algorithm'] == 'yolox_nano_supervised_schedule_validation_v1' and inspection['compiled_sources'] == expected()
            and inspection['policy'] == freshness.POLICY and inspection['control_permission'] is False
            and inspection['model_read'] is False and inspection['inference_run'] is False, 'model-free complete preflight')
    require(report['source_sequence_sha256'] == inspection['source_sequence_sha256'], 'source hash binding')
    for key in ('camera_id','width','height'):
        require(report[key] == inspection[key] == source[key], 'fixed camera geometry and identity')
    require(report['config'] == inspection['config'], 'fixed tracking calibration')
    slots = slots_for(source)
    require(report['source_slots'] == inspection['slots'] == slots, 'original acquisition inventory without renewal')
    counts = dict(source_images=sum(s['source_image_sha256'] is not None for s in slots),superseded_images=0,excluded_images=0,
        started_images=0,completed_images=0,worker_errors=0,accepted_result_images=0,available_result_images=0,inferred_but_late_images=0,
        delivered_source_barriers=0,source_barriers_observed_while_worker_active=0,periodic_polls_while_worker_active=0,
        max_observed_poll_gap_s=0.,max_source_barrier_delay_s=None,final_ready=False,all_source_results_available=False)
    require(inspection['images_read'] == counts['source_images'], 'preflight reads all original sources')
    next_source=0; pending=None; active=None; dispatch=None; previous=None; last_time=None; last_guard=None; last_poll=None
    expect_guard=None; expect_post=False; frames={}; events=[]; outputs=[]; finished={}; dispositions={}; trigger=None
    for row in report['trace']:
        now=row['observed_s']; change=row['change']; kind=change['kind']; index=change.get('event_index')
        require(type(now) in (int,float) and math.isfinite(now) and 0 <= now <= 10 and (last_time is None or now >= last_time), 'observed monotonic bounded host clock')
        last_time=now
        expected_active = index if kind == 'started' else active
        require(row['active_source_index'] == expected_active, 'single in-flight work identity')
        if expect_guard is not None:
            require(kind == 'guard' or (kind == 'superseded' and expect_guard[0] == 'source_barrier'), 'no event may bypass required guard delivery')
        if kind == 'source':
            require(index == next_source and now >= slots[index]['release_offset_s'], 'every source released once in order, never future')
            next_source += 1; trigger=index
            data=source['events'][index]['input']
            if data['kind'] != 'image':
                expect_guard=('source_barrier', index, dict(kind='failed' if data['kind']=='failed' else 'tick'))
            if pending is None and data['kind']=='image':pending=index
            # Replacements are explicit subsequent rows, never hidden drops.
        elif kind == 'superseded':
            require(pending is not None and index == pending and index not in dispositions, 'only the single waiting image can be discarded')
            require(trigger is not None and trigger != index and now >= slots[trigger]['release_offset_s'], 'discard caused by a released source')
            image=source['events'][trigger]['input']['kind']=='image'
            require(change['reason'] == ('newer_released_image' if image else 'source_barrier'), 'explicit replacement or delivered barrier')
            dispositions[index]='superseded'; counts['superseded_images']+=1
            pending=trigger if image else None
        elif kind in ('started','excluded'):
            require(active is None and pending == index and index not in dispositions, 'start only the waiting image and never overlap inference')
            require(next_source == len(slots) or slots[next_source]['release_offset_s'] > now, 'due sources precede new inference')
            decision=precheck(previous,slots[index]['release_offset_s'],now)
            require(change['precheck_decision'] == decision and (kind=='started') == (decision=='accepted'), 'independent original admission probe')
            pending=None
            if kind=='started': active=index; dispatch=now; counts['started_images']+=1
            else:
                dispositions[index]='excluded'; counts['excluded_images']+=1
                expect_guard=('excluded_image', index, dict(kind='sample',acquired_s=slots[index]['release_offset_s'],image_sha256=slots[index]['source_image_sha256'],detections=[]))
        elif kind == 'finished':
            require(active == index and index not in finished, 'complete only current inference exactly once')
            require(next_source == len(slots) or slots[next_source]['release_offset_s'] > now, 'due faults precede completed result delivery')
            start,finish=change['worker_started_s'],change['worker_finished_s']
            require(all(type(t) in (int,float) and math.isfinite(t) for t in (start,finish)) and dispatch <= start <= finish <= now, 'actual worker clocks inside dispatch/delivery interval')
            analysis,error=change['analysis'],change['error']
            require((analysis is None) == (error is not None), 'result is either inference or a visible failure')
            counts['completed_images']+=1; counts['worker_errors']+=error is not None
            if analysis is not None:
                original=slots[index]['source_image_sha256']
                golden=next(load(ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json') for i in range(8)
                            if load(ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json')['source_image_sha256']==original)
                serial=None if references is None else references[original]
                nano.check_image(analysis,golden,serial)
                require(finish-start >= analysis['execution']['analysis_wall_s'], 'worker clock covers the entire original detector')
                f32=lambda v:struct.unpack('<f',struct.pack('<f',v))[0]
                data=dict(kind='sample',acquired_s=slots[index]['release_offset_s'],image_sha256=original,
                    detections=[dict(class_index=d['class_index'],confidence=f32(d['confidence']),bbox=list(map(f32,d['bbox']))) for d in analysis['result']['detections']])
            else:
                require(type(error) is str and error, 'visible worker error')
                data=dict(kind='failed')
            expect_guard=('worker_result',index,data); frames[index]=analysis; finished[index]=(start,finish)
            dispositions[index]='completed'; active=None; dispatch=None
        elif kind == 'guard':
            reason=change['reason']; event=change['event']; output=change['output']; data=event['input']
            require(event['now_s'] == now and (last_guard is None or now-last_guard >= 1e-6), 'guard clocks are exact observed host checkpoints')
            last_guard=now
            # Completion is recorded while active; its result is guarded after the worker is drained.
            require(row['active_source_index'] == active, 'guard never runs inside the inference worker')
            if expect_guard is not None:
                wanted,source_index,payload=expect_guard
                require((reason,index,data)==(wanted,source_index,payload), 'guard receives the exact original source or computed result')
                expect_guard=None
                if reason=='worker_result':expect_post=True
            elif reason=='post_tracking_poll':
                require(expect_post and index is None and data==dict(kind='tick'), 'post-tracking check follows each result')
                expect_post=False; counts['available_result_images']+=output['ready']
            else:
                require(not expect_post and reason in ('periodic_poll','final_poll') and index is None and data==dict(kind='tick'), 'only declared watchdog ticks without an input')
            if reason=='source_barrier':
                counts['delivered_source_barriers']+=1
                counts['source_barriers_observed_while_worker_active']+=active is not None
                delay=now-slots[index]['release_offset_s']
                old=counts['max_source_barrier_delay_s']; counts['max_source_barrier_delay_s']=delay if old is None else max(old,delay)
            if reason=='worker_result':
                counts['accepted_result_images']+=output['decision']=='accepted'
                counts['inferred_but_late_images']+=data['kind']=='sample' and now > data.get('acquired_s',now)+.2
            if reason=='periodic_poll':
                counts['periodic_polls_while_worker_active']+=active is not None
                if last_poll is not None:counts['max_observed_poll_gap_s']=max(counts['max_observed_poll_gap_s'],now-last_poll)
                last_poll=now
            if reason=='final_poll':
                end=slots[-1]['release_offset_s']; accepted=output['last_accepted_acquired_s']
                require(next_source==len(slots) and active is None and pending is None and now > max(end, end if accepted is None else accepted+.2)+.01, 'monitor until all accepted output expires after input drain')
            event=copy.deepcopy(event)
            if data['kind']=='sample':event['input']['kind']='detections'
            events.append(event); outputs.append(output); previous=output
        else: raise ValueError('unexpected trace event')
    require(next_source==len(slots) and pending is None and active is None and expect_guard is None and not expect_post, 'all sources and jobs drained')
    require(report['trace'][-1]['change']['kind']=='guard' and report['trace'][-1]['change']['reason']=='final_poll', 'complete monitored ending')
    require(set(dispositions)=={i for i,s in enumerate(slots) if s['source_image_sha256'] is not None}, 'every image has exactly one final disposition')
    counts['final_ready']=previous['ready']; counts['all_source_results_available']=counts['source_images']>0 and counts['available_result_images']==counts['source_images']
    require(counts==report['summary'] and not counts['final_ready'], 'independent full-denominator summary and final unavailability')
    require(report['model_loads']==int(counts['source_images']>0) and report['preflight_image_reads']==counts['source_images']
            and report['inference_image_reads']==counts['completed_images'], 'actual reads and model inventory')
    known=sum(a['execution']['model_forwards'] for a in frames.values() if a is not None)
    require(report['successful_model_forwards']==known and report['model_forwards']==(known if not counts['worker_errors'] else None), 'never invent a complete forward count on worker failure')
    for key in ('manifest_and_preflight_wall_s','model_load_wall_s','startup_before_schedule_wall_s','schedule_wall_s'):
        require(type(report[key]) in (int,float) and math.isfinite(report[key]) and report[key]>=0, 'finite stage clock')
    require(report['startup_before_schedule_wall_s'] >= report['manifest_and_preflight_wall_s']+report['model_load_wall_s']
            and report['schedule_wall_s']>=last_time, 'actual startup and schedule coverage')
    manifest=dict(schema_version=1,camera_id=source['camera_id'],width=source['width'],height=source['height'],config=report['config'],policy=freshness.POLICY,events=events)
    freshness.check_contract(manifest,events,outputs,tracking)
    return manifest


def common_controls(args):
    out=fresh(args.output)
    inputs=fresh(out/'inputs')
    for i in range(2):
        shutil.copyfile(ROOT/f'assets/camera-sequence-v1/controls/inputs/{i}.png',inputs/f'{i}.png')
    sequence=dict(schema_version=1,camera_id='latency-controls',width=8,height=6,events=[
        dict(timestamp_s=str(i*.1),input=dict(kind='image',path=f'inputs/{i}.png',sha256=digest(inputs/f'{i}.png')))
        for i in range(2)])
    valid=out/'valid.json'
    write(valid,sequence)
    inspected,_=native(args.binary,['--inspect-sequence',valid],out/'inspection.json')
    require(inspected['images_read']==2 and not inspected['model_read'] and not inspected['inference_run'],'model-free image preflight')
    require([s['release_offset_s'] for s in inspected['slots']]==[0.,.1],'source schedule preserved')
    invalid=inputs/'invalid.png'
    invalid.write_bytes(b'not an image\n')
    link=inputs/'link.png'
    link.symlink_to('0.png')
    fifo=inputs/'fifo.png'
    os.mkfifo(fifo)
    operations={
        'clock-reversal':lambda s:s['events'][1].__setitem__('timestamp_s','-1'),
        'clock-duplicate':lambda s:s['events'][1].__setitem__('timestamp_s','0'),
        'clock-too-close':lambda s:s['events'][1].__setitem__('timestamp_s','0.0000001'),
        'clock-nonfinite':lambda s:s['events'][1].__setitem__('timestamp_s','NaN'),
        'clock-type':lambda s:s['events'][1].__setitem__('timestamp_s',.1),
        'clock-text-budget':lambda s:s['events'][1].__setitem__('timestamp_s','0'*65),
        'schedule-span':lambda s:s['events'][1].__setitem__('timestamp_s','2.000001'),
        'reused-image':lambda s:s['events'][1].__setitem__('input',copy.deepcopy(s['events'][0]['input'])),
        'bad-hash':lambda s:s['events'][1]['input'].__setitem__('sha256','0'*64),
        'bad-dimensions':lambda s:s.__setitem__('width',9),
        'missing-late-image':lambda s:s['events'][1]['input'].__setitem__('path','inputs/missing.png'),
        'corrupt-late-image':lambda s:s['events'][1]['input'].update(path='inputs/invalid.png',sha256=digest(invalid)),
        'image-symlink':lambda s:s['events'][1]['input'].__setitem__('path','inputs/link.png'),
        'image-fifo':lambda s:s['events'][1]['input'].__setitem__('path','inputs/fifo.png'),
        'empty-sequence':lambda s:s.__setitem__('events',[]),
        'event-budget':lambda s:s.__setitem__('events',s['events']*17),
        'bad-config':lambda s:s.__setitem__('config',dict(max_tracks=301)),
        'unknown-input':lambda s:s['events'][1]['input'].__setitem__('unexpected',True),
        'unknown-root':lambda s:s.__setitem__('unexpected',True),
    }
    rejected=[]
    for name,change in operations.items():
        value=copy.deepcopy(sequence)
        change(value)
        path=out/(name+'-input.json')
        write(path,value)
        error=native(args.binary,['--model','missing-model','--sequence',path],out/(name+'-unexpected.json'),2)
        require('load model:' not in error['stderr'],'all source failures before model access')
        rejected.append(dict(name=name,**error))
    oversized=out/'oversized.json'
    with oversized.open('xb') as stream:
        stream.truncate(512*1024+1)
    rejected.append(dict(name='sequence-budget',**native(args.binary,['--inspect-sequence',oversized],out/'oversized-unexpected.json',2,'512 KiB')))
    model=inputs/'invalid.onnx'
    model.write_bytes(b'bad model')
    rejected.append(dict(name='model-size',**native(args.binary,['--model',model,'--sequence',valid],out/'model-unexpected.json',2,'load model:')))
    no_images=dict(schema_version=1,camera_id='no-images',width=8,height=6,events=[
        dict(timestamp_s='0',input=dict(kind='missing')),dict(timestamp_s='0.00001',input=dict(kind='failed'))])
    path=out/'no-images.json'
    write(path,no_images)
    no_model,_=native(args.binary,['--model','missing-model','--sequence',path],out/'no-images-result.json')
    require(no_model['model_loads']==0 and no_model['model_forwards']==0 and no_model['inference_image_reads']==0,'non-image trace never opens model')
    require(no_model['summary']['all_source_results_available'] is False,'empty trace has no availability success')
    require(not any(e['change']['kind']=='finished' for e in no_model['trace']),'no synthetic detections')
    existing=out/'existing.txt'
    existing.write_bytes(b'preserve\n')
    dangling=out/'existing-link.json'
    dangling.symlink_to('missing.json')
    output_rejections=[]
    for target in (existing,dangling):
        process=subprocess.run([str(args.binary),'--inspect-sequence','missing-source','--output',str(target)],capture_output=True,text=True,timeout=30)
        require(process.returncode==2 and 'fresh output' in process.stderr,'output checked before sources')
        output_rejections.append(dict(name=target.name,exit=process.returncode,stderr=process.stderr))
    require(existing.read_bytes()==b'preserve\n' and dangling.is_symlink(),'existing output changed')
    receipt,_=native(args.binary,['--executable-receipt'],out/'executable.json')
    require(receipt['executable_sha256']==digest(args.binary),'executable receipt')
    for name,pin in receipt['compiled_sources'].items():
        require(digest(ROOT/name)==pin,'compiled source receipt')
    write(out/'special-files.json',dict(symlinks={str(p.relative_to(out)):os.readlink(p) for p in (link,dangling)},
        fifo=str(fifo.relative_to(out)),sparse_files={oversized.name:oversized.stat().st_size}))
    for p in (link,dangling,fifo,oversized):
        p.unlink()
    write(out/'summary.json',dict(pass_checks=True,rejected_inputs=rejected,rejected_before_model_loading=20,
        output_rejections=output_rejections,model_free_non_image_trace=True,
        executable_sha256=digest(args.binary),checker_sha256=digest(__file__)))
    print(json.dumps(dict(pass_checks=True,rejected_inputs=len(rejected),output_protections=2)),flush=True)


def controls(args):
    common_controls(args)
    out=args.output
    no_images=out/'no-images.json'
    inspected,_=native(args.binary,['--inspect-sequence',no_images],out/'no-images-inspection.json')
    manifest=audit(load(out/'no-images-result.json'),load(no_images),inspected)
    write(out/'no-images-guard-input.json',manifest)
    result,_=native(args.guard,['--sequence',out/'no-images-guard-input.json'],out/'no-images-guard-result.json')
    require(result['results']==[e['change']['output'] for e in load(out/'no-images-result.json')['trace'] if e['change']['kind']=='guard'], 'model-free exact standalone guard replay')
    rows=[]
    with tempfile.TemporaryDirectory(prefix='supervised-controls-') as directory:
        temp=Path(directory); fake=temp/'zero.onnx'
        with fake.open('xb') as stream:stream.truncate(3_659_407)
        link=temp/'model-link.onnx'; link.symlink_to(fake)
        for name,model,error in [('wrong-S-model',args.s_model,'model size differs'),('wrong-nano-hash',fake,'model SHA-256 differs'),('model-symlink',link,'regular non-symlink file')]:
            rows.append(dict(name=name,**native(args.binary,['--model',model,'--sequence',out/'valid.json'],out/(name+'-unexpected.json'),2,error)))
    rows.append(dict(name='unknown-cli',**native(args.binary,['--unknown'],out/'unknown-unexpected.json',2,'expected --model')))
    write(out/'supervised-controls.json',dict(pass_checks=True,shared_rejected_inputs=21,additional_rejections=rows,output_protections=2,
        valid_model_forwards=0,model_free_guard_replays=1,checker_sha256=digest(__file__),executable_sha256=digest(args.binary)))
    print(json.dumps(dict(pass_checks=True,rejected_inputs=25,valid_model_forwards=0)),flush=True)


def supervision_metrics(report):
    completed={row['change']['event_index']:row['change'] for row in report['trace'] if row['change']['kind']=='finished'}
    overlaps=[]; expiry=[]
    for row in report['trace']:
        c=row['change']; active=row['active_source_index']
        if c['kind']!='guard' or active is None or active not in completed:continue
        work=completed[active]
        inside=work['worker_started_s'] <= row['observed_s'] < work['worker_finished_s']
        if c['reason']=='source_barrier' and c['event']['input']['kind']=='failed':
            overlaps.append(dict(event_index=c['event_index'],active_source_index=active,observed_s=row['observed_s'],
                worker_finished_s=work['worker_finished_s'],delivered_during_actual_inference=inside,
                delay_s=row['observed_s']-report['source_slots'][c['event_index']]['release_offset_s']))
        if any(u['trigger']=='deadline_expiry' for u in c['output']['backend_updates']):
            expiry.append(dict(observed_s=row['observed_s'],active_source_index=active,observed_during_actual_inference=inside,
                delay_s=row['observed_s']-c['output']['fault_epoch_s']))
    return dict(fault_observations=overlaps,expiry_observations=expiry,
        observed_poll_target_met=report['summary']['max_observed_poll_gap_s']<=.01,
        observed_fault_10ms_gate=None if not overlaps else all(r['delay_s']<=.01 and r['delivered_during_actual_inference'] for r in overlaps),
        hard_real_time_guarantee=False)


def run(args):
    out=fresh(args.output)
    require(load(args.controls/'supervised-controls.json')['valid_model_forwards']==0, 'controls before valid inference')
    source_path=ROOT/'assets/camera-sequence-v1/sequence.json'; source=load(source_path)
    require(digest(source_path)==nano.previous.SOURCE_SHA and digest(args.model)==nano.MODEL_SHA, 'unchanged original images, clocks and Nano weights')
    fault=copy.deepcopy(source); fault['events']=fault['events'][:1]
    fault['events'][0]['input']['path']=os.path.relpath((source_path.parent/source['events'][0]['input']['path']).resolve(),out.resolve())
    origin=Decimal(source['events'][0]['timestamp_s'])
    for offset,kind in [('0.010','failed'),('0.015','missing')]:
        fault['events'].append(dict(timestamp_s=str(origin+Decimal(offset)),input=dict(kind=kind)))
    fault_path=out/'fault-sequence.json'; write(fault_path,fault)
    binaries=dict(latest=args.baseline,supervised=args.binary); receipts={}; inspections={}
    for mode,binary in binaries.items():
        receipts[mode],_=native(binary,['--executable-receipt'],out/(mode+'-executable.json'))
        require(receipts[mode]['executable_sha256']==digest(binary), 'actual executable identity')
        require(receipts[mode]['compiled_sources']==(expected() if mode=='supervised' else latest.expected()), 'complete executable source closure')
        for prefix,path in [('',source_path),('fault-',fault_path)]:
            inspections[prefix+mode],_=native(binary,['--inspect-sequence',path],out/(prefix+mode+'-inspection.json'))
    pins=dict(load(ROOT/'assets/camera-nano-repeat2x-v1/source-sha256.json'))
    pins.update({name:digest(ROOT/name) for name in NEW+['integrations/onnx/camera_supervised_check.py','integrations/onnx/test_camera_supervised.py']})
    require(all(digest(ROOT/name)==pin for name,pin in pins.items()), 'all prior sources preserved before inference')
    write(out/'before-inference.json',dict(order=ORDER,source_sha256=pins,source_sequence_sha256=digest(source_path),fault_sequence_sha256=digest(fault_path),
        model_sha256=digest(args.model),executable_receipts=receipts,serial_executable_sha256=digest(args.serial_binary),guard_executable_sha256=digest(args.guard),
        policy=freshness.POLICY,inference_warmup=False,new_holdout=False,new_accuracy_scoring=False,driving_integration=False,
        fault_offsets_s=['0.010','0.015'],fault_kinds=['failed','missing'],fault_delay_gate_s=.01,
        timing_success_required_for_execution=False,trial_retention='Keep every first/repeat observation; no timing-based retries'))
    shutil.copyfile(__file__,out/'initial-checker.py')
    trials=[]
    for label in ORDER:
        prefix='fault-' if label.startswith('fault-') else ''
        mode=label.removeprefix('fault-').split('-')[0]
        report,wall=native(binaries[mode],['--model',args.model,'--sequence',fault_path if prefix else source_path],out/(label+'.json'))
        trial=dict(label=label,external_wall_s=wall,model_forwards=report['model_forwards'],summary=report['summary'])
        trials.append(trial); write(out/(label+'-trial.json'),trial)
        print(json.dumps(dict(label=label,model_forwards=report['model_forwards'])),flush=True)
    write(out/'trials.json',trials)
    references={}; fresh(out/'references')
    for index,event in enumerate(source['events']):
        image=source_path.parent/event['input']['path']
        serial,_=native(args.serial_binary,['--model',args.model,'--serial-image',image],out/'references'/f'{index}.json')
        saved=load(ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{index}.json')
        nano.check_image(serial,saved,parallel=False); references[event['input']['sha256']]=serial
    comparisons=[]
    for trial in trials:
        label=trial['label']; prefix='fault-' if label.startswith('fault-') else ''; mode=label.removeprefix('fault-').split('-')[0]
        report=load(out/(label+'.json')); original=fault if prefix else source
        require(report['source_sequence_sha256']==digest(fault_path if prefix else source_path)
                and report['compiled_sources']==receipts[mode]['compiled_sources'], 'actual report source/hash binding')
        require(report['startup_before_schedule_wall_s']+report['schedule_wall_s']<=trial['external_wall_s']+.01, 'external clock covers measured run')
        if mode=='supervised':
            manifest=audit(report,original,inspections[prefix+mode],references)
            outputs=[r['change']['output'] for r in report['trace'] if r['change']['kind']=='guard']
            available=report['summary']['available_result_images']; inferred=report['summary']['completed_images']
            fault_delay=[r['observed_s']-report['source_slots'][r['change']['event_index']]['release_offset_s'] for r in report['trace']
                if r['change']['kind']=='guard' and r['change']['reason']=='source_barrier' and r['change']['event']['input']['kind']=='failed']
            metrics=supervision_metrics(report)
        else:
            ref=[references[e['input']['sha256']] for e in original['events'] if e['input']['kind']=='image']
            manifest=latest.audit(report,original,inspections[prefix+mode],ref)
            outputs=[e['output'] for row in report['events'] for e in row['measured']['guard_events']]
            available=report['summary']['available_at_dispatched_checks']; inferred=report['summary']['inferred_images']
            fault_delay=[row['measured']['guard_events'][0]['event']['now_s']-row['source']['release_offset_s'] for row in report['events']
                if original['events'][row['source']['event_index']]['input']['kind']=='failed']
            metrics=None
        write(out/(label+'-guard-input.json'),manifest)
        result,_=native(args.guard,['--sequence',out/(label+'-guard-input.json')],out/(label+'-guard-result.json'))
        require(result['results']==outputs, 'every independent standalone guard output is exact')
        comparisons.append(dict(label=label,source_images=sum(e['input']['kind']=='image' for e in original['events']),
            model_forwards=report['model_forwards'],inferred_images=inferred,available_results=available,
            superseded_images=report['summary']['superseded_images'],inferred_but_late_images=report['summary']['inferred_but_late_images'],
            fault_delay_s=fault_delay,supervision=metrics))
    require(all(digest(ROOT/name)==pin for name,pin in pins.items()), 'all source files unchanged across inference')
    require(all(digest(binaries[k])==v['executable_sha256'] for k,v in receipts.items()) and digest(args.guard)==load(out/'before-inference.json')['guard_executable_sha256']
            and digest(args.serial_binary)==load(out/'before-inference.json')['serial_executable_sha256'], 'all executables unchanged')
    result=dict(pass_checks=True,comparison=comparisons,timed_model_forwards=sum(r['model_forwards'] for r in trials),current_reference_model_forwards=32,
        actual_model_forwards=sum(r['model_forwards'] for r in trials)+32,independent_guard_replays=8,
        current_serial_measurement_bits_exact=True,dropped_images_counted_unavailable=True,
        new_holdout=False,new_accuracy_scoring=False,driving_integration=False,hard_real_time_guarantee=False,checker_sha256=digest(__file__))
    write(out/'summary.json',result); print(json.dumps(result),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-supervised')
    parser.add_argument('--baseline',type=Path,default=HERE/'target/release/rustdriving-camera-latest')
    parser.add_argument('--serial-binary',type=Path,default=HERE/'target/release/rustdriving-camera-nano')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model/yolox_nano.onnx')
    parser.add_argument('--s-model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    (controls if args.mode=='controls' else run)(args)
