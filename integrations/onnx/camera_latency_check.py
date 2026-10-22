#!/usr/bin/env python3
"""Independently audit measured serial replay; timing failures stay visible.

The timer is real host monotonic time. Input release times come from a fixed
viewed recording. This is not live-camera latency or a real-time guarantee.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
history_spec = importlib.util.spec_from_file_location('latency_history', HERE / 'camera_history.py')
camera_history = importlib.util.module_from_spec(history_spec)
history_spec.loader.exec_module(camera_history)
FRESH_CHECK_SHA = '816e6c0a2f53abf8af38cb5b1875127cd86f25fe8294cf8b6db057f3929589b7'
SOURCE_SEQUENCE_SHA = '5fa15877e7d910fe102dae175af40b5ee473c6f614b0740f36127b58b77f864c'
MODEL_SHA = 'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path):
    require(path.stat().st_size <= 16 * 1024 * 1024, 'bounded report')
    return json.loads(path.read_text())


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def fresh(path):
    require(not path.exists() and not path.is_symlink(), 'fresh evidence directory')
    path.mkdir(parents=True)
    return path


def without_timing(value):
    if isinstance(value, dict):
        return {k:without_timing(v) for k,v in value.items() if k not in ('inference_seconds','model_load_seconds')}
    if isinstance(value, list):
        return list(map(without_timing,value))
    return value


def oracle():
    path = ROOT/'integrations/camera-tracking/freshness_check.py'
    require(digest(path)==FRESH_CHECK_SHA, 'unchanged independent freshness checker')
    spec=importlib.util.spec_from_file_location('measured_freshness_oracle',path)
    support=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    return support,support.oracle()


def native(binary, arguments, destination, expected=0, error=None):
    begin=time.perf_counter()
    process=subprocess.run([str(binary),*map(str,arguments),'--output',str(destination)],
                           capture_output=True,text=True,timeout=240)
    elapsed=time.perf_counter()-begin
    require(process.returncode==expected,'native exit: '+process.stderr[-2000:])
    if error:
        require(error in process.stderr,'failure stage: '+process.stderr)
    require(not list(destination.parent.glob('.rustdriving-camera-*.partial')),'staging leaked')
    if expected:
        require(not destination.exists(),'failure published partial output')
        return dict(exit=process.returncode,stderr=process.stderr)
    return load(destination),elapsed


def controls(args):
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
    require(no_model['summary']['deadline_gate_pass'] is None,'empty trace has no image timing gate')
    require(all(e['measured']['analysis'] is None for e in no_model['events']),'no synthetic detections')
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


def audit(report,source,inspection,support,freshness):
    require(report['algorithm']=='yolox_s_measured_serial_replay_v1' and report['clock']=='std::time::Instant_serial_replay','measured-clock identity')
    require(report['control_permission'] is False and report['policy']==freshness.POLICY,'fixed prior freshness policy')
    require(report['camera_id']==source['camera_id'] and report['width']==source['width'] and report['height']==source['height'],'camera identity')
    require(len(report['events'])==len(source['events']),'all scheduled frames retained')
    origin=float(source['events'][0]['timestamp_s'])
    dispatch_events,outputs=[],[]
    previous=None
    counts=dict(image_events=0,inferred_images=0,skipped_images=0,inferred_but_late_images=0,late_at_check_images=0,available_at_check_images=0)
    ages=[]
    for index,(original,actual,slot) in enumerate(zip(source['events'],report['events'],inspection['slots'])):
        require(actual['source']==slot,'checked original source metadata')
        offset=float(original['timestamp_s'])-origin
        require(slot['event_index']==index and slot['source_timestamp_text']==original['timestamp_s'] and slot['release_offset_s']==offset,'original time/offset identity')
        measured=actual['measured']
        timing=measured['timing']
        started,submitted,checked=[timing[k] for k in ('processing_started_s','sample_submitted_s','checked_after_tracking_s')]
        require(all(math.isfinite(v) for v in (started,submitted,checked)) and 0<=started and submitted-started>=1e-6 and checked-submitted>=1e-6,'observed checkpoint order')
        require(started>=offset,'frame not released before original source offset')
        if previous is not None:
            require(started-previous['now_s']>=1e-6,'serial host clock continues across frames')
        require(timing['read_analysis_and_probe_wall_s']==submitted-started and timing['sample_guard_wall_s']==checked-submitted,'measured durations derived from checkpoints')
        entries=measured['guard_events']
        require(len(entries)==2 and entries[0]['event']['now_s']==submitted and entries[1]['event']==dict(now_s=checked,input=dict(kind='tick')),'post-tracking check actually evaluated')
        data=entries[0]['event']['input']
        if original['input']['kind']=='image':
            counts['image_events']+=1
            require(data['kind']=='sample' and data['acquired_s']==offset and data['image_sha256']==original['input']['sha256'],'image/source clock enters guard unchanged')
            require(timing['source_age_at_start_s']==started-offset and timing['source_age_at_check_s']==checked-offset
                    and timing['acquisition_deadline_s']==offset+.2,'queue and output ages')
            missed=checked>offset+.2
            require(timing['deadline_missed_at_check']==missed,'deadline evaluated from actual checkpoint')
            ages.append(checked-offset)
            barrier=previous['fault_epoch_s'] if previous else None
            accepted=previous['last_accepted_acquired_s'] if previous else None
            armed=accepted is not None and (barrier is None or accepted>barrier)
            if armed and started>accepted+.2:
                barrier=accepted+.2
            expected=('before_fault' if barrier is not None and offset<=barrier else
                      'out_of_order' if accepted is not None and offset-accepted<1e-6 else
                      'stale_sample' if started>offset+.2 else 'accepted')
            require(measured['precheck_decision']==expected,'independent start-time admission')
            inferred=measured['analysis'] is not None
            require(inferred==(expected=='accepted'),'analyze exactly when start-time admission accepts')
            counts['inferred_images']+=inferred
            counts['skipped_images']+=not inferred
            counts['inferred_but_late_images']+=inferred and missed
            counts['late_at_check_images']+=missed
            counts['available_at_check_images']+=entries[1]['output']['ready']
            if inferred:
                f32=lambda v:struct.unpack('<f',struct.pack('<f',v))[0]
                boxes=[dict(class_index=d['class_index'],confidence=f32(d['confidence']),bbox=list(map(f32,d['bbox']))) for d in measured['analysis']['result']['detections']]
                require(data['detections']==boxes,'actual model f32 measurements enter guard')
                view_time=sum(v['prediction']['inference_seconds'] for v in measured['analysis']['base_prediction']['views'])
                require(timing['read_analysis_and_probe_wall_s']>=view_time,'full processing covers actual model forwards')
            else:
                require(data['detections']==[],'no invented detections for skipped frame')
        else:
            require(measured['precheck_decision'] is None and measured['analysis'] is None,'non-image event has no analyzer probe')
        for item in entries:
            event=copy.deepcopy(item['event'])
            if event['input']['kind']=='sample':
                event['input']['kind']='detections'
            dispatch_events.append(event)
            outputs.append(item['output'])
        previous=entries[-1]['output']
    maximum=max(ages) if ages else None
    expected=counts|dict(maximum_source_age_at_check_s=maximum,
                        deadline_gate_pass=counts['late_at_check_images']==0 if ages else None)
    require(report['summary']==expected,'independently reconstructed deadline summary')
    require(report['model_loads']==1 and report['preflight_image_reads']==8 and report['inference_image_reads']==counts['inferred_images']
            and report['model_forwards']==counts['inferred_images']*4,'model/read/forward accounting')
    require(report['startup_before_schedule_wall_s']>=report['manifest_and_preflight_wall_s']+report['model_load_wall_s'],'startup stage accounting')
    require(report['schedule_wall_s']>=previous['now_s'],'measured schedule covers final check')
    manifest=dict(schema_version=1,camera_id=report['camera_id'],width=report['width'],height=report['height'],
                  config=report['config'],policy=report['policy'],events=dispatch_events)
    backend,snapshots=freshness.check_contract(manifest,dispatch_events,outputs,support)
    return manifest,backend,snapshots


def run(args):
    out=fresh(args.output)
    controls=load(args.controls/'summary.json')
    require(controls['pass_checks'] and controls['executable_sha256']==digest(args.binary) and controls['checker_sha256']==digest(__file__),'matching source-bound controls')
    require(digest(args.model)==MODEL_SHA,'pinned original model')
    source_path=ROOT/'assets/camera-sequence-v1/sequence.json'
    require(digest(source_path)==SOURCE_SEQUENCE_SHA,'unchanged original eight-frame selection')
    original=load(source_path)
    source=copy.deepcopy(original)
    for event in source['events']:
        event['input']['path']=os.path.relpath(source_path.parent/event['input']['path'],out)
    manifest_path=out/'sequence.json'
    write(manifest_path,source)
    freshness,support=oracle()
    receipt,_=native(args.binary,['--executable-receipt'],out/'executable.json')
    inspected,_=native(args.binary,['--inspect-sequence',manifest_path],out/'inspection.json')
    source_pins=dict(receipt['compiled_sources'])
    for path in [Path(__file__),HERE/'camera_history.py',ROOT/'integrations/camera-tracking/freshness_check.py',ROOT/'integrations/camera-tracking/check.py']:
        source_pins[str(path.relative_to(ROOT))]=digest(path)
    write(out/'before-inference.json',dict(executable_sha256=digest(args.binary),source_sha256=source_pins,
        model_sha256=MODEL_SHA,original_source_sequence_sha256=SOURCE_SEQUENCE_SHA,measured_sequence_sha256=digest(manifest_path),
        image_selection='Previously viewed TUM Freiburg3 sitting_xyz original RGB indices102..109',
        deadline_policy_fixed_before_measurement=freshness.POLICY,serial_fifo_original_offsets=True,warmup=False,
        measured_host_time=True,live_camera=False,accuracy_ground_truth=False))
    saved={load(p)['source_image_sha256']:load(p) for p in (ROOT/'assets/camera-sequence-v1/recorded/exports').glob('first-*.json')}
    rows=[]
    historical=camera_history.Comparison()
    forwards=0
    for label in ('first','repeat'):
        print('Measuring '+label,flush=True)
        report,outer_wall=native(args.binary,['--model',args.model,'--sequence',manifest_path],out/(label+'.json'))
        require(report['compiled_sources']==receipt['compiled_sources'] and report['source_sequence_sha256']==digest(manifest_path),'source receipts')
        require(report['startup_before_schedule_wall_s']+report['schedule_wall_s']<=outer_wall+.01,'external process clock bounds internal measurements')
        guard_manifest,backend,snapshots=audit(report,source,inspected,support,freshness)
        for index,event in enumerate(report['events']):
            analysis=event['measured']['analysis']
            if analysis is not None:
                historical.same(without_timing(saved[event['source']['source_image_sha256']]),without_timing(analysis),label+f'.analysis[{index}]')
        guard_path=out/(label+'-guard-input.json')
        write(guard_path,guard_manifest)
        guard_result,_=native(args.guard,['--sequence',guard_path],out/(label+'-guard-result.json'))
        require(guard_result['results']==[d['output'] for e in report['events'] for d in e['measured']['guard_events']],'exact independent-process guard replay')
        if backend:
            tracker_path=out/(label+'-tracker-input.json')
            write(tracker_path,dict(schema_version=1,camera_id=report['camera_id'],width=report['width'],height=report['height'],config=report['config'],events=backend))
            tracker,_=native(args.tracker,['--sequence',tracker_path],out/(label+'-tracker-result.json'))
            require(tracker['snapshots']==snapshots,'exact independent-process tracker replay')
        forwards+=report['model_forwards']
        rows.append(dict(run=label,external_process_wall_s=outer_wall,manifest_and_preflight_wall_s=report['manifest_and_preflight_wall_s'],
            model_load_wall_s=report['model_load_wall_s'],schedule_wall_s=report['schedule_wall_s'],summary=report['summary'],
            inferred_processing_wall_s=[e['measured']['timing']['read_analysis_and_probe_wall_s'] for e in report['events'] if e['measured']['analysis'] is not None],
            exact_guard_replay=True,backend_updates_independently_checked=len(backend)))
        print(json.dumps(rows[-1]),flush=True)
    for name,pin in source_pins.items():
        require(digest(ROOT/name)==pin,'sources changed during measurement')
    require(digest(args.binary)==receipt['executable_sha256'],'executable changed during measurement')
    write(out/'summary.json',dict(pass_checks=True,measurement_runs=rows,actual_model_forwards=forwards,
        recorded_unique_images=8,measured_host_processing_time=True,scheduled_source_offsets_from_original_recording=True,
        live_camera_ingestion=False,sensor_transport_latency_measured=False,final_poll_and_json_publication_excluded=True,
        exact_guard_replay=True,real_object_identity_accuracy_measured=False,observed_deadline_gates=[r['summary']['deadline_gate_pass'] for r in rows],
        deadline_success_required_for_execution_success=False,timing_repeat_identity_required=False,
        historical_comparison=historical.report(),
        source_sha256=source_pins,executable_sha256=digest(args.binary),checker_sha256=digest(__file__)))
    print(json.dumps(dict(pass_checks=True,actual_model_forwards=forwards,observed_deadline_gates=[r['summary']['deadline_gate_pass'] for r in rows])),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-latency')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--tracker',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-track')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    for key in ('binary','guard','tracker','model','output'):
        setattr(args,key,getattr(args,key).absolute())
    if args.mode=='controls':
        controls(args)
    else:
        require(args.controls is not None,'--controls required for measured run')
        run(args)
