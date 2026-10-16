#!/usr/bin/env python3
"""Independently audit latest-ready selection, actual clocks and unchanged Nano pixels."""
import argparse
import ast
import copy
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location('latest_nano_support', HERE / 'camera_nano_check.py')
nano = importlib.util.module_from_spec(spec); spec.loader.exec_module(nano)
latency, freshness, tracking = nano.latency, nano.freshness, nano.tracking
require, digest, load, write, fresh, native = latency.require, latency.digest, latency.load, latency.write, latency.fresh, latency.native
NEW = ['integrations/onnx/src/camera_latest.rs', 'integrations/onnx/src/bin/rustdriving-camera-latest.rs', 'integrations/onnx/profile-latest.json']
ORDER = ['fifo-first', 'latest-first', 'latest-repeat', 'fifo-repeat']


def packet(name):
    return json.loads(gzip.decompress((ROOT / ('assets/camera-nano-v1/' + name + '.gz')).read_bytes()))


def expected():
    return dict(nano.expected('runtime'), **{name:digest(ROOT / name) for name in NEW})


def check_shared_controller():
    original = (HERE / 'src/camera_latency.rs').read_text()
    require(digest(HERE / 'src/camera_latency.rs') == '8d30d058cf7959b951f24501d662647fcc1d5b7551267e4fb6ed0bd5a90f4908', 'original controller source')
    block = original[original.index('trait Clock {'):original.index('#[derive(Serialize)]\npub struct EventReport')]
    current = (HERE / 'src/camera_latest.rs').read_text()
    require(current[current.index('trait Clock {'):current.index('// Non-image events')] == block, 'unchanged complete measured per-frame controller')
    text = (HERE / 'camera_latency_check.py').read_text()
    node = next(n for n in ast.parse(text).body if isinstance(n, ast.FunctionDef) and n.name == 'controls')
    shared = ast.get_source_segment(text, node).replace('def controls(', 'def common_controls(').replace("['deadline_gate_pass']", "['dispatched_deadline_pass']")
    current = Path(__file__).read_text()
    node = next(n for n in ast.parse(current).body if isinstance(n, ast.FunctionDef) and n.name == 'common_controls')
    require(ast.get_source_segment(current, node) == shared, 'unchanged input and publication controls except renamed summary key')
    nano.check_clock_source()


def audit(report, source, inspection, references=None):
    require(report['schema_version'] == 1 and report['algorithm'] == 'yolox_nano_latest_ready_replay_v1'
            and report['clock'] == 'std::time::Instant_latest_ready_replay', 'distinct latest-ready identity')
    require(report['compiled_sources'] == expected() and report['model_sha256'] == nano.MODEL_SHA, 'complete latest-ready source/model receipt')
    require(report['policy'] == freshness.POLICY and report['control_permission'] is False, 'original age/recovery policy')
    require(report['source_sequence_sha256'] == inspection['source_sequence_sha256'] and inspection['compiled_sources'] == expected(), 'source inspection binding')
    require(inspection['algorithm'] == 'yolox_nano_latest_ready_schedule_validation_v1'
            and inspection['policy'] == freshness.POLICY and inspection['control_permission'] is False, 'inspection identity and policy')
    require(not inspection['model_read'] and not inspection['inference_run'], 'complete preflight without model')
    for field in ('camera_id', 'width', 'height'):
        require(report[field] == inspection[field] == source[field], 'original camera identity')
    require(report['config'] == inspection['config'], 'original tracking configuration')
    origin = float(source['events'][0]['timestamp_s']); slots = []
    for index, event in enumerate(source['events']):
        value = float(event['timestamp_s']); offset = value - origin
        require(math.isfinite(value) and 0 <= offset <= 2 and (not slots or value - slots[-1]['source_timestamp_s'] >= 1e-6), 'original bounded acquisition clock')
        slots.append(dict(event_index=index, source_timestamp_text=event['timestamp_s'], source_timestamp_s=value,
            release_offset_s=offset, source_image_sha256=event['input'].get('sha256')))
    require(report['source_slots'] == inspection['slots'] == slots and inspection['images_read'] == sum(s['source_image_sha256'] is not None for s in slots), 'complete original source inventory')
    next_index = 0; previous = None; events = []; outputs = []
    counts = dict(source_images=sum(s['source_image_sha256'] is not None for s in slots), dispatched_images=0,
        superseded_images=0, inferred_images=0, inference_excluded_images=0, inferred_but_late_images=0,
        late_dispatched_images=0, available_at_dispatched_checks=0)
    for row in report['events']:
        selected_at = row['selection_checked_s']
        require(type(selected_at) in (int,float) and math.isfinite(selected_at) and 0 <= selected_at <= 1e12, 'observed finite selection clock')
        if previous is not None:require(selected_at - previous['now_s'] >= 1e-6, 'selection follows previous processing')
        due = [i for i in range(next_index,len(slots)) if slots[i]['release_offset_s'] <= selected_at]
        require(due and due[0] == next_index, 'selection only after original release')
        barriers = [i for i in due if source['events'][i]['input']['kind'] != 'image']
        index = min(barriers) if barriers else max(due)
        require(row['source'] == slots[index] and row['superseded_sources'] == slots[next_index:index], 'independent latest-ready choice and exact discarded identities')
        require(all(source['events'][i]['input']['kind'] == 'image' for i in range(next_index,index)), 'no missing/failure event discarded')
        counts['superseded_images'] += index - next_index
        original = source['events'][index]; offset = slots[index]['release_offset_s']
        measured = row['measured']; timing = measured['timing']
        start, submit, check = [timing[k] for k in ('processing_started_s','sample_submitted_s','checked_after_tracking_s')]
        require(all(type(v) in (int,float) and math.isfinite(v) and v <= 1e12 for v in (start,submit,check))
                and start - selected_at >= 1e-6 and submit - start >= 1e-6 and check - submit >= 1e-6, 'actual ordered host checkpoints')
        require(timing['read_analysis_and_probe_wall_s'] == submit-start and timing['sample_guard_wall_s'] == check-submit, 'exact measured stage arithmetic')
        entries = measured['guard_events']
        require(len(entries) == 2 and entries[0]['event']['now_s'] == submit and entries[1]['event'] == dict(now_s=check,input=dict(kind='tick')), 'complete sample and post-tracking check')
        data = entries[0]['event']['input']; inferred = measured['analysis'] is not None
        if original['input']['kind'] == 'image':
            counts['dispatched_images'] += 1
            require(data['kind'] == 'sample' and data['acquired_s'] == offset and data['image_sha256'] == original['input']['sha256'], 'unchanged acquired time and image hash')
            require(timing['source_age_at_start_s'] == start-offset and timing['source_age_at_check_s'] == check-offset
                    and timing['acquisition_deadline_s'] == offset+.2, 'exact source ages and unchanged deadline')
            missed = check > offset+.2
            require(timing['deadline_missed_at_check'] is missed, 'same inclusive deadline')
            accepted = previous['last_accepted_acquired_s'] if previous else None
            barrier = previous['fault_epoch_s'] if previous else None
            armed = accepted is not None and (barrier is None or accepted > barrier)
            if armed and start > accepted+.2:barrier = accepted+.2
            decision = ('before_fault' if barrier is not None and offset <= barrier else
                        'out_of_order' if accepted is not None and offset-accepted < 1e-6 else
                        'stale_sample' if start > offset+.2 else 'accepted')
            require(measured['precheck_decision'] == decision and inferred == (decision == 'accepted'), 'independent original inference admission')
            if inferred:
                analysis = measured['analysis']
                saved = load(ROOT / f'assets/camera-sequence-v1/recorded/exports/first-{index}.json')
                nano.check_image(analysis, saved, None if references is None else references[index])
                f32 = lambda v:struct.unpack('<f',struct.pack('<f',v))[0]
                boxes = [dict(class_index=d['class_index'],confidence=f32(d['confidence']),bbox=list(map(f32,d['bbox']))) for d in analysis['result']['detections']]
                require(data['detections'] == boxes, 'unchanged f32 detections enter the original guard')
                require(submit-start >= analysis['execution']['analysis_wall_s'], 'processing covers full image analysis')
            else:require(data['detections'] == [], 'excluded image invents no measurements')
            counts['inferred_images'] += inferred; counts['inference_excluded_images'] += not inferred
            counts['inferred_but_late_images'] += inferred and missed; counts['late_dispatched_images'] += missed
            counts['available_at_dispatched_checks'] += entries[1]['output']['ready']
        else:
            require(data == dict(kind='tick' if original['input']['kind'] == 'missing' else 'failed'), 'non-image barrier delivered unchanged')
            require(measured['precheck_decision'] is None and not inferred, 'barrier does no inference or sample recovery')
            require(all(timing[k] is None for k in ('source_age_at_start_s','source_age_at_check_s','acquisition_deadline_s','deadline_missed_at_check')), 'no invented barrier source age')
        for entry in entries:
            event = copy.deepcopy(entry['event'])
            if event['input']['kind'] == 'sample':event['input']['kind'] = 'detections'
            events.append(event); outputs.append(entry['output'])
        previous = entries[-1]['output']; next_index = index+1
    require(next_index == len(slots), 'every original event accounted once')
    counts.update(dispatched_deadline_pass=None if not counts['dispatched_images'] else counts['late_dispatched_images'] == 0,
        all_source_images_inferred=counts['source_images'] > 0 and counts['inferred_images'] == counts['source_images'],
        all_source_results_available=counts['source_images'] > 0 and counts['available_at_dispatched_checks'] == counts['source_images'])
    require(report['summary'] == counts and counts['dispatched_images'] + counts['superseded_images'] == counts['source_images'], 'independent complete coverage and deadline denominators')
    require(report['model_loads'] == int(counts['source_images'] > 0) and report['preflight_image_reads'] == counts['source_images']
            and report['inference_image_reads'] == counts['inferred_images'] and report['model_forwards'] == counts['inferred_images']*4, 'actual inference and model reads')
    require(report['startup_before_schedule_wall_s'] >= report['manifest_and_preflight_wall_s'] + report['model_load_wall_s']
            and report['schedule_wall_s'] >= previous['now_s'], 'startup and measured schedule accounting')
    manifest = dict(schema_version=1,camera_id=report['camera_id'],width=report['width'],height=report['height'],config=report['config'],policy=report['policy'],events=events)
    freshness.check_contract(manifest, events, outputs, tracking)
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
    require(no_model['summary']['dispatched_deadline_pass'] is None,'empty trace has no image timing gate')
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


def controls(args):
    check_shared_controller()
    common_controls(args)
    out = args.output; added = []
    with tempfile.TemporaryDirectory(prefix='latest-model-controls-') as directory:
        fake = Path(directory) / 'bad.onnx'
        with fake.open('xb') as stream:stream.truncate(3_659_407)
        link = Path(directory) / 'linked.onnx'; link.symlink_to(fake)
        for name, model, error in [('wrong-model', args.s_model, 'model size differs'),
                                   ('wrong-hash', fake, 'model SHA-256 differs'),
                                   ('model-symlink', link, 'regular non-symlink file')]:
            added.append(dict(name=name, **native(args.binary, ['--model',model,'--sequence',out/'valid.json'], out/(name+'-unexpected.json'), 2, error)))
    added.append(dict(name='unknown-cli', **native(args.binary, ['--unknown'], out/'cli-unexpected.json', 2, 'expected --model')))
    inspection,_ = native(args.binary, ['--inspect-sequence',out/'no-images.json'], out/'no-images-inspection.json')
    manifest = audit(load(out/'no-images-result.json'),load(out/'no-images.json'),inspection)
    write(out/'no-images-guard-input.json',manifest)
    guard,_ = native(args.guard,['--sequence',out/'no-images-guard-input.json'],out/'no-images-guard-result.json')
    require(guard['results'] == [e['output'] for row in load(out/'no-images-result.json')['events'] for e in row['measured']['guard_events']], 'non-image independent guard replay')
    write(out/'latest-controls.json',dict(pass_checks=True,shared_rejected_inputs=21,additional_rejections=added,output_protections=2,
        original_frame_controller_exact=True,independent_non_image_guard_replay=True,valid_model_forwards=0,
        checker_sha256=digest(__file__),executable_sha256=digest(args.binary)))


def run(args):
    out = fresh(args.output); check_shared_controller()
    control = load(args.controls/'latest-controls.json')
    require(control['pass_checks'] and control['checker_sha256'] == digest(__file__) and control['executable_sha256'] == digest(args.binary), 'matching input controls')
    source_path = ROOT/'assets/camera-sequence-v1/sequence.json'; source = load(source_path)
    require(digest(source_path) == nano.previous.SOURCE_SHA and digest(args.model) == nano.MODEL_SHA, 'unchanged schedule and Nano weights')
    binaries = dict(fifo=args.baseline, latest=args.binary); receipts = {}; inspections = {}
    for mode, binary in binaries.items():
        receipts[mode],_ = native(binary,['--executable-receipt'],out/(mode+'-executable.json'))
        require(receipts[mode]['executable_sha256'] == digest(binary), 'actual native binary receipt')
        require(receipts[mode]['compiled_sources'] == (expected() if mode == 'latest' else nano.expected('runtime')), 'complete native sources')
        inspections[mode],_ = native(binary,['--inspect-sequence',source_path],out/(mode+'-inspection.json'))
    require(inspections['latest']['slots'] == inspections['fifo']['slots'], 'exact same original acquisition schedule')
    pins = dict(load(ROOT/'assets/camera-nano-v1/source-sha256.json'))
    pins.update({name:digest(ROOT/name) for name in NEW + ['integrations/onnx/camera_latest_check.py']})
    require(all(digest(ROOT/name) == pin for name,pin in pins.items()), 'all frozen sources unchanged before inference')
    write(out/'before-inference.json',dict(order=ORDER,source_sha256=pins,source_sequence_sha256=digest(source_path),model_sha256=digest(args.model),
        executable_receipts=receipts,guard_executable_sha256=digest(args.guard),policy=freshness.POLICY,new_holdout=False,inference_warmup=False,
        current_reference_images=8,full_source_denominator=8,continuous_live_camera=False))
    trials = []
    for label in ORDER:
        mode = label.split('-')[0]
        report,wall = native(binaries[mode],['--model',args.model,'--sequence',source_path],out/(label+'.json'))
        trials.append(dict(label=label,external_wall_s=wall,model_forwards=report['model_forwards'],summary=report['summary']))
        write(out/(label+'-trial.json'),trials[-1])
    write(out/'trials.json',trials)
    fresh(out/'references')
    references = []
    for index,event in enumerate(source['events']):
        image = source_path.parent/event['input']['path']
        saved = load(ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{index}.json')
        require(digest(image) == event['input']['sha256'] == saved['source_image_sha256'], 'original full image identity')
        serial,_ = native(args.baseline,['--model',args.model,'--serial-image',image],out/'references'/f'{index}.json')
        nano.check_image(serial,saved,parallel=False); references.append(serial)
    comparisons = []
    for trial in trials:
        label = trial['label']; mode = label.split('-')[0]; report = load(out/(label+'.json'))
        require(report['compiled_sources'] == receipts[mode]['compiled_sources'] and report['source_sequence_sha256'] == nano.previous.SOURCE_SHA, 'timed report receipt')
        require(report['startup_before_schedule_wall_s'] + report['schedule_wall_s'] <= trial['external_wall_s']+.01, 'external timer covers startup and schedule')
        if mode == 'latest':manifest = audit(report,source,inspections[mode],references)
        else:
            manifest,_,_ = nano.clock.audit_measured(report,source,inspections[mode],freshness,tracking)
            for row in report['events']:
                if row['measured']['analysis'] is not None:
                    index = row['source']['event_index']
                    nano.check_image(row['measured']['analysis'],load(ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{index}.json'),references[index])
        path = out/(label+'-guard-input.json'); write(path,manifest)
        guard,_ = native(args.guard,['--sequence',path],out/(label+'-guard-result.json'))
        require(guard['results'] == [entry['output'] for row in report['events'] for entry in row['measured']['guard_events']], 'exact independent native guard replay')
        summary = report['summary']
        comparisons.append(dict(label=label,source_images=8,model_forwards=report['model_forwards'],
            inferred_images=summary['inferred_images'],superseded_images=summary.get('superseded_images',0),
            available_results=summary['available_at_dispatched_checks'] if mode == 'latest' else summary['available_at_check_images'],
            inferred_but_late_images=summary['inferred_but_late_images'],
            selected_indices=[row['source']['event_index'] for row in report['events']],
            processing_s=[row['measured']['timing']['read_analysis_and_probe_wall_s'] for row in report['events'] if row['measured']['analysis'] is not None]))
    require(all(digest(ROOT/name) == pin for name,pin in pins.items()) and all(digest(binaries[m]) == r['executable_sha256'] for m,r in receipts.items()), 'sources and binaries unchanged during comparison')
    require(digest(args.guard) == load(out/'before-inference.json')['guard_executable_sha256'], 'unchanged guard executable')
    summary = dict(pass_checks=True,comparison= comparisons,timed_model_forwards=sum(r['model_forwards'] for r in trials),
        current_reference_model_forwards=32,actual_model_forwards=sum(r['model_forwards'] for r in trials)+32,
        current_serial_measurement_bits_exact=True,independent_guard_replays=4,source_images_per_trial=8,
        dropped_images_counted_unavailable=True,timing_or_availability_success_required_for_execution=False,
        new_holdout=False,new_accuracy_scoring=False,driving_integration=False,checker_sha256=digest(__file__))
    write(out/'summary.json',summary); print(json.dumps(summary),flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-latest')
    parser.add_argument('--baseline',type=Path,default=HERE/'target/release/rustdriving-camera-nano')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model/yolox_nano.onnx')
    parser.add_argument('--s-model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    for name in ('binary','baseline','guard','model','s_model','output'):
        setattr(args,name,getattr(args,name).absolute())
    if args.mode == 'controls':controls(args)
    else:
        require(args.controls is not None,'--controls required'); run(args)
