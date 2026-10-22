#!/usr/bin/env python3
"""Compare frozen indexed rows and slice traversal with exact existing admission."""
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import statistics
import struct
import tempfile
import camera_depthwise_check as previous
nano,repeat,supervised=previous.nano,previous.repeat,previous.supervised
HERE,ROOT=previous.HERE,previous.ROOT
require,digest,load,write,fresh=previous.require,previous.digest,previous.load,previous.write,previous.fresh
latency,freshness,tracking=previous.latency,previous.freshness,previous.tracking
native=previous.native
MODEL_SHA=previous.MODEL_SHA
measurement_bits,historical_detections,compare=previous.measurement_bits,previous.historical_detections,previous.compare
slots_for,precheck=previous.slots_for,previous.precheck
audit_profile,audit_fifo=previous.audit_profile,previous.audit_fifo
NEW=['integrations/onnx/src/camera_rowzip.rs','integrations/onnx/src/bin/rustdriving-camera-rowzip.rs','integrations/onnx/profile-rowzip.json']
ORDER=['baseline-first','optimized-first','optimized-repeat','baseline-repeat']

def expected():return dict(previous.expected(),**{name:digest(ROOT/name) for name in NEW})

def variant(report):
    require(report['producer_variant']=='nano_depthwise_rowzip_v1' and report['variant_profile_sha256']==digest(HERE/'profile-rowzip.json'),'internally computed slice-traversal producer')
    require(report['compiled_sources']==expected(),'complete slice producer source receipt')

def check_sources():
    for name in ('check_image','audit_supervised','controls'):
        text=Path(__file__).read_text();node=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        old=(HERE/'camera_depthwise_check.py').read_text();original=next(n for n in ast.parse(old).body if isinstance(n,ast.FunctionDef) and n.name==name)
        require(ast.get_source_segment(text,node)==ast.get_source_segment(old,original),'unchanged image/clock/input audit '+name)

def check_image(actual,saved,serial=None):
    require(actual['schema_version']==1 and actual['algorithm']=='yolox_nano_depthwise_parallel_image_analysis_v1','distinct lowered image producer')
    require(actual['compiled_sources']==expected() and actual['model_sha256']==MODEL_SHA and actual['control_permission'] is False,'complete lowered image receipt and no control permission')
    require(actual['graph_optimization']=='validated_resize2x_and_rowwise_depthwise3x3','only guarded Resize lowering')
    base=actual['base_prediction'];geometry=saved['base_prediction']
    require(base['algorithm']=='pinned_yolox_nano_depthwise_parallel_fixed_views_v1' and base['compiled_sources']==expected(),'distinct lowered base receipt')
    require(actual['source_image_sha256']==base['source_image_sha256']==saved['source_image_sha256'] and base['model_sha256']==MODEL_SHA,'original encoded image/model')
    require((base['width'],base['height'])==(geometry['width'],geometry['height']) and len(base['views'])==len(geometry['views'])==4,'all original views and dimensions')
    require(nano.exact_f32(base['score_threshold'],.3) and nano.exact_f32(base['nms_iou_threshold'],.45),'unchanged threshold bits')
    for view,old in zip(base['views'],geometry['views']):
        require(view['rectangle']==old['rectangle'] and view['rgb_sha256']==old['rgb_sha256'],'original ordered crop pixels')
        p=view['prediction']
        require(p['algorithm']=='pinned_yolox_nano_depthwise_cpu_image_triangle_argmax_classwise' and p['model_profile']=='yolox-nano-depthwise-v1','distinct lowered detector identity')
        require(p['compiled_sources']==expected() and p['model_profile_sha256']==digest(HERE/'profile-depthwise.json'),'complete detector receipt')
        require(p['model_sha256']==MODEL_SHA and p['input_shape']==[1,3,416,416] and p['output_shape']==[1,3549,85],'original Nano weights/tensor geometry')
        require(p['width']==view['rectangle']['width'] and p['height']==view['rectangle']['height'] and nano.exact_f32(p['score_threshold'],.3) and nano.exact_f32(p['nms_iou_threshold'],.45),'local geometry and thresholds')
        require(all(math.isfinite(p[k]) and p[k]>0 for k in ('model_load_seconds','inference_seconds')),'actual inference/load observations')
    fixture=dict(width=base['width'],height=base['height'],views=[dict(rectangle=v['rectangle'],detections=v['prediction']['detections']) for v in base['views']])
    nano.tiles.same(nano.tiles.merge(fixture),{k:base[k] for k in ('detections','detection_origins','merge')})
    nano.tiles.same(nano.consistency.oracle(fixture,nano.tiles),actual['result'])
    execution=actual['execution']
    require({k:v for k,v in execution.items() if k!='analysis_wall_s'}==dict(model_read=True,images_read=1,model_forwards=4,inference_run=True,view_workers=4,shared_model_plans=1),'bounded original concurrent views and actual work count')
    require(execution['analysis_wall_s']>=max(v['prediction']['inference_seconds'] for v in base['views']),'whole analyzer covers every forward')
    if serial is not None:
        exclude={'algorithm','compiled_sources','model_profile','model_profile_sha256','model_load_seconds','inference_seconds'}
        for view,reference in zip(base['views'],serial['base_prediction']['views']):
            compare({k:v for k,v in reference['prediction'].items() if k not in exclude},{k:v for k,v in view['prediction'].items() if k not in exclude},'every current original per-view f32 measurement bit')
        for key in ('detections','detection_origins','merge'):compare(serial['base_prediction'][key],base[key],'current original global merge')
        compare(serial['result'],actual['result'],'every current original final hypothesis/decision bit')

def audit_supervised(report, source, inspection, references=None):
    require(report['schema_version'] == 1 and report['algorithm'] == 'yolox_nano_depthwise_supervised_replay_v1', 'distinct supervised producer')
    require(report['compiled_sources'] == expected() and report['model_sha256'] == nano.MODEL_SHA, 'complete compiled source and model binding')
    require(report['policy'] == freshness.POLICY and report['control_permission'] is False, 'original recovery and age policy')
    require((report['poll_period_s'],report['in_flight_capacity'],report['pending_image_capacity']) == (.01,1,1), 'bounded work and declared poll target')
    require(report['clock'] == 'std::time::Instant_supervised_replay', 'measured clock identity')
    require(inspection['algorithm'] == 'yolox_nano_depthwise_schedule_validation_v1' and inspection['compiled_sources'] == expected()
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
                check_image(analysis,golden,serial)
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

def controls(args):
    # Same complete source/model controls; the native FIFO report shape is unchanged.
    latency.controls(args)
    out=args.output; rows=[]
    with tempfile.TemporaryDirectory(prefix='depthwise-controls-') as directory:
        fake=Path(directory)/'zero.onnx'
        with fake.open('xb') as stream:stream.truncate(3_659_407)
        link=Path(directory)/'link.onnx';link.symlink_to(fake)
        for name,model,error in [('wrong-S-model',args.s_model,'model size differs'),('wrong-nano-hash',fake,'model SHA-256 differs'),('model-symlink',link,'regular non-symlink file')]:
            rows.append(dict(name=name,**native(args.binary,['--model',model,'--sequence',out/'valid.json'],out/(name+'-unexpected.json'),2,error)))
    for flag in ('--image','--profile-image'):
        rows.append(dict(name=flag,**native(args.binary,['--model','missing',flag,'missing'],out/(flag[2:]+'-unexpected.json'),2,'read image:')))
    rows.append(dict(name='unknown-cli',**native(args.binary,['--unknown'],out/'unknown-unexpected.json',2,'expected --model')))
    empty,_=native(args.binary,['--model','missing','--supervised-sequence',out/'no-images.json'],out/'no-images-supervised.json')
    # Read an actual complete native inspection rather than manufacture its calibration.
    inspection,_=native(args.binary,['--inspect-sequence',out/'no-images.json'],out/'no-images-supervised-inspection.json')
    manifest=audit_supervised(empty,load(out/'no-images.json'),inspection)
    write(out/'no-images-supervised-guard-input.json',manifest)
    guard,_=native(args.guard,['--sequence',out/'no-images-supervised-guard-input.json'],out/'no-images-supervised-guard-result.json')
    require(guard['results']==[r['change']['output'] for r in empty['trace'] if r['change']['kind']=='guard'], 'model-free exact independent guard replay')
    write(out/'depthwise-controls.json',dict(pass_checks=True,shared_rejected_inputs=21,additional_rejections=rows,output_protections=2,
        valid_model_forwards=0,model_free_guard_replays=1,executable_sha256=digest(args.binary),checker_sha256=digest(__file__)))
    print(json.dumps(dict(pass_checks=True,rejected_inputs=27,valid_model_forwards=0)),flush=True)


def run(args):
    out=fresh(args.output);check_sources()
    control=load(args.controls/'depthwise-controls.json')
    require(control['pass_checks'] and control['valid_model_forwards']==0 and control['executable_sha256']==digest(args.binary),'zero-forward input controls bind the candidate')
    source_path=ROOT/'assets/camera-sequence-v1/sequence.json';source=load(source_path)
    require(digest(source_path)==nano.previous.SOURCE_SHA and digest(args.model)==MODEL_SHA,'original acquisition schedule and pinned weights')
    selection=[(f'tum-{i}',source_path.parent/e['input']['path'],ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json') for i,e in enumerate(source['events'])]
    for row in nano.matcher.protocol()['images']:
        name=row['id'];selection.append((name,ROOT/'artifacts/bdd-camera'/(name+'.jpg'),ROOT/f'assets/camera-analysis-v1/publication-repair/regression/first/{name}.json'))
    require(len(selection)==16 and all(digest(p)==load(saved)['source_image_sha256'] for _,p,saved in selection),'same sixteen viewed images')
    binaries=dict(baseline=args.baseline,optimized=args.binary);receipts={};inspections={}
    for mode,binary in binaries.items():
        receipts[mode],_=native(binary,['--executable-receipt'],out/(mode+'-executable.json'))
        require(receipts[mode]['executable_sha256']==digest(binary) and receipts[mode]['compiled_sources']==(expected() if mode=='optimized' else previous.expected()),'complete actual native binary/source receipt')
        inspections[mode],_=native(binary,['--inspect-sequence',source_path],out/(mode+'-inspection.json'))
        if mode=='optimized':variant(receipts[mode]);variant(inspections[mode])
    pins=dict(load(ROOT/'assets/camera-depthwise-v1/source-sha256.json'))
    pins.update({name:digest(ROOT/name) for name in NEW+['integrations/onnx/camera_rowzip_check.py','integrations/onnx/test_camera_rowzip.py']})
    require(all(digest(ROOT/name)==pin for name,pin in pins.items()),'all source pins before candidate inference')
    protocol=dict(source_sha256=pins,source_sequence_sha256=digest(source_path),model_sha256=MODEL_SHA,order=ORDER,schedulers=['fifo','supervised'],
        executable_receipts=receipts,serial_executable_sha256=digest(args.serial_binary),guard_executable_sha256=digest(args.guard),
        prior_candidate_model_forwards=0,additional_inference_warmup=False,all_timed_trials_before_profiles=True,policy=freshness.POLICY,
        image_selection=[dict(name=name,image_sha256=digest(p),saved_report=str(saved.relative_to(ROOT)),saved_report_sha256=digest(saved),
            profile_order=['baseline','optimized'] if i%2==0 else ['optimized','baseline'],reference_first=i%2==1) for i,(name,p,saved) in enumerate(selection)],
        execution_environment=json.loads(args.environment.read_text()),new_holdout=False,new_accuracy_scoring=False,driving_integration=False,
        timing_success_required_for_execution=False,trial_retention='Keep every first/repeat observation, including all regressions; no timing-based retries')
    write(out/'before-inference.json',protocol);shutil.copyfile(__file__,out/'initial-checker.py')
    trials=[]
    for scheduler in ('fifo','supervised'):
        folder=fresh(out/scheduler)
        for label in ORDER:
            mode='optimized' if label.startswith('optimized') else 'baseline'
            flag='--supervised-sequence' if scheduler=='supervised' else '--sequence'
            r,wall=native(binaries[mode],['--model',args.model,flag,source_path],folder/(label+'.json'))
            if mode=='optimized':variant(r)
            t=dict(scheduler=scheduler,label=label,model_forwards=r['model_forwards'],summary=r['summary'],external_wall_s=wall)
            write(folder/(label+'-trial.json'),t);trials.append(t)
            print(json.dumps(dict(scheduler=scheduler,label=label,model_forwards=r['model_forwards'])),flush=True)
    write(out/'trials.json',trials)
    fresh(out/'references');fresh(out/'profiles');fresh(out/'images');references={};pairs=[]
    for row,(name,image,saved) in zip(protocol['image_selection'],selection):
        reference,_=native(args.serial_binary,['--model',args.model,'--serial-image',image],out/'references'/(name+'.json'))
        nano.check_image(reference,load(saved),parallel=False);references[name]=reference
        profiles={}
        for mode in row['profile_order']:
            flags=['--model',args.model,'--profile-image',image]+(['--reference-first'] if row['reference_first'] else [])
            profile,_=native(binaries[mode],flags,out/'profiles'/(name+'-'+mode+'.json'))
            if mode=='optimized':variant(profile)
            audit_profile(profile,reference,expected() if mode=='optimized' else previous.expected(),True,row['reference_first'])
            profiles[mode]=profile
        for old,new in zip(profiles['baseline']['views'],profiles['optimized']['views']):
            require(old['input_tensor_f32le_sha256']==new['input_tensor_f32le_sha256'] and old['output_tensor_f32le_sha256']==new['output_tensor_f32le_sha256'], 'same current input and every current baseline/candidate raw output bit')
        result,_=native(args.binary,['--model',args.model,'--image',image],out/'images'/(name+'.json'))
        variant(result);check_image(result,load(saved),reference)
        times={mode:sum(v['profile']['forward_wall_ns'] for v in profile['views']) for mode,profile in profiles.items()}
        pairs.append(dict(name=name,profile_order=row['profile_order'],reference_first=row['reference_first'],baseline_observed_ns=times['baseline'],
            optimized_observed_ns=times['optimized'],reduction_fraction=1-times['optimized']/times['baseline']))
        print('Audited '+name,flush=True)
    by_sha={source['events'][i]['input']['sha256']:references[f'tum-{i}'] for i in range(8)};summaries=[]
    for t in trials:
        scheduler,label=t['scheduler'],t['label'];optimized=label.startswith('optimized');mode='optimized' if optimized else 'baseline'
        r=load(out/scheduler/(label+'.json'))
        require(r['compiled_sources']==receipts[mode]['compiled_sources'] and r['source_sequence_sha256']==digest(source_path),'timed source and acquisition bindings')
        require(r['startup_before_schedule_wall_s']+r['schedule_wall_s']<=t['external_wall_s']+.01,'external timer covers startup and all schedule work')
        if scheduler=='supervised':
            manifest=(audit_supervised if optimized else previous.audit_supervised)(r,source,inspections[mode],by_sha)
            outputs=[x['change']['output'] for x in r['trace'] if x['change']['kind']=='guard']
            available=r['summary']['available_result_images'];inferred=r['summary']['completed_images']
            times=[x['change']['worker_finished_s']-x['change']['worker_started_s'] for x in r['trace'] if x['change']['kind']=='finished']
            poll_gap=r['summary']['max_observed_poll_gap_s']
        else:
            manifest,_,_=audit_fifo(r,source,inspections[mode],freshness,tracking)
            for x in r['events']:
                a=x['measured']['analysis'];index=x['source']['event_index']
                if a is not None:(check_image if optimized else previous.check_image)(a,load(selection[index][2]),references[f'tum-{index}'])
            outputs=[e['output'] for x in r['events'] for e in x['measured']['guard_events']]
            available=r['summary']['available_at_check_images'];inferred=r['summary']['inferred_images']
            times=[x['measured']['timing']['read_analysis_and_probe_wall_s'] for x in r['events'] if x['measured']['analysis'] is not None];poll_gap=None
        write(out/scheduler/(label+'-guard-input.json'),manifest)
        guard,_=native(args.guard,['--sequence',out/scheduler/(label+'-guard-input.json')],out/scheduler/(label+'-guard-result.json'))
        require(guard['results']==outputs,'all standalone native guard results exactly replayed')
        summaries.append(dict(scheduler=scheduler,label=label,source_images=8,model_forwards=r['model_forwards'],inferred_images=inferred,available_results=available,
            inferred_but_late_images=r['summary']['inferred_but_late_images'],processing_s=times,max_observed_poll_gap_s=poll_gap))
    require(all(digest(ROOT/n)==pin for n,pin in pins.items()) and all(digest(binaries[k])==r['executable_sha256'] for k,r in receipts.items()), 'source and native executables unchanged throughout all candidate inference')
    require(digest(args.serial_binary)==protocol['serial_executable_sha256'] and digest(args.guard)==protocol['guard_executable_sha256'],'unchanged independent executables')
    timed=sum(t['model_forwards'] for t in trials)
    result=dict(pass_checks=True,comparison=summaries,serial_profile_pairs=pairs,
        serial_profile_aggregate_reduction_fraction=1-sum(p['optimized_observed_ns'] for p in pairs)/sum(p['baseline_observed_ns'] for p in pairs),
        serial_faster_pairs=sum(p['reduction_fraction']>0 for p in pairs),timed_model_forwards=timed,paired_profile_model_forwards=256,parallel_image_model_forwards=64,current_serial_model_forwards=64,
        actual_model_forwards=timed+384,raw_candidate_vs_ordinary_pairs=64,raw_baseline_vs_ordinary_pairs=64,current_baseline_vs_candidate_pairs=64,
        total_f32_bit_comparisons=192*301665,independent_guard_replays=8,full_current_measurement_bits_exact=True,
        new_holdout=False,new_accuracy_scoring=False,driving_integration=False,hard_real_time_guarantee=False,checker_sha256=digest(__file__))
    write(out/'summary.json',result);print(json.dumps(result),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-rowzip')
    parser.add_argument('--baseline',type=Path,default=HERE/'target/release/rustdriving-camera-depthwise')
    parser.add_argument('--serial-binary',type=Path,default=HERE/'target/release/rustdriving-camera-nano')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model/yolox_nano.onnx')
    parser.add_argument('--s-model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--environment',type=Path,default=ROOT/'artifacts/camera-rowzip-v1/container-environment.json')
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();check_sources();(controls if args.mode=='controls' else run)(args)
