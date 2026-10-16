#!/usr/bin/env python3
"""Audit unchanged detections and paired serial/parallel host timings.

Uses the standard library. Timing success, deadline success and unchanged
previous accuracy are separate findings; the fixed age limit is never tuned.
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
import statistics
import struct
import subprocess
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
LATENCY_CHECK_SHA='317011d9c10a6aaa4a01ce94685a97d42d2dc43f5488e512a77838ec1de2a556'
SOURCE_SHA='5fa15877e7d910fe102dae175af40b5ee473c6f614b0740f36127b58b77f864c'
MODEL_SHA='c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'
ORDER=['serial-first','parallel-first','parallel-repeat','serial-repeat']


def require(condition,message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def load(path):
    require(path.stat().st_size<=16*1024*1024,'bounded result')
    return json.loads(path.read_text())


def write(path,value):
    with path.open('x') as stream:
        json.dump(value,stream,indent=2,allow_nan=False)
        stream.write('\n')


def fresh(path):
    require(not path.exists() and not path.is_symlink(),'fresh evidence directory')
    path.mkdir(parents=True)
    return path


def support():
    path=HERE/'camera_latency_check.py'
    require(digest(path)==LATENCY_CHECK_SHA,'unchanged measured-replay auditor')
    spec=importlib.util.spec_from_file_location('parallel_latency_support',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    freshness,tracking=module.oracle()
    return module,freshness,tracking


def stripped(value):
    if isinstance(value,dict):
        return {k:stripped(v) for k,v in value.items() if k not in ('inference_seconds','model_load_seconds','analysis_wall_s')}
    if isinstance(value,list):
        return list(map(stripped,value))
    return value


def parallel_analysis(actual,serial):
    require(actual['schema_version']==1 and actual['algorithm']=='yolox_s_parallel_image_analysis_v1','distinct parallel producer identity')
    require(actual['source_image_sha256']==serial['source_image_sha256'],'original encoded-image identity')
    expected_keys=load(HERE/'profile-parallel.json')
    require(expected_keys['bounds']['view_workers']==4,'fixed worker budget')
    for name,pin in actual['compiled_sources'].items():
        require(digest(ROOT/name)==pin,'parallel producer source receipt')
    base=actual['base_prediction']
    require(base['algorithm']=='pinned_yolox_s_parallel_fixed_views_v1' and base['compiled_sources']==actual['compiled_sources'],'parallel base provenance')
    require(actual['result']==serial['result'],'every final hypothesis, origin, selection and work count unchanged')
    project=lambda b:{k:stripped(v) for k,v in b.items() if k not in ('algorithm','compiled_sources')}
    require(project(base)==project(serial['base_prediction']),'all per-view pixels/model measurements/order and merge unchanged')
    execution=actual['execution']
    n=len(base['views'])
    require({k:v for k,v in execution.items() if k!='analysis_wall_s'}==dict(model_read=True,images_read=1,model_forwards=n,
        inference_run=True,view_workers=n,shared_model_plans=1),'one plan, fixed original views, bounded workers')
    require(1<=n<=4 and math.isfinite(execution['analysis_wall_s']) and execution['analysis_wall_s']>0,'bounded observed analysis time')
    longest=max(v['prediction']['inference_seconds'] for v in base['views'])
    require(execution['analysis_wall_s']>=longest,'parallel wall time covers every individual forward')


def controls(args):
    module,_,_=support()
    module.controls(args)
    out=args.output
    failures=[]
    for name,image,model,error in [
        ('missing-image',out/'inputs/absent.png',out/'inputs/absent.onnx','read image:'),
        ('corrupt-image',out/'inputs/invalid.png',out/'inputs/absent.onnx','read image:'),
        ('missing-image-model',out/'inputs/0.png',out/'inputs/absent.onnx','load model:'),
        ('invalid-image-model',out/'inputs/0.png',out/'inputs/invalid.onnx','load model:'),
    ]:
        failures.append(dict(name=name,**module.native(args.binary,['--model',model,'--image',image],out/(name+'-unexpected.json'),2,error)))
    write(out/'parallel-summary.json',dict(pass_checks=True,reused_timing_controls=21,image_entry_rejections=failures,
        existing_output_protections=2,checker_sha256=digest(__file__),shared_checker_sha256=LATENCY_CHECK_SHA,
        executable_sha256=digest(args.binary)))


def benchmark(args,directory,module):
    fresh(directory)
    source=ROOT/'assets/camera-sequence-v1/sequence.json'
    binaries=dict(serial=args.serial,parallel=args.binary)
    receipts={}
    for mode,binary in binaries.items():
        receipts[mode],_=module.native(binary,['--executable-receipt'],directory/(mode+'-executable.json'))
    write(directory/'before-inference.json',dict(source_sequence_sha256=SOURCE_SHA,model_sha256=MODEL_SHA,order=ORDER,
        policy=dict(max_age_s=.2,recovery_samples=2),executable_receipts=receipts,
        image_parity_selection='All eight previously viewed TUM images and all eight previously viewed BDD images; no new subset selection',
        view_geometry_thresholds_model_unchanged=True,host_timing_results_may_vary=True))
    trials=[]
    for label in ORDER:
        mode=label.split('-')[0]
        print('Timing '+label,flush=True)
        report,wall=module.native(binaries[mode],['--model',args.model,'--sequence',source],directory/(label+'.json'))
        trials.append(dict(label=label,external_wall_s=wall,summary=report['summary'],
            processing_s=[e['measured']['timing']['read_analysis_and_probe_wall_s'] for e in report['events'] if e['measured']['analysis'] is not None],
            actual_model_forwards=report['model_forwards']))
    write(directory/'trials.json',trials)


def audit_parallel(report,source,inspection,freshness,tracking):
    require(report['algorithm']=='yolox_s_measured_serial_replay_v1' and report['clock']=='std::time::Instant_serial_replay','unchanged measured controller')
    require(report['policy']==freshness.POLICY and report['control_permission'] is False,'unchanged age/recovery policy')
    require(len(report['events'])==len(source['events'])==8,'all original scheduled images retained')
    origin=float(source['events'][0]['timestamp_s'])
    previous=None
    events,outputs=[],[]
    counts=dict(image_events=0,inferred_images=0,skipped_images=0,inferred_but_late_images=0,late_at_check_images=0,available_at_check_images=0)
    ages=[]
    for index,(original,row,slot) in enumerate(zip(source['events'],report['events'],inspection['slots'])):
        require(row['source']==slot and slot['event_index']==index and slot['source_timestamp_text']==original['timestamp_s'],'source identity and order')
        source_s=float(original['timestamp_s'])-origin
        require(slot['release_offset_s']==source_s,'original acquisition intervals')
        measured=row['measured']
        timing=measured['timing']
        start,submit,check=[timing[k] for k in ('processing_started_s','sample_submitted_s','checked_after_tracking_s')]
        require(all(math.isfinite(v) for v in (start,submit,check)) and start>=source_s and submit-start>=1e-6 and check-submit>=1e-6,'actual host checkpoint order')
        if previous is not None:
            require(start-previous['now_s']>=1e-6,'serial frame clock advances')
        require(timing['read_analysis_and_probe_wall_s']==submit-start and timing['sample_guard_wall_s']==check-submit,'measured stage arithmetic')
        require(timing['source_age_at_start_s']==start-source_s and timing['source_age_at_check_s']==check-source_s
                and timing['acquisition_deadline_s']==source_s+.2,'independent source age/deadline')
        missed=check>source_s+.2
        require(timing['deadline_missed_at_check']==missed,'same inclusive age gate')
        entries=measured['guard_events']
        require(len(entries)==2 and entries[0]['event']['now_s']==submit and entries[1]['event']==dict(now_s=check,input=dict(kind='tick')),'post-tracking guard check')
        data=entries[0]['event']['input']
        require(data['kind']=='sample' and data['acquired_s']==source_s and data['image_sha256']==original['input']['sha256'],'image identity and acquisition enter guard')
        accepted=previous['last_accepted_acquired_s'] if previous else None
        barrier=previous['fault_epoch_s'] if previous else None
        armed=accepted is not None and (barrier is None or accepted>barrier)
        if armed and start>accepted+.2:
            barrier=accepted+.2
        decision=('before_fault' if barrier is not None and source_s<=barrier else
                  'out_of_order' if accepted is not None and source_s-accepted<1e-6 else
                  'stale_sample' if start>source_s+.2 else 'accepted')
        require(measured['precheck_decision']==decision,'independent admission at processing start')
        inferred=measured['analysis'] is not None
        require(inferred==(decision=='accepted'),'no inference for excluded work')
        if inferred:
            analysis=measured['analysis']
            f32=lambda v:struct.unpack('<f',struct.pack('<f',v))[0]
            boxes=[dict(class_index=d['class_index'],confidence=f32(d['confidence']),bbox=list(map(f32,d['bbox']))) for d in analysis['result']['detections']]
            require(data['detections']==boxes,'unchanged original f32 measurements')
            require(timing['read_analysis_and_probe_wall_s']>=analysis['execution']['analysis_wall_s'],'measured frame covers full parallel analyzer')
        else:
            require(data['detections']==[],'excluded image does not invent boxes')
        for entry in entries:
            event=copy.deepcopy(entry['event'])
            if event['input']['kind']=='sample':
                event['input']['kind']='detections'
            events.append(event)
            outputs.append(entry['output'])
        counts['image_events']+=1
        counts['inferred_images']+=inferred
        counts['skipped_images']+=not inferred
        counts['inferred_but_late_images']+=inferred and missed
        counts['late_at_check_images']+=missed
        counts['available_at_check_images']+=entries[1]['output']['ready']
        ages.append(check-source_s)
        previous=entries[-1]['output']
    require(report['summary']==counts|dict(maximum_source_age_at_check_s=max(ages),deadline_gate_pass=counts['late_at_check_images']==0),'independent deadline counts')
    require(report['model_loads']==1 and report['preflight_image_reads']==8 and report['inference_image_reads']==counts['inferred_images']
            and report['model_forwards']==counts['inferred_images']*4,'one plan and actual forward/read counts')
    require(report['startup_before_schedule_wall_s']>=report['manifest_and_preflight_wall_s']+report['model_load_wall_s']
            and report['schedule_wall_s']>=previous['now_s'],'startup/schedule accounting')
    manifest=dict(schema_version=1,camera_id=report['camera_id'],width=report['width'],height=report['height'],
        config=report['config'],policy=report['policy'],events=events)
    backend,snapshots=freshness.check_contract(manifest,events,outputs,tracking)
    return manifest,backend,snapshots


def run(args):
    out=fresh(args.output)
    module,freshness,tracking=support()
    control=load(args.controls/'parallel-summary.json')
    require(control['pass_checks'] and control['checker_sha256']==digest(__file__) and control['executable_sha256']==digest(args.binary),'matching parallel controls')
    source_path=ROOT/'assets/camera-sequence-v1/sequence.json'
    require(digest(source_path)==SOURCE_SHA and digest(args.model)==MODEL_SHA,'fixed source images and model')
    source=load(source_path)
    receipt,_=module.native(args.binary,['--executable-receipt'],out/'executable.json')
    source_pins=dict(receipt['compiled_sources'])
    for path in [Path(__file__),HERE/'camera_latency_check.py',ROOT/'integrations/camera-tracking/freshness_check.py',ROOT/'integrations/camera-tracking/check.py']:
        source_pins[str(path.relative_to(ROOT))]=digest(path)
    selection=[]
    for index,event in enumerate(source['events']):
        selection.append((f'tum-{index}',source_path.parent/event['input']['path'],ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{index}.json','TUM'))
    for saved in sorted((ROOT/'assets/camera-analysis-v1/publication-repair/regression/first').glob('*.json')):
        selection.append((saved.stem,args.bdd/(saved.stem+'.jpg'),saved,'BDD'))
    require(len(selection)==16,'all eight TUM and eight BDD images')
    for _,image,saved,_ in selection:
        require(digest(image)==load(saved)['source_image_sha256'],'immutable cached image bytes')
    write(out/'before-verification.json',dict(source_sha256=source_pins,executable_sha256=digest(args.binary),model_sha256=MODEL_SHA,
        image_selection=[dict(name=name,image_sha256=digest(image),saved_report_sha256=digest(saved),data=data) for name,image,saved,data in selection],
        original_age_limit_s=.2,thresholds_geometry_model_unchanged=True,viewed_development_data=True,independent_fresh_holdout=False))
    if args.benchmark is not None:
        shutil.copytree(args.benchmark,out/'benchmark')
    else:
        benchmark(args,out/'benchmark',module)
    directory=out/'benchmark'
    plan=load(directory/'before-inference.json')
    require(plan['order']==ORDER and plan['source_sequence_sha256']==SOURCE_SHA and plan['model_sha256']==MODEL_SHA
            and plan['policy']==freshness.POLICY,'fixed ABBA benchmark plan')
    for mode,binary in [('serial',args.serial),('parallel',args.binary)]:
        require(plan['executable_receipts'][mode]['executable_sha256']==digest(binary),'matching timed executable')
        for name,pin in plan['executable_receipts'][mode]['compiled_sources'].items():
            require(digest(ROOT/name)==pin,'matching timed sources')
    inspections={}
    for mode,binary in [('serial',args.serial),('parallel',args.binary)]:
        inspections[mode],_=module.native(binary,['--inspect-sequence',source_path],out/(mode+'-inspection.json'))
    trials=load(directory/'trials.json')
    require([r['label'] for r in trials]==ORDER,'complete benchmark order retained')
    benchmark_forwards=0
    for trial in trials:
        label=trial['label']
        mode=label.split('-')[0]
        report=load(directory/(label+'.json'))
        require(report['compiled_sources']==plan['executable_receipts'][mode]['compiled_sources'] and report['source_sequence_sha256']==SOURCE_SHA,'timed source receipts')
        require(report['startup_before_schedule_wall_s']+report['schedule_wall_s']<=trial['external_wall_s']+.01,'external process timer bounds reported time')
        if mode=='serial':
            manifest,_,_=module.audit(report,source,inspections[mode],tracking,freshness)
        else:
            manifest,_,_=audit_parallel(report,source,inspections[mode],freshness,tracking)
        for event in report['events']:
            analysis=event['measured']['analysis']
            if analysis is not None:
                saved=load(ROOT/f"assets/camera-sequence-v1/recorded/exports/first-{event['source']['event_index']}.json")
                if mode=='parallel':parallel_analysis(analysis,saved)
                else:require(module.without_timing(analysis)==module.without_timing(saved),'serial analyzer baseline unchanged')
        measured=[e['measured']['timing']['read_analysis_and_probe_wall_s'] for e in report['events'] if e['measured']['analysis'] is not None]
        require(trial['processing_s']==measured and trial['summary']==report['summary'] and trial['actual_model_forwards']==report['model_forwards'],'trial summary bound to raw output')
        path=out/(label+'-guard-input.json')
        write(path,manifest)
        guard,_=module.native(args.guard,['--sequence',path],out/(label+'-guard-result.json'))
        require(guard['results']==[d['output'] for e in report['events'] for d in e['measured']['guard_events']],'exact guard replay for actual timed result')
        benchmark_forwards+=report['model_forwards']
    reports=fresh(out/'parity')
    rows=[]
    parity_forwards=0
    for name,image,saved,data in selection:
        print('Comparing '+data+' '+name,flush=True)
        first,_=module.native(args.binary,['--model',args.model,'--image',image],reports/(name+'-first.json'))
        repeat,_=module.native(args.binary,['--model',args.model,'--image',image],reports/(name+'-repeat.json'))
        reference=load(saved)
        parallel_analysis(first,reference)
        parallel_analysis(repeat,reference)
        require(stripped(first)==stripped(repeat),'repeat exact except actual durations')
        rows.append(dict(name=name,data=data,complete_per_view_and_merge_parity=True,complete_consistency_result_parity=True,repeated_measurements_identical=True))
        parity_forwards+=first['execution']['model_forwards']+repeat['execution']['model_forwards']
    # New producer identity must not pass the previous pinned serial admission.
    new_report=reports/'tum-0-first.json'
    rejected=out/'legacy-profile-input.json'
    write(rejected,dict(schema_version=1,camera_id='profile-check',width=640,height=480,events=[
        dict(now_s=0.,input=dict(kind='analysis',acquired_s=0.,report=os.path.relpath(new_report,out),sha256=digest(new_report)))]))
    profile_error=module.native(args.guard,['--sequence',rejected],out/'legacy-profile-unexpected.json',2,'invalid pinned camera analysis report')
    serial=[r['processing_s'][0] for r in trials if r['label'].startswith('serial')]
    parallel=[r['processing_s'][0] for r in trials if r['label'].startswith('parallel')]
    require(len(serial)==len(parallel)==2,'two paired-order timings per implementation')
    ratio=statistics.mean(serial)/statistics.mean(parallel)
    quality=load(ROOT/'assets/camera-analysis-v1/publication-repair/regression/results.json')
    for name,pin in source_pins.items():
        require(digest(ROOT/name)==pin,'sources unchanged during verification')
    require(digest(args.binary)==receipt['executable_sha256'],'executable unchanged during verification')
    write(out/'summary.json',dict(pass_checks=True,benchmark_order=ORDER,serial_processing_s=serial,parallel_processing_s=parallel,
        mean_speedup=ratio,mean_processing_reduction_fraction=1-statistics.mean(parallel)/statistics.mean(serial),
        observed_parallel_faster=statistics.mean(parallel)<statistics.mean(serial),age_limit_s=.2,
        observed_deadline_gates={r['label']:r['summary']['deadline_gate_pass'] for r in trials},
        timing_speedup_required_for_execution_success=False,parity=rows,original_image_count=16,
        benchmark_actual_model_forwards=benchmark_forwards,parity_actual_model_forwards=parity_forwards,
        total_actual_model_forwards=benchmark_forwards+parity_forwards,all_view_pixels_and_detections_identical=True,
        all_merge_and_consistency_results_identical=True,legacy_serial_profile_rejection=profile_error,
        inherited_bdd_quality=quality['overall'],inherited_quality_gates_pass=quality['quality_gates']['pass'],
        quality_metrics_inherited_from_exact_parity=True,new_accuracy_improvement_measured=False,
        source_sha256=source_pins,executable_sha256=digest(args.binary),checker_sha256=digest(__file__)))
    print(json.dumps(dict(pass_checks=True,mean_speedup=ratio,parity_images=16,total_actual_model_forwards=benchmark_forwards+parity_forwards)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-parallel')
    parser.add_argument('--serial',type=Path,default=HERE/'target/release/rustdriving-camera-latency')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--bdd',type=Path,default=ROOT/'artifacts/bdd-camera')
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--benchmark',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    for key in ('binary','serial','guard','model','bdd','output'):
        setattr(args,key,getattr(args,key).absolute())
    if args.mode=='controls':controls(args)
    else:
        require(args.controls is not None,'--controls required')
        run(args)
