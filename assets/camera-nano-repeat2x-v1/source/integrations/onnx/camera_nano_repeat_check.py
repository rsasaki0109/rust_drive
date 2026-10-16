#!/usr/bin/env python3
"""Verify guarded Nano Resize lowering, every output bit and measured availability."""
import argparse
import ast
import collections
import copy
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import statistics
import struct
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result
nano=module('repeat_nano_support',HERE/'camera_nano_check.py')
latest=module('repeat_latest_support',HERE/'camera_latest_check.py')
clock=module('repeat_new_clocks',HERE/'camera_nano_repeat_clock.py')
operator=nano.operator
require,digest,load,write,fresh=nano.require,nano.digest,nano.load,nano.write,nano.fresh
latency,freshness,tracking=nano.latency,nano.freshness,nano.tracking
MODEL_SHA=nano.MODEL_SHA
measurement_bits=operator.measurement_bits
historical_detections=operator.historical_detections
NEW=['integrations/onnx/src/camera_nano_repeat2.rs','integrations/onnx/src/camera_nano_repeat_detector.rs',
     'integrations/onnx/src/camera_nano_repeat_analysis.rs','integrations/onnx/src/bin/rustdriving-camera-nano-repeat2x.rs','integrations/onnx/profile-nano-repeat2x.json']
PILOT=['integrations/onnx/src/camera_nano_profile.rs','integrations/onnx/src/bin/rustdriving-camera-nano-profile.rs','integrations/onnx/profile-nano-operators.json']
EXTRA=['integrations/onnx/src/camera_profile.rs','integrations/onnx/src/camera_repeat2.rs','integrations/onnx/src/bin/rustdriving-camera-profile.rs','integrations/onnx/profile-operators.json']
LATEST=['integrations/onnx/src/camera_latest.rs','integrations/onnx/src/bin/rustdriving-camera-latest.rs','integrations/onnx/profile-latest.json']
ORDER=['baseline-first','optimized-first','optimized-repeat','baseline-repeat']

def expected():return dict(nano.expected('runtime'),**{name:digest(ROOT/name) for name in NEW+PILOT+EXTRA+LATEST})

def compare(expected,actual,label,historical=None):
    require(historical is None,'no historical numerical tolerance for current comparisons')
    require(nano.result_bits(actual)==nano.result_bits(expected),label)

def check_sources():
    def fn(path,name):
        text=path.read_text();node=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    original=(HERE/'src/camera_profile.rs').read_text();current=(HERE/'src/camera_nano_profile.rs').read_text()
    a,b=original.split('#[cfg(test)]',1)
    require(current==a+'#[cfg(test)]'+b.replace('640','416'),'unchanged observer, Nano-sized input fixture only')
    original=(HERE/'src/camera_repeat2.rs').read_text()
    require((HERE/'src/camera_nano_repeat2.rs').read_text()==original.replace('("Resize_165", 256, 20, 20), ("Resize_187", 128, 40, 40)','("Resize_201", 128, 13, 13), ("Resize_226", 64, 26, 26)'),'same algebra and guards, only Nano node/shape pins differ')
    adapted=fn(HERE/'camera_nano_clock.py','audit_measured').replace('def audit_measured(', 'def audit_fifo(').replace('yolox_nano_measured_replay_v1','yolox_nano_repeat2x_measured_replay_v1')
    require(fn(HERE/'camera_nano_repeat_clock.py','audit_fifo')==adapted,'unchanged FIFO arithmetic and guard policy')
    adapted=fn(HERE/'camera_latest_check.py','audit').replace('def audit(report, source, inspection, references=None):','def audit_latest(report, source, inspection, references=None, expected=None, check_image=None):').replace('yolox_nano_latest_ready_replay_v1','yolox_nano_repeat2x_latest_ready_replay_v1').replace('yolox_nano_latest_ready_schedule_validation_v1','yolox_nano_repeat2x_schedule_validation_v1').replace('nano.check_image(', 'check_image(')
    require(fn(HERE/'camera_nano_repeat_clock.py','audit_latest')==adapted,'unchanged latest selection/clock/guard oracle with new provenance callbacks')
    adapted=fn(HERE/'camera_operator_check.py','audit').replace('def audit(', 'def audit_profile(').replace('yolox_s_repeat2x_operator_profile_v2','yolox_nano_repeat2x_operator_profile_v1').replace('yolox_s_cpu_operator_profile_v2','yolox_nano_cpu_operator_profile_v1').replace('previous.MODEL_SHA','MODEL_SHA').replace("report['score_threshold']==.3 and report['nms_iou_threshold']==.45","nano.exact_f32(report['score_threshold'],.3) and nano.exact_f32(report['nms_iou_threshold'],.45)").replace("view['output_values']==714000","view['output_values']==301665")
    require(fn(Path(__file__),'audit_profile')==adapted,'same operator-clock and full-bit audit with Nano identities, shape and exact threshold bits')
    latest.check_shared_controller()


def check_image(actual,saved,serial=None):
    require(actual['schema_version']==1 and actual['algorithm']=='yolox_nano_repeat2x_parallel_image_analysis_v1','distinct lowered image producer')
    require(actual['compiled_sources']==expected() and actual['model_sha256']==MODEL_SHA and actual['control_permission'] is False,'complete lowered image receipt and no control permission')
    require(actual['graph_optimization']=='validated_asymmetric_floor_nearest_2x_only','only guarded Resize lowering')
    base=actual['base_prediction'];geometry=saved['base_prediction']
    require(base['algorithm']=='pinned_yolox_nano_repeat2x_parallel_fixed_views_v1' and base['compiled_sources']==expected(),'distinct lowered base receipt')
    require(actual['source_image_sha256']==base['source_image_sha256']==saved['source_image_sha256'] and base['model_sha256']==MODEL_SHA,'original encoded image/model')
    require((base['width'],base['height'])==(geometry['width'],geometry['height']) and len(base['views'])==len(geometry['views'])==4,'all original views and dimensions')
    require(nano.exact_f32(base['score_threshold'],.3) and nano.exact_f32(base['nms_iou_threshold'],.45),'unchanged threshold bits')
    for view,old in zip(base['views'],geometry['views']):
        require(view['rectangle']==old['rectangle'] and view['rgb_sha256']==old['rgb_sha256'],'original ordered crop pixels')
        p=view['prediction']
        require(p['algorithm']=='pinned_yolox_nano_repeat2x_cpu_image_triangle_argmax_classwise' and p['model_profile']=='yolox-nano-repeat2x-v1','distinct lowered detector identity')
        require(p['compiled_sources']==expected() and p['model_profile_sha256']==digest(HERE/'profile-nano-repeat2x.json'),'complete detector receipt')
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


def audit_profile(report,saved,pins,repeat2x,reference_first,historical=None):
    require(report['schema_version']==2 and report['algorithm']==('yolox_nano_repeat2x_operator_profile_v1' if repeat2x else 'yolox_nano_cpu_operator_profile_v1'),'distinct diagnostic algorithm')
    require(report['compiled_sources']==pins,'complete compiled-source receipt')
    require(report['control_permission'] is False,'diagnosis cannot supply control permission')
    require(report['model_sha256']==MODEL_SHA and report['source_image_sha256']==saved['source_image_sha256'],'original weights and encoded image')
    base=saved['base_prediction'];n=len(base['views'])
    require((report['width'],report['height'])==(base['width'],base['height']) and nano.exact_f32(report['score_threshold'],.3) and nano.exact_f32(report['nms_iou_threshold'],.45),'unchanged geometry/thresholds')
    execution=report['execution'];nodes=execution['graph_nodes']
    require(type(nodes) is int and 1<=nodes<=4096,'bounded graph')
    expected=dict(model_loads=1,model_plans=2 if repeat2x else 1,images_read=1,graph_nodes=nodes,
        profiled_forwards=n,ordinary_reference_forwards=n,model_forwards=n*2,view_workers=1,resizes_rewritten=2 if repeat2x else 0,
        order='reference_then_profile_per_view' if reference_first else 'profile_then_reference_per_view',
        timing_purpose='serial_operator_hotspots_not_deadline_measurement')
    require(execution==expected,'actual work inventory and fixed execution order')
    require(len(report['views'])==n,'complete original view inventory')
    for view,original in zip(report['views'],base['views']):
        require(view['rectangle']==original['rectangle'] and view['rgb_sha256']==original['rgb_sha256'],'original crop pixels')
        require(measurement_bits(view['detections'])==measurement_bits(view['reference_detections']),'every current decoded class/box/confidence f32 bit/order unchanged')
        if historical is None:
            require(measurement_bits(view['reference_detections'])==measurement_bits(original['prediction']['detections']),'historical detections exact by default')
        else:
            historical.same(historical_detections(original['prediction']['detections']),historical_detections(view['reference_detections']),'views.'+str(view['rectangle'])+'.reference_detections')
        for field in ('input_tensor_f32le_sha256','output_tensor_f32le_sha256','reference_output_tensor_f32le_sha256'):
            require(re.fullmatch('[0-9a-f]{64}',view[field]) is not None,'canonical float hash')
        require(view['output_bitwise_equal'] is True and view['output_values']==301665
                and view['output_tensor_f32le_sha256']==view['reference_output_tensor_f32le_sha256'],'every raw output bit verified')
        profile=view['profile'];wall=profile['forward_wall_ns']
        require(type(wall) is int and wall>0 and type(view['reference_forward_wall_ns']) is int and view['reference_forward_wall_ns']>0,'observed nanosecond durations')
        rows=profile['operators'];require(0<len(rows)<=nodes,'bounded operator observations')
        seen=set();end=0;group_ns=collections.Counter();group_calls=collections.Counter()
        for row in rows:
            node=row['node_id'];begin=row['started_ns'];finish=row['finished_ns'];elapsed=row['elapsed_ns']
            require(all(type(v) is int for v in (node,begin,finish,elapsed)) and 0<=node<nodes and node not in seen,'unique graph node observations')
            require(end<=begin<=finish<=wall and elapsed==finish-begin,'serial nonoverlapping actual clock arithmetic')
            seen.add(node);end=finish
            group_ns[row['operation']]+=elapsed;group_calls[row['operation']]+=1
        total=sum(group_ns.values())
        require(profile['operator_sum_ns']==total and profile['unattributed_ns']==wall-total,'full wall-time accounting')
        expected_groups=[dict(operation=name,calls=group_calls[name],elapsed_ns=group_ns[name]) for name in sorted(group_ns,key=lambda k:(-group_ns[k],k))]
        require(profile['by_operation']==expected_groups,'independent operation regrouping')
        if repeat2x:
            require(not group_calls['Resize'],'original generic Resize excluded after guarded lowering')
            require(sum(r['operation']=='MultiBroadcastTo' and r['node_name'].endswith('.repeat2') for r in rows)==2,'both standard repeat broadcasts executed')
        else:
            require(group_calls['Resize']==2,'both original Resize calls observed')
    require(report['merged']==report['reference_merged'],'exact current-host merge')
    require(report['result']==report['reference_result'],'exact current-host final decisions and results')
    actual_merge=dict(report['reference_merged'],detections=historical_detections(report['reference_merged']['detections']))
    expected_merge={k:base[k] for k in ('detections','detection_origins','merge')}
    expected_merge['detections']=historical_detections(expected_merge['detections'])
    compare(expected_merge,actual_merge,'historical original merge',historical)
    compare(saved['result'],report['reference_result'],'historical original result',historical)


def controls(args):
    check_sources()
    latency.controls(args)
    out=args.output; rows=[]
    with tempfile.TemporaryDirectory(prefix='nano-repeat-controls-') as directory:
        fake=Path(directory)/'bad.onnx'
        with fake.open('xb') as stream:stream.truncate(3_659_407)
        link=Path(directory)/'link.onnx';link.symlink_to(fake)
        for name,model,error in [('wrong-model',args.s_model,'model size differs'),('wrong-hash',fake,'model SHA-256 differs'),('model-symlink',link,'regular non-symlink file')]:
            rows.append(dict(name=name,**latency.native(args.binary,['--model',model,'--sequence',out/'valid.json'],out/(name+'-unexpected.json'),2,error)))
        rows.append(dict(name='profile-rejects-wrong-model',**latency.native(args.binary,['--model',fake,'--profile-image',out/'inputs/0.png'],out/'profile-unexpected.json',2,'model SHA-256 differs')))
    rows.append(dict(name='unknown-cli',**latency.native(args.binary,['--unknown'],out/'cli-unexpected.json',2,'expected --model')))
    value,_=latency.native(args.binary,['--model','missing-model','--latest-sequence',out/'no-images.json'],out/'latest-no-images-result.json')
    inspection,_=latency.native(args.binary,['--inspect-sequence',out/'no-images.json'],out/'latest-no-images-inspection.json')
    manifest=clock.audit_latest(value,load(out/'no-images.json'),inspection,expected=expected,check_image=check_image)
    write(out/'latest-no-images-guard-input.json',manifest)
    guard,_=latency.native(args.guard,['--sequence',out/'latest-no-images-guard-input.json'],out/'latest-no-images-guard-result.json')
    require(guard['results']==[entry['output'] for row in value['events'] for entry in row['measured']['guard_events']],'latest no-image exact independent guard replay')
    receipt=load(out/'executable.json');require(receipt['compiled_sources']==expected(),'complete 55-source receipt')
    write(out/'nano-repeat-summary.json',dict(pass_checks=True,common_rejected_inputs=21,additional_rejections=rows,output_protections=2,
        valid_model_forwards=0,checker_sha256=digest(__file__),clock_checker_sha256=digest(HERE/'camera_nano_repeat_clock.py'),executable_sha256=digest(args.binary)))


def run(args):
    out=fresh(args.output);check_sources()
    control=load(args.controls/'nano-repeat-summary.json')
    require(control['pass_checks'] and control['checker_sha256']==digest(__file__) and control['clock_checker_sha256']==digest(HERE/'camera_nano_repeat_clock.py')
            and control['executable_sha256']==digest(args.binary),'matching source-bound input controls')
    source_path=ROOT/'assets/camera-sequence-v1/sequence.json';source=load(source_path)
    require(digest(source_path)==nano.previous.SOURCE_SHA and digest(args.model)==MODEL_SHA,'unchanged acquisition schedule and original Nano weights')
    selection=[(f'tum-{i}',source_path.parent/event['input']['path'],ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json') for i,event in enumerate(source['events'])]
    selection += [(p.stem,args.bdd/(p.stem+'.jpg'),p) for p in sorted((ROOT/'assets/camera-analysis-v1/publication-repair/regression/first').glob('*.json'))]
    require(len(selection)==16 and all(digest(image)==load(saved)['source_image_sha256'] for _,image,saved in selection),'fixed complete sixteen original images')
    binaries=dict(baseline=args.baseline,latest=args.latest,optimized=args.binary)
    receipts={};inspections={}
    for mode,binary in binaries.items():
        receipts[mode],_=latency.native(binary,['--executable-receipt'],out/(mode+'-executable.json'))
        require(receipts[mode]['executable_sha256']==digest(binary),'actual executable binding')
        require(receipts[mode]['compiled_sources']==(expected() if mode=='optimized' else latest.expected() if mode=='latest' else nano.expected('runtime')),'complete native source closure')
        inspections[mode],_=latency.native(binary,['--inspect-sequence',source_path],out/(mode+'-inspection.json'))
    pins=dict(load(ROOT/'assets/camera-latest-v1/source-sha256.json'))
    pins.update({name:digest(ROOT/name) for name in NEW+PILOT+['integrations/onnx/camera_nano_repeat_check.py','integrations/onnx/camera_nano_repeat_clock.py']})
    require(all(digest(ROOT/name)==pin for name,pin in pins.items()),'all original and candidate sources fixed before timings')
    write(out/'before-inference.json',dict(source_sha256=pins,model_sha256=MODEL_SHA,source_sequence_sha256=nano.previous.SOURCE_SHA,
        executable_receipts=receipts,guard_executable_sha256=digest(args.guard),policy=freshness.POLICY,
        order_by_scheduler=dict(fifo=ORDER,latest=ORDER),scheduler_order=['fifo','latest'],inference_warmup=False,new_holdout=False,
        image_selection=[dict(name=name,image_sha256=digest(image),reference_report_sha256=digest(saved)) for name,image,saved in selection]))
    trials=[]
    for scheduler in ('fifo','latest'):
        fresh(out/scheduler)
        for label in ORDER:
            optimized=label.startswith('optimized');mode='optimized' if optimized else 'baseline' if scheduler=='fifo' else 'latest'
            flag='--latest-sequence' if scheduler=='latest' and optimized else '--sequence'
            print('Measured '+scheduler+' '+label,flush=True)
            report,wall=latency.native(binaries[mode],['--model',args.model,flag,source_path],out/scheduler/(label+'.json'))
            trial=dict(scheduler=scheduler,label=label,external_wall_s=wall,model_forwards=report['model_forwards'],summary=report['summary'])
            write(out/scheduler/(label+'-trial.json'),trial);trials.append(trial)
    write(out/'trials.json',trials)
    references=fresh(out/'references');profiles=fresh(out/'profiles');images=fresh(out/'images')
    current={};profile_count=0;parallel_count=0
    for name,image,saved in selection:
        print('Exact original/optimized raw outputs and parallel results '+name,flush=True)
        serial,_=latency.native(args.baseline,['--model',args.model,'--serial-image',image],references/(name+'.json'))
        nano.check_image(serial,load(saved),parallel=False);current[name]=serial
        numeric=[];parallel=[]
        for repeat,reference_first in [('first',False),('repeat',True)]:
            flags=['--model',args.model,'--profile-image',image]+(['--reference-first'] if reference_first else [])
            profile,_=latency.native(args.binary,flags,profiles/(name+'-'+repeat+'.json'))
            audit_profile(profile,serial,expected(),True,reference_first)
            numeric.append(operator.numerical_projection(profile));profile_count+=profile['execution']['model_forwards']
            result,_=latency.native(args.binary,['--model',args.model,'--image',image],images/(name+'-'+repeat+'.json'))
            check_image(result,load(saved),serial);parallel_count+=result['execution']['model_forwards']
            parallel.append(nano.previous.stripped(result))
        require(numeric[0]==numeric[1],'all repeated input/output bits, measurements, merges and decisions identical')
        require(parallel[0]==parallel[1],'complete repeated parallel outputs identical except durations')
    summaries=[]
    for trial in trials:
        scheduler=trial['scheduler'];label=trial['label'];optimized=label.startswith('optimized')
        mode='optimized' if optimized else 'baseline' if scheduler=='fifo' else 'latest'
        report=load(out/scheduler/(label+'.json'))
        require(report['compiled_sources']==receipts[mode]['compiled_sources'] and report['source_sequence_sha256']==nano.previous.SOURCE_SHA,'exact timed producer and acquisition source')
        require(report['startup_before_schedule_wall_s']+report['schedule_wall_s']<=trial['external_wall_s']+.01,'external clock covers all startup and schedule time')
        if scheduler=='fifo':
            manifest,_,_=(clock.audit_fifo if optimized else nano.clock.audit_measured)(report,source,inspections[mode],freshness,tracking)
        elif optimized:
            manifest=clock.audit_latest(report,source,inspections[mode],references=[current[f'tum-{i}'] for i in range(8)],expected=expected,check_image=check_image)
        else:manifest=latest.audit(report,source,inspections[mode],[current[f'tum-{i}'] for i in range(8)])
        for row in report['events']:
            if row['measured']['analysis'] is not None:
                i=row['source']['event_index'];saved=load(ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json')
                (check_image if optimized else nano.check_image)(row['measured']['analysis'],saved,current[f'tum-{i}'])
        path=out/scheduler/(label+'-guard-input.json');write(path,manifest)
        guard,_=latency.native(args.guard,['--sequence',path],out/scheduler/(label+'-guard-result.json'))
        require(guard['results']==[entry['output'] for row in report['events'] for entry in row['measured']['guard_events']],'complete exact independent native guard replay')
        s=report['summary']
        summaries.append(dict(scheduler=scheduler,label=label,source_images=8,inferred_images=s['inferred_images'],superseded_images=s.get('superseded_images',0),
            available_results=s['available_at_check_images'] if scheduler=='fifo' else s['available_at_dispatched_checks'],
            inferred_but_late_images=s['inferred_but_late_images'],processing_s=[row['measured']['timing']['read_analysis_and_probe_wall_s'] for row in report['events'] if row['measured']['analysis'] is not None]))
    require(all(digest(ROOT/name)==pin for name,pin in pins.items()) and all(digest(binaries[mode])==receipt['executable_sha256'] for mode,receipt in receipts.items()),'sources and executables unchanged throughout actual comparison')
    require(digest(args.guard)==load(out/'before-inference.json')['guard_executable_sha256'],'unchanged standalone guard executable')
    timed_count=sum(t['model_forwards'] for t in trials)
    reductions={}
    for scheduler in ('fifo','latest'):
        b=[s['processing_s'][0] for s in summaries if s['scheduler']==scheduler and s['label'].startswith('baseline')]
        o=[s['processing_s'][0] for s in summaries if s['scheduler']==scheduler and s['label'].startswith('optimized')]
        reductions[scheduler]=dict(baseline_first_image_s=b,optimized_first_image_s=o,mean_reduction_fraction=1-statistics.mean(o)/statistics.mean(b))
    summary=dict(pass_checks=True,comparison=summaries,first_image_timings=reductions,
        timed_model_forwards=timed_count,raw_profile_model_forwards=profile_count,parallel_model_forwards=parallel_count,
        current_serial_reference_model_forwards=64,actual_model_forwards=timed_count+profile_count+parallel_count+64,
        raw_outputs_per_view=301665,raw_bitwise_pairs=128,original_images=16,repeats=2,exact_guard_replays=8,
        fresh_quality_scoring=False,inherited_nano_quality=load(ROOT/'assets/camera-nano-v1/summary.json').get('quality'),
        new_holdout=False,driving_integration=False,deadline_or_quality_success_required_for_execution=False,
        checker_sha256=digest(__file__),clock_checker_sha256=digest(HERE/'camera_nano_repeat_clock.py'))
    write(out/'summary.json',summary);print(json.dumps(summary),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-nano-repeat2x')
    parser.add_argument('--baseline',type=Path,default=HERE/'target/release/rustdriving-camera-nano')
    parser.add_argument('--latest',type=Path,default=HERE/'target/release/rustdriving-camera-latest')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model/yolox_nano.onnx')
    parser.add_argument('--s-model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--bdd',type=Path,default=ROOT/'artifacts/bdd-camera')
    parser.add_argument('--controls',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    for name in ('binary','baseline','latest','guard','model','s_model','bdd','output'):setattr(args,name,getattr(args,name).absolute())
    if args.mode=='controls':controls(args)
    else:require(args.controls is not None,'--controls required');run(args)
