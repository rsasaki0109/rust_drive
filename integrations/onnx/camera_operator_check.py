#!/usr/bin/env python3
"""Audit operator clocks and exact original-model output for the repeat2x pass."""
import argparse
import collections
import copy
import importlib.util
import json
from pathlib import Path
import re
import shutil
import statistics
import struct
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
def import_file(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result
previous=import_file('operator_parallel_support',HERE/'camera_parallel_check.py')
require,digest,load,write,fresh=previous.require,previous.digest,previous.load,previous.write,previous.fresh
require(digest(HERE/'camera_parallel_check.py')=='f9d4e081b4989f3ffe3bd40f7f030f817797188161370b636276c127ac90d109','pinned parallel auditor')
latency,_,_=previous.support()
NEW_SOURCES=['integrations/onnx/src/camera_profile.rs','integrations/onnx/src/camera_repeat2.rs',
             'integrations/onnx/src/bin/rustdriving-camera-profile.rs','integrations/onnx/profile-operators.json']


def expected_sources():
    result=load(ROOT/'assets/camera-sequence-v1/recorded/exports/first-0.json')['compiled_sources']
    result.update({name:digest(ROOT/name) for name in NEW_SOURCES})
    return result


def controls(args):
    out=fresh(args.output)
    cases=[]
    with tempfile.TemporaryDirectory(prefix='operator-input-') as temporary:
        inputs=Path(temporary)
        image=inputs/'valid.png'
        shutil.copy2(ROOT/'assets/camera-sequence-v1/controls/inputs/0.png',image)
        corrupt=inputs/'corrupt.png';corrupt.write_bytes(b'invalid PNG')
        model=inputs/'invalid.onnx';model.write_bytes(b'invalid model')
        link=inputs/'link.png';link.symlink_to(image)
        model_link=inputs/'model-link.onnx';model_link.symlink_to(model)
        wrong_hash=inputs/'wrong-hash.onnx'
        with wrong_hash.open('wb') as stream:stream.truncate(35_858_002)
        for name,img,weights,error in [
            ('missing-image',inputs/'absent.png',model,'read image:'),
            ('corrupt-image',corrupt,model,'read image:'),
            ('image-symlink',link,model,'read image:'),
            ('missing-model',image,inputs/'absent.onnx','load model:'),
            ('invalid-model-size',image,model,'model size differs'),
            ('invalid-model-hash',image,wrong_hash,'model SHA-256 differs'),
            ('model-symlink',image,model_link,'regular non-symlink file'),
        ]:
            cases.append(dict(name=name,**latency.native(args.binary,['--repeat2x','--model',weights,'--image',img],out/(name+'-unexpected.json'),2,error)))
        cases.append(dict(name='unknown-cli',**latency.native(args.binary,['--unknown'],out/'cli-unexpected.json',2,'expected [--repeat2x]')))
        existing=out/'existing.json';existing.write_bytes(b'preserve original output\n')
        directory=out/'existing-directory';directory.mkdir();(directory/'marker').write_bytes(b'preserve directory')
        for target in (existing,directory):
            before=(existing.read_bytes(),(directory/'marker').read_bytes())
            result=subprocess.run([str(args.binary),'--repeat2x','--model',str(inputs/'absent.onnx'),'--image',str(inputs/'absent.png'),'--output',str(target)],capture_output=True,text=True,timeout=10)
            require(result.returncode==2 and 'fresh output required before input reads' in result.stderr,'existing-output admission')
            require(before==(existing.read_bytes(),(directory/'marker').read_bytes()),'existing output preserved')
        require(not list(out.glob('.rustdriving-camera-*.partial')),'no leaked staging')
    write(out/'summary.json',dict(pass_checks=True,rejected_inputs=cases,existing_output_protections=2,
        executable_sha256=digest(args.binary),checker_sha256=digest(__file__)))


def measurement_bits(detections):
    # Saved base_prediction is serde_json::Value: it prints exact promoted f64
    # values. Typed diagnostic fields print shortest round-tripping f32 text.
    # Compare every represented f32 bit, without a numerical tolerance.
    bits=lambda v:struct.unpack('<I',struct.pack('<f',v))[0]
    return [dict(d,confidence=bits(d['confidence']),bbox=list(map(bits,d['bbox']))) for d in detections]


def historical_detections(detections):
    # Normalize typed f32 JSON and promoted f64 JSON to their represented f32.
    number=lambda value:struct.unpack('<f',struct.pack('<f',value))[0]
    return [dict(d,confidence=number(d['confidence']),bbox=list(map(number,d['bbox']))) for d in detections]


def compare(expected,actual,label,historical):
    if historical is None:require(actual==expected,label)
    else:historical.same(expected,actual,label)


def audit(report,saved,pins,repeat2x,reference_first,historical=None):
    require(report['schema_version']==2 and report['algorithm']==('yolox_s_repeat2x_operator_profile_v2' if repeat2x else 'yolox_s_cpu_operator_profile_v2'),'distinct diagnostic algorithm')
    require(report['compiled_sources']==pins,'complete compiled-source receipt')
    require(report['control_permission'] is False,'diagnosis cannot supply control permission')
    require(report['model_sha256']==previous.MODEL_SHA and report['source_image_sha256']==saved['source_image_sha256'],'original weights and encoded image')
    base=saved['base_prediction'];n=len(base['views'])
    require((report['width'],report['height'])==(base['width'],base['height']) and report['score_threshold']==.3 and report['nms_iou_threshold']==.45,'unchanged geometry/thresholds')
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
        require(view['output_bitwise_equal'] is True and view['output_values']==714000
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


def numerical_projection(report):
    return dict(views=[{k:v[k] for k in ('rectangle','rgb_sha256','input_tensor_f32le_sha256','output_tensor_f32le_sha256','reference_output_tensor_f32le_sha256','output_values','detections','reference_detections')} for v in report['views']],merged=report['merged'],result=report['result'],reference_merged=report['reference_merged'],reference_result=report['reference_result'])


def run(args):
    out=fresh(args.output)
    historical=latency.camera_history.Comparison()
    control=load(args.controls/'summary.json')
    require(control['pass_checks'] and control['checker_sha256']==digest(__file__) and control['executable_sha256']==digest(args.binary),'matching input controls')
    receipt,_=latency.native(args.binary,['--executable-receipt'],out/'executable.json')
    pins=expected_sources();require(receipt['compiled_sources']==pins,'complete 27-source executable receipt')
    require(receipt['executable_sha256']==digest(args.binary) and not receipt['model_read'] and receipt['model_forwards']==0,'model-free executable binding')
    source_path=ROOT/'assets/camera-sequence-v1/sequence.json'
    require(digest(source_path)==previous.SOURCE_SHA and digest(args.model)==previous.MODEL_SHA,'unchanged source schedule/model')
    source=load(source_path);selection=[]
    for i,event in enumerate(source['events']):
        selection.append((f'tum-{i}',source_path.parent/event['input']['path'],ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json','TUM'))
    for saved in sorted((ROOT/'assets/camera-analysis-v1/publication-repair/regression/first').glob('*.json')):
        selection.append((saved.stem,args.bdd/(saved.stem+'.jpg'),saved,'BDD'))
    require(len(selection)==16,'all original eight TUM and eight BDD images')
    for _,image,saved,_ in selection:require(digest(image)==load(saved)['source_image_sha256'],'original image bytes')
    all_pins=dict(pins)
    for path in (Path(__file__),HERE/'camera_history.py',HERE/'camera_parallel_check.py',HERE/'camera_latency_check.py'):
        all_pins[str(path.relative_to(ROOT))]=digest(path)
    write(out/'before-inference.json',dict(source_sha256=all_pins,executable_sha256=digest(args.binary),model_sha256=previous.MODEL_SHA,
        image_selection=[dict(name=name,data=data,image_sha256=digest(image),reference_report_sha256=digest(saved)) for name,image,saved,data in selection],
        baseline_profile='TUM image0 once using current unmodified-plan mode; one profiled and one ordinary forward per view',
        order=['optimized_profile_then_original_reference','original_reference_then_optimized_profile'],viewed_development_data=True,
        independent_fresh_holdout=False,deadline_measurement=False,new_accuracy_improvement_claimed=False))
    first_saved=load(selection[0][2])
    baseline,wall=latency.native(args.binary,['--model',args.model,'--image',selection[0][1]],out/'baseline-profile.json')
    audit(baseline,first_saved,pins,False,False,historical)
    require(sum(v['profile']['forward_wall_ns']+v['reference_forward_wall_ns'] for v in baseline['views'])<=wall*1e9,'external timer covers baseline forwards')
    directory=fresh(out/'reports');pairs=[];forwards=baseline['execution']['model_forwards']
    first_report=None
    for name,image,saved,data in selection:
        print('Profiling and exact comparison '+data+' '+name,flush=True)
        results=[];timings=[]
        for repeat,reference_first in [('first',False),('repeat',True)]:
            flags=['--repeat2x']+(['--reference-first'] if reference_first else [])
            report,wall=latency.native(args.binary,[*flags,'--model',args.model,'--image',image],directory/(name+'-'+repeat+'.json'))
            audit(report,load(saved),pins,True,reference_first,historical)
            timed=sum(v['profile']['forward_wall_ns']+v['reference_forward_wall_ns'] for v in report['views'])
            require(timed<=wall*1e9,'external process timer bounds all timed forwards')
            results.append(numerical_projection(report));forwards+=report['execution']['model_forwards']
            timings.append(dict(order=report['execution']['order'],optimized_ns=sum(v['profile']['forward_wall_ns'] for v in report['views']),reference_ns=sum(v['reference_forward_wall_ns'] for v in report['views']),external_wall_s=wall))
            if first_report is None:first_report=report
        require(results[0]==results[1],'exact repeated full numerical output')
        pairs.append(dict(name=name,data=data,measurements=timings))
    changes=[('aggregation',lambda j:j['views'][0]['profile'].__setitem__('operator_sum_ns',0)),
        ('clock',lambda j:j['views'][0]['profile']['operators'][0].__setitem__('finished_ns',10**30)),
        ('raw-output',lambda j:j['views'][0].__setitem__('reference_output_tensor_f32le_sha256','0'*64)),
        ('sources',lambda j:j.__setitem__('compiled_sources',{})),
        ('work',lambda j:j['execution'].__setitem__('model_forwards',4)),
        ('grouping',lambda j:j['views'][0]['profile'].__setitem__('by_operation',[])),
        ('detections',lambda j:j['views'][0].__setitem__('detections',[])),
        ('current-confidence-one-ulp',lambda j:j['views'][0]['detections'][0].__setitem__('confidence',struct.unpack('<f',struct.pack('<I',measurement_bits(j['views'][0]['detections'])[0]['confidence']+1))[0]))]
    rejected=[]
    for name,change in changes:
        bad=copy.deepcopy(first_report);change(bad)
        try:audit(bad,first_saved,pins,True,False,latency.camera_history.Comparison())
        except ValueError as error:rejected.append(dict(name=name,error=str(error)))
        else:raise ValueError('corrupt profile accepted: '+name)
    write(out/'report-rejections.json',rejected)
    manifest=out/'legacy-profile-input.json'
    report_path=directory/'tum-0-first.json'
    write(manifest,dict(schema_version=1,camera_id='operator-diagnostic',width=640,height=480,events=[dict(now_s=0.,input=dict(kind='analysis',acquired_s=0.,report='reports/tum-0-first.json',sha256=digest(report_path)))]))
    legacy=latency.native(args.guard,['--sequence',manifest],out/'legacy-profile-unexpected.json',2,'invalid pinned camera analysis report')
    require(all(digest(ROOT/name)==pin for name,pin in all_pins.items()) and digest(args.binary)==receipt['executable_sha256'],'source/executable unchanged during validation')
    optimized=[m['optimized_ns'] for row in pairs for m in row['measurements']]
    reference=[m['reference_ns'] for row in pairs for m in row['measurements']]
    quality=load(ROOT/'assets/camera-analysis-v1/publication-repair/regression/results.json')
    write(out/'summary.json',dict(pass_checks=True,original_images=16,repeats_per_image=2,full_raw_tensor_bitwise_parity=True,
        original_per_view_and_final_detections_identical=True,paired_order_alternated=True,pairs=pairs,
        current_host_reference_exact=True,historical_comparison=historical.report(),
        mean_optimized_four_view_inference_s=statistics.mean(optimized)/1e9,mean_original_four_view_inference_s=statistics.mean(reference)/1e9,
        aggregate_inference_speedup=sum(reference)/sum(optimized),aggregate_inference_reduction_fraction=1-sum(optimized)/sum(reference),
        faster_images=sum(sum(m['optimized_ns'] for m in r['measurements'])<sum(m['reference_ns'] for m in r['measurements']) for r in pairs),
        hardware_speedup_required_for_success=False,deadline_measurement=False,existing_parallel_pipeline_changed=False,
        validation_actual_model_forwards=forwards,baseline_actual_model_forwards=8,repeat2x_actual_model_forwards=forwards-8,
        rejected_report_controls=rejected,legacy_profile_rejection=legacy,inherited_bdd_quality=quality['overall'],
        inherited_quality_gates_pass=quality['quality_gates']['pass'],new_accuracy_improvement_measured=False,
        executable_sha256=digest(args.binary),checker_sha256=digest(__file__)))
    print(json.dumps(dict(pass_checks=True,original_images=16,aggregate_inference_speedup=sum(reference)/sum(optimized),actual_model_forwards=forwards)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-profile')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--bdd',type=Path,default=ROOT/'artifacts/bdd-camera')
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    for key in ('binary','guard','model','bdd','output'):setattr(args,key,getattr(args,key).absolute())
    if args.mode=='controls':controls(args)
    else:
        require(args.controls is not None,'--controls required')
        run(args)
