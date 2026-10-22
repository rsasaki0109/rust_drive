#!/usr/bin/env python3
"""Check lowered-plan parallel execution against the existing parallel baseline."""
import argparse
import importlib.util
import json
from pathlib import Path
import statistics

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
spec=importlib.util.spec_from_file_location('repeat_parallel_support',HERE/'camera_parallel_check.py')
previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
require,digest,load,write,fresh=previous.require,previous.digest,previous.load,previous.write,previous.fresh
require(digest(HERE/'camera_parallel_check.py')=='f9d4e081b4989f3ffe3bd40f7f030f817797188161370b636276c127ac90d109','pinned parallel auditor')
NEW=['integrations/onnx/src/camera_repeat2.rs','integrations/onnx/src/camera_repeat_detector.rs',
     'integrations/onnx/src/camera_repeat_analysis.rs','integrations/onnx/src/bin/rustdriving-camera-repeat2x.rs','integrations/onnx/profile-repeat2x.json']
ORDER=['baseline-first','repeat2x-first','repeat2x-repeat','baseline-repeat']


def expected(base,exclude=()):
    return dict(base,**{p:digest(ROOT/p) for p in NEW if p not in exclude})


def check_analysis(actual,serial,historical=None):
    image_sources=expected(load(ROOT/'assets/camera-parallel-v1/verification/parity/tum-0-first.json')['compiled_sources'])
    require(actual['schema_version']==1 and actual['algorithm']=='yolox_s_repeat2x_parallel_image_analysis_v1','distinct optimized image identity')
    require(actual['compiled_sources']==image_sources and actual['source_image_sha256']==serial['source_image_sha256'],'complete optimized producer sources and original image')
    require(actual['graph_optimization']=='validated_asymmetric_floor_nearest_2x_only','explicit graph lowering')
    base=actual['base_prediction'];old=serial['base_prediction']
    require(base['algorithm']=='pinned_yolox_s_repeat2x_parallel_fixed_views_v1' and base['compiled_sources']==image_sources,'distinct lowered base identity')
    project=lambda obj:{k:v for k,v in obj.items() if k not in ('algorithm','compiled_sources','views')}
    def compare(expected,observed,label):
        if historical is None:require(observed==expected,label)
        else:historical.same(expected,observed,label)
    compare(project(old),project(base),'complete original geometry, thresholds and merged proposals')
    require(len(base['views'])==len(old['views']),'all original views')
    for row,reference in zip(base['views'],old['views']):
        require(row['rectangle']==reference['rectangle'] and row['rgb_sha256']==reference['rgb_sha256'],'original view pixels')
        p=row['prediction'];q=reference['prediction']
        require(p['algorithm']=='pinned_yolox_s_repeat2x_cpu_image_triangle_argmax_classwise' and p['model_profile']=='yolox-s-repeat2x-v1','distinct lowered detector identity')
        require(p['compiled_sources']==expected(q['compiled_sources'],['integrations/onnx/src/camera_repeat_analysis.rs'])
                and p['model_profile_sha256']==digest(HERE/'profile-repeat2x.json'),'complete detector/recipe receipt')
        excluded={'algorithm','model_profile','compiled_sources','model_profile_sha256','model_load_seconds','inference_seconds'}
        compare({k:v for k,v in q.items() if k not in excluded},{k:v for k,v in p.items() if k not in excluded},'all original per-view measurements exactly preserved')
    compare(serial['result'],actual['result'],'all final consistency hypotheses and work counts unchanged')
    execution=actual['execution'];n=len(base['views'])
    require({k:v for k,v in execution.items() if k!='analysis_wall_s'}==dict(model_read=True,images_read=1,model_forwards=n,inference_run=True,view_workers=n,shared_model_plans=1),'single lowered plan and original concurrent views')
    require(execution['analysis_wall_s']>=max(v['prediction']['inference_seconds'] for v in base['views']),'complete wrapper wall time covers every forward')


def controls(args):
    previous.controls(args)
    row=load(args.output/'parallel-summary.json')
    write(args.output/'repeat-summary.json',dict(pass_checks=row['pass_checks'],executable_sha256=digest(args.binary),checker_sha256=digest(__file__),fatal_inputs=25,existing_output_protections=2))


def run(args):
    out=fresh(args.output);latency,freshness,tracking=previous.support()
    historical=latency.camera_history.Comparison()
    controls=load(args.controls/'repeat-summary.json')
    require(controls['pass_checks'] and controls['checker_sha256']==digest(__file__) and controls['executable_sha256']==digest(args.binary),'matching lowered-plan controls')
    source_path=ROOT/'assets/camera-sequence-v1/sequence.json';source=load(source_path)
    require(digest(source_path)==previous.SOURCE_SHA and digest(args.model)==previous.MODEL_SHA,'original source schedule/weights')
    binaries=dict(baseline=args.baseline,repeat2x=args.binary);receipts={}
    original=load(ROOT/'assets/camera-parallel-v1/verification/executable.json')['compiled_sources']
    for mode,binary in binaries.items():
        receipts[mode],_=latency.native(binary,['--executable-receipt'],out/(mode+'-executable.json'))
        require(receipts[mode]['compiled_sources']==(original if mode=='baseline' else expected(original)),'complete unchanged baseline or lowered executable sources')
        require(receipts[mode]['executable_sha256']==digest(binary),'actual executable binding')
    selection=[]
    for i,event in enumerate(source['events']):
        selection.append((f'tum-{i}',source_path.parent/event['input']['path'],ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json'))
    for saved in sorted((ROOT/'assets/camera-analysis-v1/publication-repair/regression/first').glob('*.json')):
        selection.append((saved.stem,args.bdd/(saved.stem+'.jpg'),saved))
    require(len(selection)==16,'complete original image inventory')
    for _,image,saved in selection:require(digest(image)==load(saved)['source_image_sha256'],'original image bytes')
    pins=expected(original)
    for path in (Path(__file__),HERE/'camera_history.py',HERE/'camera_parallel_check.py',HERE/'camera_latency_check.py'):
        pins[str(path.relative_to(ROOT))]=digest(path)
    write(out/'before-inference.json',dict(order=ORDER,policy=freshness.POLICY,source_sequence_sha256=previous.SOURCE_SHA,model_sha256=previous.MODEL_SHA,
        executable_receipts=receipts,source_sha256=pins,image_selection=[dict(name=name,image_sha256=digest(image),reference_report_sha256=digest(saved)) for name,image,saved in selection],
        original_thresholds_views_and_controller=True,new_holdout=False))
    inspections={}
    for mode,binary in binaries.items():inspections[mode],_=latency.native(binary,['--inspect-sequence',source_path],out/(mode+'-inspection.json'))
    trials=[];forwards=0
    for label in ORDER:
        mode=label.split('-')[0];print('Measured '+label,flush=True)
        report,wall=latency.native(binaries[mode],['--model',args.model,'--sequence',source_path],out/(label+'.json'))
        require(report['compiled_sources']==receipts[mode]['compiled_sources'] and report['source_sequence_sha256']==previous.SOURCE_SHA,'timed complete source receipt')
        require(report['startup_before_schedule_wall_s']+report['schedule_wall_s']<=wall+.01,'external time bounds reported work')
        manifest,_,_=previous.audit_parallel(report,source,inspections[mode],freshness,tracking)
        for row in report['events']:
            actual=row['measured']['analysis']
            if actual is not None:
                saved=load(ROOT/f"assets/camera-sequence-v1/recorded/exports/first-{row['source']['event_index']}.json")
                if mode=='baseline':previous.parallel_analysis(actual,saved,historical)
                else:check_analysis(actual,saved,historical)
        path=out/(label+'-guard-input.json');write(path,manifest)
        guard,_=latency.native(args.guard,['--sequence',path],out/(label+'-guard-result.json'))
        require(guard['results']==[item['output'] for row in report['events'] for item in row['measured']['guard_events']],'exact independent guard replay')
        trials.append(dict(label=label,external_wall_s=wall,processing_s=[r['measured']['timing']['read_analysis_and_probe_wall_s'] for r in report['events'] if r['measured']['analysis'] is not None],summary=report['summary'],model_forwards=report['model_forwards']))
        forwards+=report['model_forwards']
    write(out/'trials.json',trials)
    reports=fresh(out/'reports');references=fresh(out/'current-reference');parity_forwards=0;reference_forwards=0
    current={}
    for name,image,saved in selection:
        print('Comparing lowered parallel '+name,flush=True);results=[]
        reference,_=latency.native(args.baseline,['--model',args.model,'--image',image],references/(name+'.json'))
        previous.parallel_analysis(reference,load(saved),historical)
        current[name]=reference;reference_forwards+=reference['execution']['model_forwards']
        for repeat in ('first','repeat'):
            actual,_=latency.native(args.binary,['--model',args.model,'--image',image],reports/(name+'-'+repeat+'.json'))
            check_analysis(actual,reference);results.append(previous.stripped(actual));parity_forwards+=actual['execution']['model_forwards']
        require(results[0]==results[1],'exact repeat except measured durations')
    # Keep reference work after all timed trials; every inferred timed frame is
    # still checked against the same current-host baseline without a tolerance.
    for label in ORDER:
        for row in load(out/(label+'.json'))['events']:
            actual=row['measured']['analysis']
            if actual is not None:
                reference=current['tum-'+str(row['source']['event_index'])]
                if label.startswith('baseline'):previous.parallel_analysis(actual,reference)
                else:check_analysis(actual,reference)
    manifest=out/'legacy-profile-input.json';report_path=reports/'tum-0-first.json'
    write(manifest,dict(schema_version=1,camera_id='repeat2x-profile',width=640,height=480,events=[dict(now_s=0.,input=dict(kind='analysis',acquired_s=0.,report='reports/tum-0-first.json',sha256=digest(report_path)))]))
    legacy=latency.native(args.guard,['--sequence',manifest],out/'legacy-profile-unexpected.json',2,'invalid pinned camera analysis report')
    require(all(digest(ROOT/name)==pin for name,pin in pins.items()),'source unchanged during validation')
    require(all(digest(binaries[m])==r['executable_sha256'] for m,r in receipts.items()),'executable unchanged')
    baseline=[r['processing_s'][0] for r in trials if r['label'].startswith('baseline')]
    optimized=[r['processing_s'][0] for r in trials if r['label'].startswith('repeat2x')]
    quality=load(ROOT/'assets/camera-analysis-v1/publication-repair/regression/results.json')
    write(out/'summary.json',dict(pass_checks=True,order=ORDER,baseline_processing_s=baseline,repeat2x_processing_s=optimized,
        mean_speedup=statistics.mean(baseline)/statistics.mean(optimized),mean_reduction_fraction=1-statistics.mean(optimized)/statistics.mean(baseline),
        hardware_speedup_required_for_success=False,age_limit_s=.2,deadline_gates={r['label']:r['summary']['deadline_gate_pass'] for r in trials},
        actual_measured_parallel_replay=True,original_images=16,repeats_per_image=2,complete_original_measurement_and_result_parity=True,
        current_host_reference_exact=True,historical_comparison=historical.report(),
        timed_actual_model_forwards=forwards,parity_actual_model_forwards=parity_forwards,current_reference_model_forwards=reference_forwards,total_actual_model_forwards=forwards+parity_forwards+reference_forwards,
        inherited_bdd_quality=quality['overall'],inherited_quality_gates_pass=quality['quality_gates']['pass'],new_accuracy_improvement_measured=False,
        legacy_profile_rejection=legacy,checker_sha256=digest(__file__),executable_sha256=digest(args.binary)))
    print(json.dumps(dict(pass_checks=True,mean_speedup=statistics.mean(baseline)/statistics.mean(optimized),actual_model_forwards=forwards+parity_forwards+reference_forwards)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-repeat2x')
    parser.add_argument('--baseline',type=Path,default=HERE/'target/release/rustdriving-camera-parallel')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--bdd',type=Path,default=ROOT/'artifacts/bdd-camera');parser.add_argument('--controls',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    for key in ('binary','baseline','guard','model','bdd','output'):setattr(args,key,getattr(args,key).absolute())
    if args.mode=='controls':controls(args)
    else:require(args.controls is not None,'--controls required');run(args)
