#!/usr/bin/env python3
"""Audit every saved raw pair, controller transition and source without inference."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def sha(data):return hashlib.sha256(data).hexdigest()


def raw(name):
    p=HERE/name
    return p.read_bytes() if p.exists() else gzip.decompress(Path(str(p)+'.gz').read_bytes())


def read(name):return json.loads(raw(name))


def main():
    files=read('files.json')
    assert set(files)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p!=HERE/'files.json'}
    for name,item in files.items():
        data=(HERE/name).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    for name,item in read('raw-files.json').items():
        data=raw(name);assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    before=read('before.json');previous=json.loads((ROOT/'assets/camera-supervised-v1/source-sha256.json').read_text())
    assert before['starting_commit']=='50dbaf4f04b4e46c16a2a7b6ef33e70fc8155c91' and before['previous_sources']==previous
    assert len(previous)==137 and len(before['prior_indexes'])==76
    count=0
    for name,pin in before['prior_indexes'].items():
        p=ROOT/name;assert sha(p.read_bytes())==pin,name
        for member,item in json.loads(p.read_text()).items():
            data=(p.parent/member).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],member
            count+=1
    assert count==6033
    folders=[(ROOT/'assets'/name,json.loads((ROOT/'assets'/name/'source-sha256.json').read_text())) for name in
             ['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1','camera-supervised-v1']]
    pins=read('source-sha256.json');assert len(pins)==149
    for name,pin in pins.items():
        assert sha((ROOT/name).read_bytes())==pin,name
        folder=next((p for p,mapping in folders if name in mapping),HERE)
        assert sha((folder/'source'/name).read_bytes())==pin,name
    assert read('immutability.json')==dict(previous_source_files=137,prior_indexes=76,prior_evidence_entries=6033,core_outputs=245,all_unchanged=True)
    assert read('core-output-sha256.json')==before['core_outputs']==json.loads((ROOT/'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert not any(p.suffix in ('.onnx','.jpg','.exe') for p in HERE.rglob('*') if p.is_file())
    assert {str(p.relative_to(HERE)) for p in HERE.rglob('*.png')}=={'controls/inputs/0.png','controls/inputs/1.png','controls/inputs/invalid.png'}
    for i in range(2):assert (HERE/f'controls/inputs/{i}.png').read_bytes()==(ROOT/f'assets/camera-sequence-v1/controls/inputs/{i}.png').read_bytes()
    graph=[json.loads(row) for row in raw('bound-graph.jsonl').splitlines()]
    assert len(graph)==30 and raw('graph-inspection.jsonl')==raw('bound-graph.jsonl')
    assert len({g['name'] for g in graph})==30 and all(g['original']['op']=='Conv' for g in graph)

    sys.path.insert(0,str(ROOT/'integrations/onnx'))
    import camera_depthwise_check as c
    old=(c.HERE/'src/camera_nano_repeat_analysis.rs').read_text()
    assert (c.HERE/'src/camera_depthwise_analysis.rs').read_text()==old.replace('nano_repeat2x','nano_depthwise').replace('validated_asymmetric_floor_nearest_2x_only','validated_resize2x_and_rowwise_depthwise3x3')
    original=(c.HERE/'src/camera_nano_profile.rs').read_text();current=(c.HERE/'src/camera_depthwise_profile.rs').read_text()
    assert original[original.index('fn preprocess('):original.index('fn values(')]==current[current.index('fn preprocess('):current.index('fn values(')]
    original=(c.HERE/'src/camera_nano_repeat_detector.rs').read_text();current=(c.HERE/'src/camera_depthwise_detector.rs').read_text()
    assert original[original.index('    pub fn detect('):original.index('        let outputs =')]==current[current.index('    pub fn detect('):current.index('        let outputs =')]
    pilot_before=read('pilot-before.json')
    for name,pin in pilot_before['source_sha256'].items():
        if name!='integrations/onnx/camera_depthwise_check.py':assert pins[name]==pin,name
    assert sha(raw('pilot-checker.py'))==pilot_before['source_sha256']['integrations/onnx/camera_depthwise_check.py']
    assert (c.HERE/'camera_depthwise_check.py').read_bytes().startswith(raw('pilot-checker.py'))
    assert pilot_before['planned_model_forwards']==16 and pilot_before['diagnostic_only']
    pilot=read('pilot.json');serial=read('pilot-serial.json');image=read('pilot-image.json')
    saved=json.loads((ROOT/'assets/camera-sequence-v1/recorded/exports/first-0.json').read_text())
    c.nano.check_image(serial,saved,parallel=False);c.audit_profile(pilot,serial,c.expected(),True,False);c.check_image(image,saved,serial)
    a=sum(v['profile']['forward_wall_ns'] for v in pilot['views']);b=sum(v['reference_forward_wall_ns'] for v in pilot['views'])
    assert read('pilot-summary.json')==dict(pass_checks=True,actual_model_forwards=16,output_bits_exact=True,serial_optimized_ns=a,serial_resize_only_ns=b,reduction_fraction=1-a/b)
    protocol=read('validation/before-inference.json')
    for name,pin in protocol['source_sha256'].items():assert pins[name]==pin,name
    assert protocol['schedulers']==['fifo','supervised'] and protocol['order']==c.ORDER and protocol['policy']==c.freshness.POLICY
    assert protocol['prior_diagnostic_forwards']==16 and protocol['additional_inference_warmup'] is False
    assert protocol['all_timed_trials_before_main_profiles_and_references'] and not protocol['timing_success_required_for_execution']
    assert protocol['baselines']==dict(fifo='existing Nano Resize2x',supervised='existing unoptimized Nano supervised')
    assert sha(raw('validation/initial-checker.py'))==pins['integrations/onnx/camera_depthwise_check.py']==c.digest(c.__file__)
    source=json.loads((ROOT/'assets/camera-sequence-v1/sequence.json').read_text())
    assert sha((ROOT/'assets/camera-sequence-v1/sequence.json').read_bytes())==protocol['source_sequence_sha256']==c.nano.previous.SOURCE_SHA
    refs={};pairs=[];profile_count=0;image_count=0
    for row in protocol['image_selection']:
        name=row['name'];saved_path=ROOT/row['saved_report'];assert sha(saved_path.read_bytes())==row['saved_report_sha256']
        original=json.loads(saved_path.read_text());assert row['image_sha256']==original['source_image_sha256']
        reference=read('validation/references/'+name+'.json');c.nano.check_image(reference,original,parallel=False);refs[name]=reference
        p=read('validation/profiles/'+name+'.json');c.audit_profile(p,reference,c.expected(),True,row['reference_first']);profile_count+=p['execution']['model_forwards']
        r=read('validation/images/'+name+'.json');c.check_image(r,original,reference);image_count+=r['execution']['model_forwards']
        for v in p['views']:
            assert {o['node_name'] for o in v['profile']['operators'] if o['operation']=='SafeRowDepthWise3x3'}=={g['name'] for g in graph}
        a=sum(v['profile']['forward_wall_ns'] for v in p['views']);b=sum(v['reference_forward_wall_ns'] for v in p['views'])
        pairs.append(dict(name=name,optimized_observed_ns=a,resize_only_reference_ns=b,reduction_fraction=1-a/b,reference_first=row['reference_first']))
    assert len(refs)==16 and profile_count==128 and image_count==64
    trials=read('validation/trials.json');assert [(t['scheduler'],t['label']) for t in trials]==[(s,l) for s in ('fifo','supervised') for l in c.ORDER]
    by_sha={source['events'][i]['input']['sha256']:refs[f'tum-{i}'] for i in range(8)};comparison=[];timed=0
    for t in trials:
        scheduler,label=t['scheduler'],t['label'];optimized=label.startswith('optimized');mode='optimized' if optimized else scheduler
        r=read(f'validation/{scheduler}/{label}.json');inspection=read(f'validation/{mode}-inspection.json');receipt=read(f'validation/{mode}-executable.json')
        assert receipt==protocol['executable_receipts'][mode] and r['compiled_sources']==receipt['compiled_sources']
        assert r['source_sequence_sha256']==inspection['source_sequence_sha256']==protocol['source_sequence_sha256']
        assert t==read(f'validation/{scheduler}/{label}-trial.json') and t['model_forwards']==r['model_forwards'] and t['summary']==r['summary']
        assert r['startup_before_schedule_wall_s']+r['schedule_wall_s']<=t['external_wall_s']+.01
        if scheduler=='supervised':
            manifest=(c.audit_supervised if optimized else c.supervised.audit)(r,source,inspection,by_sha)
            outputs=[x['change']['output'] for x in r['trace'] if x['change']['kind']=='guard']
            available=r['summary']['available_result_images'];inferred=r['summary']['completed_images']
            times=[x['change']['worker_finished_s']-x['change']['worker_started_s'] for x in r['trace'] if x['change']['kind']=='finished']
            assert r['summary']['max_observed_poll_gap_s']>.01
        else:
            manifest,_,_=(c.audit_fifo if optimized else c.repeat.clock.audit_fifo)(r,source,inspection,c.freshness,c.tracking)
            for row in r['events']:
                report=row['measured']['analysis'];i=row['source']['event_index']
                if report is not None:(c.check_image if optimized else c.repeat.check_image)(report,json.loads((ROOT/protocol['image_selection'][i]['saved_report']).read_text()),refs[f'tum-{i}'])
            outputs=[e['output'] for row in r['events'] for e in row['measured']['guard_events']]
            available=r['summary']['available_at_check_images'];inferred=r['summary']['inferred_images']
            times=[row['measured']['timing']['read_analysis_and_probe_wall_s'] for row in r['events'] if row['measured']['analysis'] is not None]
        assert manifest==read(f'validation/{scheduler}/{label}-guard-input.json')
        assert outputs==read(f'validation/{scheduler}/{label}-guard-result.json')['results'];timed+=r['model_forwards']
        comparison.append(dict(scheduler=scheduler,label=label,source_images=8,model_forwards=r['model_forwards'],inferred_images=inferred,available_results=available,
            inferred_but_late_images=r['summary']['inferred_but_late_images'],processing_s=times))
    summary=read('validation/summary.json')
    assert summary['pass_checks'] and summary['comparison']==comparison and summary['serial_profile_pairs']==pairs
    assert summary['serial_profile_aggregate_reduction_fraction']==1-sum(p['optimized_observed_ns'] for p in pairs)/sum(p['resize_only_reference_ns'] for p in pairs)
    assert summary['serial_profile_faster_pairs']==sum(p['reduction_fraction']>0 for p in pairs)==15
    assert [t['available_results'] for t in comparison]==[1,2,2,1,3,4,4,1]
    assert timed==summary['timed_model_forwards']==136 and summary['actual_main_model_forwards']==392 and summary['actual_all_model_forwards']==408
    assert summary['raw_output_pairs']==64 and summary['raw_output_f32_values_compared']==19306560 and summary['independent_guard_replays']==8
    assert summary['full_current_measurement_bits_exact'] and summary['checker_sha256']==c.digest(c.__file__)
    for key in ('new_holdout','new_accuracy_scoring','driving_integration','hard_real_time_guarantee'):assert summary[key] is False
    control=read('controls/depthwise-controls.json');assert control['pass_checks'] and control['valid_model_forwards']==0
    assert control['shared_rejected_inputs']==21 and len(control['additional_rejections'])==6 and control['output_protections']==2 and control['model_free_guard_replays']==1
    assert control['checker_sha256']==c.digest(c.__file__) and control['executable_sha256']==protocol['executable_receipts']['optimized']['executable_sha256']
    empty=read('controls/no-images-supervised.json')
    assert c.audit_supervised(empty,read('controls/no-images.json'),read('controls/no-images-supervised-inspection.json'))==read('controls/no-images-supervised-guard-input.json')
    assert [x['change']['output'] for x in empty['trace'] if x['change']['kind']=='guard']==read('controls/no-images-supervised-guard-result.json')['results']
    check=read('checks/results.json');log=raw('checks/core.log').decode()
    assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',log)))==check['root_tests']==371
    assert log.count('"verified": true')==check['scenario_replays']==49
    assert b'64 passed; 0 failed' in raw('checks/tests.log') and b'Ran 12 tests' in raw('checks/python-tests.log')
    assert not check['fresh_full_orchestrator_execution'] and check['saved_pilot_orchestrator_audit']
    orchestrator_audit=json.loads(raw('checks/reproduce-audit.log'));assert orchestrator_audit['model_execution_this_check'] is False
    assert {k:v for k,v in orchestrator_audit.items() if k!='model_execution_this_check'}==read('pilot-summary.json')
    print(json.dumps(dict(pass_checks=True,packet_files=len(files),prior_evidence_entries=count,actual_model_forwards_audited=408,
        main_raw_f32_values_compared=19306560,exact_guard_replays=8,model_free_guard_replays=1,model_execution_this_check=False)))


if __name__=='__main__':main()
