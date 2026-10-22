#!/usr/bin/env python3
"""Reconstruct both Debian experiments, every raw pair and every guard output."""
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
    index=read('files.json')
    assert set(index)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p!=HERE/'files.json'}
    for name,item in index.items():
        data=(HERE/name).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    for name,item in read('raw-files.json').items():
        data=raw(name);assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    before=read('before.json');old=json.loads((ROOT/'assets/camera-depthwise-v1/source-sha256.json').read_text())
    assert before['starting_commit']=='2a6c66f5a16b4e25cf05e50e00d7bb3cc9392938' and before['previous_sources']==old and len(old)==149
    assert len(before['prior_indexes'])==77
    count=0
    for name,pin in before['prior_indexes'].items():
        p=ROOT/name;assert sha(p.read_bytes())==pin,name
        for member,item in json.loads(p.read_text()).items():
            data=(p.parent/member).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],member
            count+=1
    assert count==6204
    folders=[(ROOT/'assets'/name,json.loads((ROOT/'assets'/name/'source-sha256.json').read_text())) for name in
             ['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1','camera-supervised-v1','camera-depthwise-v1']]
    pins=read('source-sha256.json');assert len(pins)==157
    for name,pin in pins.items():
        assert sha((ROOT/name).read_bytes())==pin,name
        folder=next((p for p,mapping in folders if name in mapping),HERE)
        assert sha((folder/'source'/name).read_bytes())==pin,name
    assert read('immutability.json')==dict(previous_source_files=149,prior_indexes=77,prior_evidence_entries=6204,core_outputs=245,all_unchanged=True)
    assert read('core-output-sha256.json')==before['core_outputs']==json.loads((ROOT/'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert not any(p.suffix in ('.onnx','.jpg','.exe','.whl') for p in HERE.rglob('*') if p.is_file())
    assert all('/inputs/' in str(p.relative_to(HERE)) for p in HERE.rglob('*.png'))
    for p in HERE.rglob('*.png'):
        if p.name in ('0.png','1.png'):assert p.read_bytes()==(ROOT/'assets/camera-sequence-v1/controls/inputs'/p.name).read_bytes()
        else:assert p.name=='invalid.png' and p.read_bytes()==b'not an image\n'
    receipt=read('container-receipt.json');env=read('container-environment.json')
    assert receipt['image']=='python@sha256:392307d22300de8b5986851a12d9176dfc0fc073e65bf6523ebd7dcbeb23564e'
    assert receipt['network']==env['network']=='none' and receipt['same_physical_host'] and not receipt['independent_hardware']
    assert env['python']=='3.12.14' and env['libc']==['glibc','2.36'] and env['numpy']=='2.3.5' and env['pillow']=='12.3.0'
    assert env['cpu_quota']=='400000 100000' and 'Debian GNU/Linux 12' in env['os_release']
    assert len(read('wheel-sha256.json'))==2

    sys.path.insert(0,str(ROOT/'integrations/onnx'))
    import camera_rowzip_check as c
    c.check_sources();source=json.loads((ROOT/'assets/camera-sequence-v1/sequence.json').read_text())
    source_sha=sha((ROOT/'assets/camera-sequence-v1/sequence.json').read_bytes());assert source_sha==c.nano.previous.SOURCE_SHA
    # First actually executed full reproduction of the previously frozen wrapper.
    prefix='container-baseline/'
    protocol=read(prefix+'validation/before-inference.json')
    for name,pin in protocol['source_sha256'].items():assert pins[name]==pin,name
    assert sha(raw(prefix+'validation/initial-checker.py'))==c.digest(c.previous.__file__)
    pilot=read(prefix+'pilot/pilot.json');serial=read(prefix+'pilot/pilot-serial.json');image=read(prefix+'pilot/pilot-image.json')
    saved=json.loads((ROOT/'assets/camera-sequence-v1/recorded/exports/first-0.json').read_text())
    c.nano.check_image(serial,saved,parallel=False);c.previous.audit_profile(pilot,serial,c.previous.expected(),True,False);c.previous.check_image(image,saved,serial)
    pilot_before=read(prefix+'pilot/pilot-before.json')
    assert pilot_before['planned_model_forwards']==16 and pilot_before['diagnostic_only']
    assert pilot_before['checker_sha256']==c.digest(c.previous.__file__) and pilot_before['orchestrator_sha256']==c.digest(c.HERE/'camera_depthwise_reproduce.py')
    refs={};profile_count=0;parallel_count=0;pairs=[]
    for row in protocol['image_selection']:
        name=row['name'];p=ROOT/row['saved_report'];assert sha(p.read_bytes())==row['saved_report_sha256']
        original=json.loads(p.read_text());assert original['source_image_sha256']==row['image_sha256']
        r=read(prefix+'validation/references/'+name+'.json');c.nano.check_image(r,original,parallel=False);refs[name]=r
        p=read(prefix+'validation/profiles/'+name+'.json');c.previous.audit_profile(p,r,c.previous.expected(),True,row['reference_first']);profile_count+=p['execution']['model_forwards']
        a=read(prefix+'validation/images/'+name+'.json');c.previous.check_image(a,original,r);parallel_count+=a['execution']['model_forwards']
        t=sum(v['profile']['forward_wall_ns'] for v in p['views']);b=sum(v['reference_forward_wall_ns'] for v in p['views'])
        pairs.append(dict(name=name,optimized_observed_ns=t,resize_only_reference_ns=b,reduction_fraction=1-t/b,reference_first=row['reference_first']))
    assert len(refs)==16 and profile_count==128 and parallel_count==64
    runtime_timed=0;comparisons=[];by_sha={source['events'][i]['input']['sha256']:refs[f'tum-{i}'] for i in range(8)}
    trials=read(prefix+'validation/trials.json');assert [(t['scheduler'],t['label']) for t in trials]==[(s,l) for s in ('fifo','supervised') for l in c.ORDER]
    for t in trials:
        scheduler,label=t['scheduler'],t['label'];optimized=label.startswith('optimized');mode='optimized' if optimized else scheduler
        r=read(f'{prefix}validation/{scheduler}/{label}.json');inspection=read(f'{prefix}validation/{mode}-inspection.json');binary=read(f'{prefix}validation/{mode}-executable.json')
        assert binary==protocol['executable_receipts'][mode] and r['compiled_sources']==binary['compiled_sources']
        assert r['source_sequence_sha256']==inspection['source_sequence_sha256']==source_sha
        assert t==read(f'{prefix}validation/{scheduler}/{label}-trial.json') and t['summary']==r['summary'] and t['model_forwards']==r['model_forwards']
        if scheduler=='supervised':
            manifest=(c.previous.audit_supervised if optimized else c.supervised.audit)(r,source,inspection,by_sha)
            outputs=[x['change']['output'] for x in r['trace'] if x['change']['kind']=='guard']
            available=r['summary']['available_result_images'];inferred=r['summary']['completed_images']
            times=[x['change']['worker_finished_s']-x['change']['worker_started_s'] for x in r['trace'] if x['change']['kind']=='finished']
        else:
            manifest,_,_=(c.previous.audit_fifo if optimized else c.repeat.clock.audit_fifo)(r,source,inspection,c.freshness,c.tracking)
            for x in r['events']:
                a=x['measured']['analysis'];i=x['source']['event_index']
                if a is not None:(c.previous.check_image if optimized else c.repeat.check_image)(a,json.loads((ROOT/protocol['image_selection'][i]['saved_report']).read_text()),refs[f'tum-{i}'])
            outputs=[e['output'] for x in r['events'] for e in x['measured']['guard_events']]
            available=r['summary']['available_at_check_images'];inferred=r['summary']['inferred_images']
            times=[x['measured']['timing']['read_analysis_and_probe_wall_s'] for x in r['events'] if x['measured']['analysis'] is not None]
        assert manifest==read(f'{prefix}validation/{scheduler}/{label}-guard-input.json') and outputs==read(f'{prefix}validation/{scheduler}/{label}-guard-result.json')['results']
        runtime_timed+=r['model_forwards']
        comparisons.append(dict(scheduler=scheduler,label=label,source_images=8,model_forwards=r['model_forwards'],inferred_images=inferred,available_results=available,
            inferred_but_late_images=r['summary']['inferred_but_late_images'],processing_s=times))
    runtime=read(prefix+'validation/summary.json');assert runtime['pass_checks'] and runtime['comparison']==comparisons and runtime['serial_profile_pairs']==pairs
    assert [t['available_results'] for t in comparisons]==[1,1,1,1,3,3,4,1]
    assert [t['inferred_but_late_images'] for t in comparisons]==[1,1,1,0,0,0,0,0]
    assert runtime_timed==132 and runtime['actual_main_model_forwards']==388 and runtime['actual_all_model_forwards']==404
    assert runtime['serial_profile_aggregate_reduction_fraction']==1-sum(p['optimized_observed_ns'] for p in pairs)/sum(p['resize_only_reference_ns'] for p in pairs)
    assert runtime['serial_profile_faster_pairs']==16 and runtime['independent_guard_replays']==8

    # New comparison: byte-for-byte shared admission/image auditors and distinct receipts.
    protocol=read('validation/before-inference.json');assert protocol['order']==c.ORDER and protocol['schedulers']==['fifo','supervised']
    assert protocol['prior_candidate_model_forwards']==0 and not protocol['additional_inference_warmup'] and protocol['all_timed_trials_before_profiles']
    assert protocol['execution_environment']==env and protocol['policy']==c.freshness.POLICY and protocol['source_sequence_sha256']==source_sha
    for name,pin in protocol['source_sha256'].items():assert pins[name]==pin,name
    assert sha(raw('validation/initial-checker.py'))==c.digest(c.__file__)
    refs={};pairs=[]
    for row in protocol['image_selection']:
        name=row['name'];saved_path=ROOT/row['saved_report'];assert sha(saved_path.read_bytes())==row['saved_report_sha256']
        original=json.loads(saved_path.read_text());assert original['source_image_sha256']==row['image_sha256']
        reference=read('validation/references/'+name+'.json');c.nano.check_image(reference,original,parallel=False);refs[name]=reference;profiles={}
        for mode in row['profile_order']:
            p=read('validation/profiles/'+name+'-'+mode+'.json');profiles[mode]=p
            c.audit_profile(p,reference,c.expected() if mode=='optimized' else c.previous.expected(),True,row['reference_first'])
            if mode=='optimized':c.variant(p)
        for old,new in zip(profiles['baseline']['views'],profiles['optimized']['views']):
            assert old['input_tensor_f32le_sha256']==new['input_tensor_f32le_sha256'] and old['output_tensor_f32le_sha256']==new['output_tensor_f32le_sha256']
        a=read('validation/images/'+name+'.json');c.variant(a);c.check_image(a,original,reference)
        t={m:sum(v['profile']['forward_wall_ns'] for v in p['views']) for m,p in profiles.items()}
        pairs.append(dict(name=name,profile_order=row['profile_order'],reference_first=row['reference_first'],baseline_observed_ns=t['baseline'],optimized_observed_ns=t['optimized'],reduction_fraction=1-t['optimized']/t['baseline']))
    by_sha={source['events'][i]['input']['sha256']:refs[f'tum-{i}'] for i in range(8)};timed=0;comparisons=[]
    trials=read('validation/trials.json');assert [(t['scheduler'],t['label']) for t in trials]==[(s,l) for s in ('fifo','supervised') for l in c.ORDER]
    for t in trials:
        scheduler,label=t['scheduler'],t['label'];optimized=label.startswith('optimized');mode='optimized' if optimized else 'baseline'
        r=read(f'validation/{scheduler}/{label}.json');inspection=read(f'validation/{mode}-inspection.json');binary=read(f'validation/{mode}-executable.json')
        assert binary==protocol['executable_receipts'][mode] and r['compiled_sources']==binary['compiled_sources']
        assert r['source_sequence_sha256']==inspection['source_sequence_sha256']==source_sha
        assert t==read(f'validation/{scheduler}/{label}-trial.json') and t['summary']==r['summary'] and t['model_forwards']==r['model_forwards']
        assert r['startup_before_schedule_wall_s']+r['schedule_wall_s']<=t['external_wall_s']+.01
        if optimized:c.variant(r);c.variant(inspection);c.variant(binary)
        if scheduler=='supervised':
            manifest=(c.audit_supervised if optimized else c.previous.audit_supervised)(r,source,inspection,by_sha)
            outputs=[x['change']['output'] for x in r['trace'] if x['change']['kind']=='guard']
            available=r['summary']['available_result_images'];inferred=r['summary']['completed_images']
            times=[x['change']['worker_finished_s']-x['change']['worker_started_s'] for x in r['trace'] if x['change']['kind']=='finished'];gap=r['summary']['max_observed_poll_gap_s']
            assert gap>.01
        else:
            manifest,_,_=c.audit_fifo(r,source,inspection,c.freshness,c.tracking)
            for x in r['events']:
                a=x['measured']['analysis'];i=x['source']['event_index']
                if a is not None:(c.check_image if optimized else c.previous.check_image)(a,json.loads((ROOT/protocol['image_selection'][i]['saved_report']).read_text()),refs[f'tum-{i}'])
            outputs=[e['output'] for x in r['events'] for e in x['measured']['guard_events']]
            available=r['summary']['available_at_check_images'];inferred=r['summary']['inferred_images'];gap=None
            times=[x['measured']['timing']['read_analysis_and_probe_wall_s'] for x in r['events'] if x['measured']['analysis'] is not None]
        assert manifest==read(f'validation/{scheduler}/{label}-guard-input.json') and outputs==read(f'validation/{scheduler}/{label}-guard-result.json')['results'];timed+=r['model_forwards']
        comparisons.append(dict(scheduler=scheduler,label=label,source_images=8,model_forwards=r['model_forwards'],inferred_images=inferred,available_results=available,
            inferred_but_late_images=r['summary']['inferred_but_late_images'],processing_s=times,max_observed_poll_gap_s=gap))
    result=read('validation/summary.json');assert result['pass_checks'] and result['comparison']==comparisons and result['serial_profile_pairs']==pairs
    assert result['serial_profile_aggregate_reduction_fraction']==1-sum(p['optimized_observed_ns'] for p in pairs)/sum(p['baseline_observed_ns'] for p in pairs)
    assert result['serial_faster_pairs']==sum(p['reduction_fraction']>0 for p in pairs)==11
    assert [t['available_results'] for t in comparisons]==[1,1,1,1,3,1,2,1]
    assert timed==120 and result['actual_model_forwards']==timed+384==504 and result['independent_guard_replays']==8
    assert result['total_f32_bit_comparisons']==192*301665==57919680 and result['checker_sha256']==c.digest(c.__file__)
    for key in ('new_holdout','new_accuracy_scoring','driving_integration','hard_real_time_guarantee'):assert result[key] is False
    for prefix in ('container-baseline/',''):
        control=read(prefix+'controls/depthwise-controls.json');assert control['pass_checks'] and control['valid_model_forwards']==0
        assert control['shared_rejected_inputs']==21 and len(control['additional_rejections'])==6 and control['output_protections']==2 and control['model_free_guard_replays']==1
        empty=read(prefix+'controls/no-images-supervised.json');f=c.previous.audit_supervised if prefix else c.audit_supervised
        assert f(empty,read(prefix+'controls/no-images.json'),read(prefix+'controls/no-images-supervised-inspection.json'))==read(prefix+'controls/no-images-supervised-guard-input.json')
        assert [x['change']['output'] for x in empty['trace'] if x['change']['kind']=='guard']==read(prefix+'controls/no-images-supervised-guard-result.json')['results']
    decision=read('decision.json');assert decision['actual_model_forwards_this_increment']==404+504==908 and decision['adopt_candidate'] is False
    assert decision['all_first_observations_retained'] and decision['unchanged_age_s']==.2 and decision['unchanged_recovery_samples']==2
    checks=read('checks/results.json');log=raw('checks/core.log').decode()
    assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',log)))==371 and log.count('"verified": true')==49
    assert b'65 passed; 0 failed' in raw('checks/tests.log') and b'Ran 14 tests' in raw('checks/python-tests.log')
    assert checks['actual_model_forwards']==908 and checks['new_kernel_tests']==1 and checks['reused_native_tests']==64
    assert not checks['combined_container_helper_fresh_execution'] and checks['constituent_container_commands_executed']
    print(json.dumps(dict(pass_checks=True,packet_files=len(index),prior_evidence_entries=count,actual_model_forwards_audited=908,exact_measured_guard_replays=16,
        model_free_guard_replays=2,candidate_f32_bit_comparisons=57919680,adopt_candidate=False,model_execution_this_check=False)))


if __name__=='__main__':main()
