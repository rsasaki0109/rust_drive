#!/usr/bin/env python3
"""Reconstruct Nano output-bit, operator, clock and availability evidence without inference."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def sha(data):return hashlib.sha256(data).hexdigest()


def raw(name):
    path=HERE/name
    return path.read_bytes() if path.exists() else gzip.decompress(Path(str(path)+'.gz').read_bytes())


def read(name):return json.loads(raw(name))


def main():
    files=read('files.json')
    assert set(files)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p!=HERE/'files.json'}
    for name,item in files.items():
        data=(HERE/name).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    for name,item in read('raw-files.json').items():
        data=raw(name);assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    old_nano=json.loads((ROOT/'assets/camera-nano-v1/source-sha256.json').read_text())
    old_latest=json.loads((ROOT/'assets/camera-latest-v1/source-sha256.json').read_text())
    pins=read('source-sha256.json')
    for name,pin in pins.items():
        assert sha((ROOT/name).read_bytes())==pin,name
        archived=(ROOT/'assets/camera-nano-v1/source'/name) if name in old_nano else (ROOT/'assets/camera-latest-v1/source'/name) if name in old_latest else HERE/'source'/name
        assert sha(archived.read_bytes())==pin,name
    before=read('before.json');assert before['previous_source_pins']==old_latest
    assert before['starting_commit']=='698899ea8d3923bee4aae2c6ea31c6468fc67396' and len(before['prior_indexes'])==74
    old_count=0
    for name,pin in before['prior_indexes'].items():
        path=ROOT/name;assert sha(path.read_bytes())==pin
        for member,item in json.loads(path.read_text()).items():
            data=(path.parent/member).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],member
            old_count+=1
    assert old_count==before['prior_evidence_entries']==5716
    assert read('core-output-sha256.json')==json.loads((ROOT/'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert read('immutability.json')==dict(previous_source_files=117,prior_indexes=74,prior_evidence_entries=5716,core_outputs=245,all_unchanged=True)

    sys.path.insert(0,str(ROOT/'integrations/onnx'))
    spec=importlib.util.spec_from_file_location('packet_nano_repeat',ROOT/'integrations/onnx/camera_nano_repeat_check.py')
    c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c);c.check_sources()
    pilot_before=read('pilot-before.json')
    for name,pin in pilot_before['source_sha256'].items():assert pins[name]==pin
    pilot=read('pilot.json');pilot_serial=read('pilot-serial.json')
    pilot_pins=dict(c.nano.expected('image'),**{name:c.digest(ROOT/name) for name in c.PILOT+c.EXTRA})
    c.audit_profile(pilot,pilot_serial,pilot_pins,False,False)
    saved=json.loads((ROOT/'assets/camera-sequence-v1/recorded/exports/first-0.json').read_text())
    c.nano.check_image(pilot_serial,saved,parallel=False)
    assert c.measurement_bits(read('pilot-original.json')['detections'])==c.measurement_bits(pilot_serial['base_prediction']['views'][0]['prediction']['detections'])
    assert pilot_before['model_forwards']==13 and pilot['execution']['model_forwards']==8
    protocol=read('validation/before-inference.json')
    for name,pin in protocol['source_sha256'].items():assert pins[name]==pin
    assert protocol['inference_warmup'] is False and protocol['new_holdout'] is False and protocol['policy']==c.freshness.POLICY
    assert protocol['scheduler_order']==['fifo','latest'] and protocol['order_by_scheduler']==dict(fifo=c.ORDER,latest=c.ORDER)
    assert sha(raw('initial-repeat-checker.py'))==protocol['source_sha256']['integrations/onnx/camera_nano_repeat_check.py']==c.digest(c.__file__)
    assert sha(raw('initial-repeat-clock.py'))==protocol['source_sha256']['integrations/onnx/camera_nano_repeat_clock.py']==c.digest(c.HERE/'camera_nano_repeat_clock.py')
    references={};profile_count=0;parallel_count=0;pairs=[]
    for selected in protocol['image_selection']:
        name=selected['name'];reference=read('validation/references/'+name+'.json')
        saved_path=(ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{name[4:]}.json') if name.startswith('tum-') else ROOT/f'assets/camera-analysis-v1/publication-repair/regression/first/{name}.json'
        assert sha(saved_path.read_bytes())==selected['reference_report_sha256']
        saved=json.loads(saved_path.read_text());c.nano.check_image(reference,saved,parallel=False)
        assert selected['image_sha256']==saved['source_image_sha256'];references[name]=reference
        numeric=[];parallel=[]
        for repeat,reference_first in [('first',False),('repeat',True)]:
            profile=read(f'validation/profiles/{name}-{repeat}.json');c.audit_profile(profile,reference,c.expected(),True,reference_first)
            numeric.append(c.operator.numerical_projection(profile));profile_count+=profile['execution']['model_forwards']
            a=sum(v['profile']['forward_wall_ns'] for v in profile['views']);b=sum(v['reference_forward_wall_ns'] for v in profile['views'])
            pairs.append(dict(report=f'{name}-{repeat}.json',optimized_observed_ns=a,ordinary_reference_ns=b,reduction_fraction=1-a/b,reference_first=reference_first))
            image=read(f'validation/images/{name}-{repeat}.json');c.check_image(image,saved,reference)
            parallel_count+=image['execution']['model_forwards'];parallel.append(c.nano.previous.stripped(image))
        assert numeric[0]==numeric[1] and parallel[0]==parallel[1]
    assert len(references)==16 and profile_count==256 and parallel_count==128
    source=json.loads((ROOT/'assets/camera-sequence-v1/sequence.json').read_text())
    assert protocol['source_sequence_sha256']==c.nano.previous.SOURCE_SHA
    trials=read('validation/trials.json');assert [(t['scheduler'],t['label']) for t in trials]==[(s,l) for s in ('fifo','latest') for l in c.ORDER]
    timed=0;comparison=[]
    for trial in trials:
        scheduler,label=trial['scheduler'],trial['label'];optimized=label.startswith('optimized')
        mode='optimized' if optimized else 'baseline' if scheduler=='fifo' else 'latest'
        report=read(f'validation/{scheduler}/{label}.json');inspection=read('validation/'+mode+'-inspection.json')
        receipt=read('validation/'+mode+'-executable.json')
        assert receipt==protocol['executable_receipts'][mode] and report['compiled_sources']==receipt['compiled_sources']
        assert report['compiled_sources']==(c.expected() if optimized else c.nano.expected('runtime') if scheduler=='fifo' else c.latest.expected())
        assert trial==read(f'validation/{scheduler}/{label}-trial.json')
        assert trial['model_forwards']==report['model_forwards'] and trial['summary']==report['summary']
        assert report['source_sequence_sha256']==c.nano.previous.SOURCE_SHA
        assert report['startup_before_schedule_wall_s']+report['schedule_wall_s']<=trial['external_wall_s']+.01
        if scheduler=='fifo':manifest,_,_=(c.clock.audit_fifo if optimized else c.nano.clock.audit_measured)(report,source,inspection,c.freshness,c.tracking)
        elif optimized:manifest=c.clock.audit_latest(report,source,inspection,[references[f'tum-{i}'] for i in range(8)],c.expected,c.check_image)
        else:manifest=c.latest.audit(report,source,inspection,[references[f'tum-{i}'] for i in range(8)])
        assert manifest==read(f'validation/{scheduler}/{label}-guard-input.json')
        assert read(f'validation/{scheduler}/{label}-guard-result.json')['results']==[e['output'] for row in report['events'] for e in row['measured']['guard_events']]
        for row in report['events']:
            if row['measured']['analysis'] is not None:
                i=row['source']['event_index'];saved=json.loads((ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json').read_text())
                (c.check_image if optimized else c.nano.check_image)(row['measured']['analysis'],saved,references[f'tum-{i}'])
        s=report['summary'];timed+=report['model_forwards']
        comparison.append(dict(scheduler=scheduler,label=label,source_images=8,inferred_images=s['inferred_images'],superseded_images=s.get('superseded_images',0),
            available_results=s['available_at_check_images'] if scheduler=='fifo' else s['available_at_dispatched_checks'],inferred_but_late_images=s['inferred_but_late_images'],
            processing_s=[row['measured']['timing']['read_analysis_and_probe_wall_s'] for row in report['events'] if row['measured']['analysis'] is not None]))
    summary=read('validation/summary.json');assert summary['pass_checks'] and summary['comparison']==comparison
    assert summary['timed_model_forwards']==timed==124 and summary['actual_model_forwards']==timed+profile_count+parallel_count+64==572
    assert summary['raw_bitwise_pairs']==128 and summary['raw_outputs_per_view']==301665 and summary['exact_guard_replays']==8
    assert summary['fresh_quality_scoring'] is False and summary['new_holdout'] is False and summary['driving_integration'] is False
    for scheduler in ('fifo','latest'):
        b=[r['processing_s'][0] for r in comparison if r['scheduler']==scheduler and r['label'].startswith('baseline')]
        o=[r['processing_s'][0] for r in comparison if r['scheduler']==scheduler and r['label'].startswith('optimized')]
        assert summary['first_image_timings'][scheduler]==dict(baseline_first_image_s=b,optimized_first_image_s=o,mean_reduction_fraction=1-statistics.mean(o)/statistics.mean(b))
    assert [r['available_results'] for r in comparison]==[1,1,1,1,3,0,1,1]
    recorded=read('serial-profile-summary.json');assert sorted(recorded['pairs'],key=lambda p:p['report'])==sorted(pairs,key=lambda p:p['report'])
    assert recorded['pair_count']==32 and recorded['faster_pairs']==sum(p['reduction_fraction']>0 for p in pairs)==31
    assert recorded['aggregate_reduction_fraction']==1-sum(p['optimized_observed_ns'] for p in pairs)/sum(p['ordinary_reference_ns'] for p in pairs)
    assert recorded['median_reduction_fraction']==statistics.median(p['reduction_fraction'] for p in pairs)
    control=read('controls/nano-repeat-summary.json')
    assert control['pass_checks'] and control['valid_model_forwards']==0 and control['common_rejected_inputs']==21 and len(control['additional_rejections'])==5
    assert control['checker_sha256']==c.digest(c.__file__) and control['clock_checker_sha256']==c.digest(c.HERE/'camera_nano_repeat_clock.py')
    assert control['executable_sha256']==protocol['executable_receipts']['optimized']['executable_sha256']
    assert c.clock.audit_latest(read('controls/latest-no-images-result.json'),read('controls/no-images.json'),read('controls/latest-no-images-inspection.json'),expected=c.expected,check_image=c.check_image)==read('controls/latest-no-images-guard-input.json')
    print(json.dumps(dict(pass_checks=True,packet_files=len(files),prior_evidence_entries=old_count,unique_model_forwards_audited=585,
        raw_f32_value_pairs=128*301665,exact_guard_replays=8,model_execution_this_check=False)))


if __name__=='__main__':main()
