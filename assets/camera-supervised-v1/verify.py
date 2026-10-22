#!/usr/bin/env python3
"""Re-audit measured supervision and preserved regressions without inference."""
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
    path=HERE/name
    return path.read_bytes() if path.exists() else gzip.decompress(Path(str(path)+'.gz').read_bytes())


def read(name):return json.loads(raw(name))


def main():
    index=read('files.json')
    assert set(index)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p!=HERE/'files.json'}
    for name,item in index.items():
        data=(HERE/name).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    for name,item in read('raw-files.json').items():
        data=raw(name);assert len(data)==item['bytes'] and sha(data)==item['sha256'],name
    before=read('before.json');old=json.loads((ROOT/'assets/camera-nano-repeat2x-v1/source-sha256.json').read_text())
    assert before['starting_commit']=='7854b0d4701c2fe854cf27bac966e62cbc7b1503' and before['previous_sources']==old
    assert len(old)==130 and len(before['prior_indexes'])==75
    assert set(before['prior_indexes']) <= {str(p.relative_to(ROOT)) for p in (ROOT/'assets').rglob('files.json') if p!=HERE/'files.json'}
    old_count=0
    for name,pin in before['prior_indexes'].items():
        path=ROOT/name;assert sha(path.read_bytes())==pin,name
        for member,item in json.loads(path.read_text()).items():
            data=(path.parent/member).read_bytes();assert len(data)==item['bytes'] and sha(data)==item['sha256'],member
            old_count+=1
    assert old_count==5917
    archive_sets=[(ROOT/'assets'/name,json.loads((ROOT/'assets'/name/'source-sha256.json').read_text()))
                  for name in ['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1']]
    pins=read('source-sha256.json');assert len(pins)==137
    for name,pin in pins.items():
        assert sha((ROOT/name).read_bytes())==pin,name
        directory=next((folder for folder,mapping in archive_sets if name in mapping),HERE)
        assert sha((directory/'source'/name).read_bytes())==pin,name
    assert read('immutability.json')==dict(previous_source_files=130,prior_indexes=75,prior_evidence_entries=5917,core_outputs=245,all_unchanged=True)
    assert read('core-output-sha256.json')==before['core_outputs']==json.loads((ROOT/'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert not any(p.suffix in ('.onnx','.jpg','.exe') for p in HERE.rglob('*') if p.is_file())
    assert {str(p.relative_to(HERE)) for p in HERE.rglob('*.png')}=={'controls/inputs/0.png','controls/inputs/1.png','controls/inputs/invalid.png'}
    for i in range(2):assert (HERE/f'controls/inputs/{i}.png').read_bytes()==(ROOT/f'assets/camera-sequence-v1/controls/inputs/{i}.png').read_bytes()

    sys.path.insert(0,str(ROOT/'integrations/onnx'))
    import camera_supervised_check as c
    protocol=read('validation/before-inference.json')
    assert protocol['order']==c.ORDER and protocol['policy']==c.freshness.POLICY
    assert protocol['inference_warmup'] is False and protocol['timing_success_required_for_execution'] is False
    assert protocol['fault_offsets_s']==['0.010','0.015'] and protocol['fault_kinds']==['failed','missing'] and protocol['fault_delay_gate_s']==.01
    for name,pin in protocol['source_sha256'].items():assert pins[name]==pin
    assert sha(raw('validation/initial-checker.py'))==pins['integrations/onnx/camera_supervised_check.py']==c.digest(c.__file__)
    source=json.loads((ROOT/'assets/camera-sequence-v1/sequence.json').read_text());fault=read('validation/fault-sequence.json')
    assert sha((ROOT/'assets/camera-sequence-v1/sequence.json').read_bytes())==protocol['source_sequence_sha256']==c.nano.previous.SOURCE_SHA
    assert sha(raw('validation/fault-sequence.json'))==protocol['fault_sequence_sha256']
    assert fault['events'][0]['timestamp_s']==source['events'][0]['timestamp_s']
    assert fault['events'][0]['input']['sha256']==source['events'][0]['input']['sha256']
    assert c.slots_for(fault)[0]['release_offset_s']==0.
    assert [e['input']['kind'] for e in fault['events']]==['image','failed','missing']
    refs={}
    for i,e in enumerate(source['events']):
        serial=read(f'validation/references/{i}.json')
        c.nano.check_image(serial,json.loads((ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json').read_text()),parallel=False)
        refs[e['input']['sha256']]=serial
    trials=read('validation/trials.json');assert [t['label'] for t in trials]==c.ORDER
    reconstructed=[];timed=0
    for trial in trials:
        label=trial['label'];prefix='fault-' if label.startswith('fault-') else '';mode=label.removeprefix('fault-').split('-')[0]
        r=read('validation/'+label+'.json');inspection=read('validation/'+prefix+mode+'-inspection.json');original=fault if prefix else source
        receipt=read('validation/'+mode+'-executable.json')
        assert receipt==protocol['executable_receipts'][mode] and r['compiled_sources']==receipt['compiled_sources']
        assert trial==read('validation/'+label+'-trial.json') and trial['summary']==r['summary'] and trial['model_forwards']==r['model_forwards']
        assert r['source_sequence_sha256']==inspection['source_sequence_sha256']==(protocol['fault_sequence_sha256'] if prefix else protocol['source_sequence_sha256'])
        assert r['startup_before_schedule_wall_s']+r['schedule_wall_s']<=trial['external_wall_s']+.01
        if mode=='supervised':
            manifest=c.audit(r,original,inspection,refs);outputs=[row['change']['output'] for row in r['trace'] if row['change']['kind']=='guard']
            available=r['summary']['available_result_images'];inferred=r['summary']['completed_images'];metrics=c.supervision_metrics(r)
            delays=[row['observed_s']-r['source_slots'][row['change']['event_index']]['release_offset_s'] for row in r['trace']
                    if row['change']['kind']=='guard' and row['change']['reason']=='source_barrier' and row['change']['event']['input']['kind']=='failed']
        else:
            reference=[refs[e['input']['sha256']] for e in original['events'] if e['input']['kind']=='image']
            manifest=c.latest.audit(r,original,inspection,reference)
            outputs=[e['output'] for row in r['events'] for e in row['measured']['guard_events']]
            available=r['summary']['available_at_dispatched_checks'];inferred=r['summary']['inferred_images'];metrics=None
            delays=[row['measured']['guard_events'][0]['event']['now_s']-row['source']['release_offset_s'] for row in r['events'] if original['events'][row['source']['event_index']]['input']['kind']=='failed']
        assert manifest==read('validation/'+label+'-guard-input.json')
        assert read('validation/'+label+'-guard-result.json')['results']==outputs
        timed+=r['model_forwards']
        reconstructed.append(dict(label=label,source_images=sum(e['input']['kind']=='image' for e in original['events']),model_forwards=r['model_forwards'],
            inferred_images=inferred,available_results=available,superseded_images=r['summary']['superseded_images'],
            inferred_but_late_images=r['summary']['inferred_but_late_images'],fault_delay_s=delays,supervision=metrics))
    summary=read('validation/summary.json')
    assert summary['pass_checks'] and summary['comparison']==reconstructed
    assert timed==summary['timed_model_forwards']==68 and summary['current_reference_model_forwards']==32 and summary['actual_model_forwards']==100
    assert summary['independent_guard_replays']==8 and summary['current_serial_measurement_bits_exact'] and summary['dropped_images_counted_unavailable']
    assert [r['available_results'] for r in reconstructed[:4]]==[2,0,0,0]
    assert [r['inferred_but_late_images'] for r in reconstructed[:4]]==[0,0,1,0]
    for r in reconstructed:
        if r['supervision'] is not None:
            assert not r['supervision']['observed_poll_target_met'] and not r['supervision']['hard_real_time_guarantee']
    for r in reconstructed[5:7]:
        assert r['supervision']['observed_fault_10ms_gate'] and all(o['delivered_during_actual_inference'] for o in r['supervision']['fault_observations'])
    for r in reconstructed[1:3]:
        assert r['supervision']['expiry_observations'] and all(o['observed_during_actual_inference'] for o in r['supervision']['expiry_observations'])
    for key in ('new_holdout','new_accuracy_scoring','driving_integration','hard_real_time_guarantee'):assert summary[key] is False
    control=read('controls/supervised-controls.json')
    assert control['pass_checks'] and control['shared_rejected_inputs']==21 and len(control['additional_rejections'])==4 and control['valid_model_forwards']==0
    assert control['output_protections']==2 and control['model_free_guard_replays']==1
    assert control['checker_sha256']==c.digest(c.__file__) and control['executable_sha256']==protocol['executable_receipts']['supervised']['executable_sha256']
    empty=read('controls/no-images-result.json')
    assert c.audit(empty,read('controls/no-images.json'),read('controls/no-images-inspection.json'))==read('controls/no-images-guard-input.json')
    assert read('controls/no-images-guard-result.json')['results']==[e['change']['output'] for e in empty['trace'] if e['change']['kind']=='guard']
    checks=read('checks/results.json');assert checks['root_tests']==371 and checks['scenario_replays']==49
    root_log=raw('checks/root.log').decode()
    assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',root_log)))==371
    assert root_log.count('"verified": true')==49 and b'47 passed; 0 failed' in raw('checks/native-tests.log')
    assert b'Ran 12 tests' in raw('checks/python-tests.log') and checks['new_native_tests']==8 and checks['reused_native_tests']==39
    ci=read('parent-ci-snapshot.json');assert ci['head_sha']=='d480e16dfdf274c0108875a0afec1a35d0f4a9af' and ci['run_id']==38962819050
    assert len(ci['jobs'])==10 and all(j['conclusion']=='success' and j['status']=='completed' for j in ci['jobs'])
    assert ci['new_supervised_variant_remote_ci_observed'] is False
    print(json.dumps(dict(pass_checks=True,packet_files=len(index),prior_evidence_entries=old_count,actual_model_forwards_audited=100,
        exact_measured_guard_replays=8,model_free_guard_replays=1,model_execution_this_check=False)))


if __name__=='__main__':main()
