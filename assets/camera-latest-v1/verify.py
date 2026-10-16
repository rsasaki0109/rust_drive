#!/usr/bin/env python3
"""Reconstruct latest-ready choices, measurements and guard outputs without models."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(data):return hashlib.sha256(data).hexdigest()


def raw(name):
    path = HERE/name
    if path.exists():return path.read_bytes()
    return gzip.decompress(Path(str(path)+'.gz').read_bytes())


def read(name):return json.loads(raw(name))


def main():
    index = read('files.json')
    assert set(index) == {str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p != HERE/'files.json'}
    for name,entry in index.items():
        data = (HERE/name).read_bytes()
        assert len(data) == entry['bytes'] and sha(data) == entry['sha256'], name
    for name,entry in read('raw-files.json').items():
        data = raw(name)
        assert len(data) == entry['bytes'] and sha(data) == entry['sha256'], name
    pins = read('source-sha256.json')
    old_pins = json.loads((ROOT/'assets/camera-nano-v1/source-sha256.json').read_text())
    for name,pin in pins.items():
        assert sha((ROOT/name).read_bytes()) == pin, name
        archived = (ROOT/'assets/camera-nano-v1/source'/name) if name in old_pins else HERE/'source'/name
        assert sha(archived.read_bytes()) == pin, name
    before = read('before.json'); old_count = 0
    assert before['previous_source_pins'] == old_pins
    assert len(before['prior_indexes']) == 73
    for name,pin in before['prior_indexes'].items():
        path = ROOT/name; assert sha(path.read_bytes()) == pin
        for member,entry in json.loads(path.read_text()).items():
            data = (path.parent/member).read_bytes()
            assert len(data) == entry['bytes'] and sha(data) == entry['sha256'], member
            old_count += 1
    assert old_count == before['prior_evidence_entries'] == 5618
    assert read('core-output-sha256.json') == json.loads((ROOT/'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert read('immutability.json') == dict(previous_source_files=110,prior_indexes=73,prior_evidence_entries=5618,core_outputs=245,all_unchanged=True)

    sys.path.insert(0,str(ROOT/'integrations/onnx'))
    spec = importlib.util.spec_from_file_location('latest_packet',ROOT/'integrations/onnx/camera_latest_check.py')
    c = importlib.util.module_from_spec(spec); spec.loader.exec_module(c)
    c.check_shared_controller()
    protocol = read('validation/before-inference.json')
    for name,pin in protocol['source_sha256'].items():assert pins[name] == pin
    assert protocol['order'] == c.ORDER and protocol['policy'] == c.freshness.POLICY
    assert protocol['inference_warmup'] is False and protocol['new_holdout'] is False
    assert sha(raw('initial-checker.py')) == protocol['source_sha256']['integrations/onnx/camera_latest_check.py'] == c.digest(c.__file__)
    source = json.loads((ROOT/'assets/camera-sequence-v1/sequence.json').read_text())
    assert protocol['source_sequence_sha256'] == c.nano.previous.SOURCE_SHA
    references = []
    for i in range(8):
        reference = read(f'validation/references/{i}.json')
        saved = json.loads((ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json').read_text())
        c.nano.check_image(reference,saved,parallel=False); references.append(reference)
    trials = read('validation/trials.json'); assert [row['label'] for row in trials] == c.ORDER
    reconstructed = []; timed_forwards = 0
    for trial in trials:
        label = trial['label']; mode = label.split('-')[0]
        report = read('validation/'+label+'.json')
        inspection = read('validation/'+mode+'-inspection.json')
        receipt = read('validation/'+mode+'-executable.json')
        assert receipt == protocol['executable_receipts'][mode]
        assert report['compiled_sources'] == receipt['compiled_sources'] == (c.expected() if mode == 'latest' else c.nano.expected('runtime'))
        assert report['source_sequence_sha256'] == protocol['source_sequence_sha256']
        assert trial == read('validation/'+label+'-trial.json')
        assert trial['model_forwards'] == report['model_forwards'] and trial['summary'] == report['summary']
        assert report['startup_before_schedule_wall_s']+report['schedule_wall_s'] <= trial['external_wall_s']+.01
        if mode == 'latest':manifest = c.audit(report,source,inspection,references)
        else:
            manifest,_,_ = c.nano.clock.audit_measured(report,source,inspection,c.freshness,c.tracking)
            for row in report['events']:
                if row['measured']['analysis'] is not None:
                    i = row['source']['event_index']
                    c.nano.check_image(row['measured']['analysis'],json.loads((ROOT/f'assets/camera-sequence-v1/recorded/exports/first-{i}.json').read_text()),references[i])
        assert manifest == read('validation/'+label+'-guard-input.json')
        assert read('validation/'+label+'-guard-result.json')['results'] == [entry['output'] for row in report['events'] for entry in row['measured']['guard_events']]
        summary = report['summary']; timed_forwards += report['model_forwards']
        reconstructed.append(dict(label=label,source_images=8,model_forwards=report['model_forwards'],inferred_images=summary['inferred_images'],
            superseded_images=summary.get('superseded_images',0),available_results=summary['available_at_dispatched_checks'] if mode == 'latest' else summary['available_at_check_images'],
            inferred_but_late_images=summary['inferred_but_late_images'],selected_indices=[row['source']['event_index'] for row in report['events']],
            processing_s=[row['measured']['timing']['read_analysis_and_probe_wall_s'] for row in report['events'] if row['measured']['analysis'] is not None]))
    summary = read('validation/summary.json')
    assert summary['pass_checks'] and summary['comparison'] == reconstructed
    assert summary['timed_model_forwards'] == timed_forwards == 56
    assert summary['current_reference_model_forwards'] == 32 and summary['actual_model_forwards'] == 88
    assert summary['independent_guard_replays'] == 4 and summary['dropped_images_counted_unavailable'] is True
    assert summary['new_holdout'] is False and summary['new_accuracy_scoring'] is False and summary['driving_integration'] is False
    assert [row['available_results'] for row in reconstructed] == [0,0,0,1]
    assert [row['inferred_but_late_images'] for row in reconstructed] == [2,0,0,0]

    control = read('controls/latest-controls.json'); assert control['pass_checks'] and control['valid_model_forwards'] == 0
    assert control['checker_sha256'] == c.digest(c.__file__)
    assert control['executable_sha256'] == protocol['executable_receipts']['latest']['executable_sha256']
    assert control['shared_rejected_inputs'] == 21 and len(control['additional_rejections']) == 4 and control['output_protections'] == 2
    no_images = read('controls/no-images-result.json')
    assert c.audit(no_images,read('controls/no-images.json'),read('controls/no-images-inspection.json')) == read('controls/no-images-guard-input.json')
    assert read('controls/no-images-guard-result.json')['results'] == [e['output'] for row in no_images['events'] for e in row['measured']['guard_events']]
    remote = read('nano-remote-summary.json'); log = raw('nano-remote-job.log').decode()
    line = next(line for line in log.splitlines() if '"actual_model_forwards": 244' in line)
    assert json.loads(line[line.index('{'):]) == remote['result']
    assert remote['head_sha'] == before['starting_commit'] == 'd480e16dfdf274c0108875a0afec1a35d0f4a9af'
    assert remote['run_id'] == 38962788218 and remote['job_id'] == 117181767087
    assert remote['result']['deadline_gates']['nano-first'] is False and remote['result']['deadline_gates']['nano-repeat'] is False
    assert remote['result']['actual_availability']['nano-first'] == remote['result']['actual_availability']['nano-repeat'] == 0
    print(json.dumps(dict(pass_checks=True,packet_files=len(index),prior_evidence_entries=old_count,
        actual_local_model_forwards_audited=88,previous_pr_remote_forwards_audited=244,exact_guard_replays=4,
        model_execution_this_check=False)))


if __name__ == '__main__':main()
