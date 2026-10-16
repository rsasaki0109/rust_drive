from pathlib import Path
import difflib
import gzip
import hashlib
import json
import re
import tarfile

r = Path('/workspace/rust_drive')
b = r/'artifacts/projective-fusion-bias-v1'
p = r/'assets/projective-fusion-bias-v1'
cases = ('tum-fr1-room-temporal', 'tum-fr1-desk2-independent', 'tum-fr1-rpy-heldout')


def sha(body):
    return hashlib.sha256(body).hexdigest()


def doc(path):
    return json.loads(path.read_text())


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def copy(source, target, compress=False):
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('xb') as stream:
        stream.write(gzip.compress(source.read_bytes(), mtime=0) if compress else source.read_bytes())


def archive(path):
    with tarfile.open(path) as stream:
        return {m.name: stream.extractfile(m).read() for m in stream.getmembers() if m.isfile()}


assert not p.exists()
historical = 0
for inventory in (r/'assets').glob('*/files.json'):
    for name, pin in doc(inventory).items():
        body = (inventory.parent/name).read_bytes()
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], (inventory, name)
        historical += 1
assert historical == 1600
epochs = {}
for label in ('before', 'candidate'):
    epochs[label] = archive(b/(label+'-source.tar.gz'))
    pins = doc(b/(label+'-source-sha256.json'))
    assert set(pins) == set(epochs[label])
    assert all(sha(epochs[label][n]) == h for n, h in pins.items())
diagnostic = archive(b/'diagnostic-source.tar.gz')
diagnostic_pins = doc(b/'diagnosis/freeze.json')['source_sha256']
assert set(diagnostic) == set(diagnostic_pins)
assert all(sha(diagnostic[n]) == h for n, h in diagnostic_pins.items())
assert all(sha(diagnostic[n]) == h for n, h in doc(b/'controls-first/freeze.json')['source_sha256'].items())
assert doc(b/'diagnosis/controls.json') == doc(b/'controls-first/controls.json')
assert doc(b/'final-bias-controls/controls.json') == doc(b/'controls-first/controls.json')
assert doc(b/'diagnosis/summary.json')['pass_diagnostic_integrity']
assert not doc(b/'diagnosis/summary.json')['root_accuracy_improvement_established']
for case in cases:
    d = doc(b/'diagnosis'/(case+'.json'))
    assert len(d['probes']) == 15 and len(d['model_updates']) == 179
    assert all(len(row['channels']) == 4 for row in d['probes'])
    assert d['reference_fusion_is_counterfactual'] and d['same_keyframe_schedule']
names = set(epochs['candidate']) | {'docs/projective-fusion.md'}
epochs['final'] = {n: (r/n).read_bytes() for n in sorted(names)}
for n, body in epochs['candidate'].items():
    if n.endswith(('.rs', '.py')):
        assert epochs['final'][n] == body, n
for n in ('integrations/projective-depth/Cargo.toml', 'integrations/projective-depth/Cargo.lock'):
    assert len({sha(data[n]) for data in epochs.values()}) == 1, n
binary = sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes())
assert sha((b/'candidate-binary').read_bytes()) == binary
for name, expected in (('unit', 56), ('core', 371)):
    log = Path('/tmp/rustdrive-fusion-'+name+'.log').read_text()
    assert sum(map(int, re.findall(r'test result: ok\. (\d+) passed;', log))) == expected
    assert 'FAILED' not in log and 'error:' not in log
    if name == 'core':
        assert log.count('"verified": true') == 49
core = doc(r/'assets/projective-range-v1/core-output-sha256.json')
assert len(core) == 245 and all(sha((r/n).read_bytes()) == h for n, h in core.items())
compatible = 0
for mode in ('depth', 'rgbd', 'timing'):
    folder = b/('compatibility-'+mode)
    a, f = doc(folder/'summary.json'), doc(folder/'freeze.json')
    assert a['pass_exact_compatibility'] and not a['numeric_reference_poses_decoded']
    assert f['binary_sha256'] == binary
    assert all(sha(epochs['final'][n]) == h for n, h in f['source_sha256'].items())
    compatible += a['identical_outcomes']
assert compatible == 1440
analytic = doc(b/'analytic/summary.json')
assert len(analytic['rejected_transport_cases']) == 64
assert analytic['fusion_sampling'] == 'visible_mean' and analytic['binary_sha256'] == binary
assert 'nearest-Z sampling substituted for visible mean' in doc(b/'analytic-raw/analytic-audit.json')['rejected_corruptions']
control = doc(b/'fusion-controls/summary.json')
assert control['pass_fusion_controls'] and control['accepted_outcomes_identical_after_rejection'] == 12
assert all(v['reference_warps'] >= 1 and v['max_position_error_m'] <= .01 and v['max_rotation_error_rad'] <= .01 for v in control['cases'].values())
for folder in ('fusion-controls', 'final-bias-controls'):
    f = doc(b/folder/'freeze.json')
    assert all(sha(epochs['final'][n]) == h for n, h in f['source_sha256'].items())
    if 'binary_sha256' in f:
        assert f['binary_sha256'] == binary
adoption = dict(schema_version=1, design_sha256=sha((b/'design.json').read_bytes()),
    criterion=doc(b/'design.json')['adoption'], baseline={}, checks={}, pass_default_adoption=False,
    default_changed=False, physical_accuracy_claim=False)
scores = {}
systems = 0
for case, label, expected, threshold in zip(cases, ('room', 'desk2', 'rpy'), (67, 161, 74), (65, 161, 74), strict=True):
    folder = b/'recorded'/case
    rawfolder = b/('raw-'+label)/case
    ledger, result, freeze = (doc(folder/n) for n in ('sensor-only.json', 'results.json', 'freeze.json'))
    raw, rawfreeze = doc(rawfolder/'audit.json'), doc(rawfolder/'freeze.json')
    assert len(ledger['frames']) == 180 and not ledger['ground_truth_parsed']
    assert [row['source_index'] for row in ledger['frames']] == list(range(100, 280))
    assert [row['update'] for row in ledger['frames']] == [row['update'] for row in result['frames']]
    s = result['summary']
    assert s['root_accurate'] == expected and s['accepted'] == 179 and not s['lost'] and not s['complete_accuracy_pass']
    assert raw['pass_raw_mathematics'] and raw['summary']['physical_information_checked'] and not raw['summary']['ground_truth_decoded']
    assert raw['summary']['frames'] == 180 and raw['summary']['fused_model_generations'] == 179
    assert raw['summary']['returned_pose_equations'] == 179
    assert raw['summary']['raw_iteration_equations'] == s['independently_checked_normal_steps']
    assert rawfreeze['original_sensor_ledger_sha256'] == sha((folder/'sensor-only.json').read_bytes())
    for f in (freeze, rawfreeze):
        assert all(sha(epochs['candidate'][n]) == h for n, h in f['source_sha256'].items())
    assert freeze['binary_sha256'] == binary and freeze['fusion_sampling'] == 'visible_mean'
    systems += raw['summary']['raw_iteration_equations']+179
    scores[case] = s
    baseline_file = r/'assets/projective-surface-v1/recorded/first-default'/case/'results.json.gz'
    baseline = json.loads(gzip.decompress(baseline_file.read_bytes()))['summary']
    adoption['baseline'][case] = dict(file=str(baseline_file.relative_to(r)), sha256=sha(baseline_file.read_bytes()))
    adoption['checks'][case] = dict(accuracy_count_pass=s['root_accurate'] >= threshold,
        maximum_position_pass=s['max_position_error_m'] <= baseline['max_position_error_m'],
        maximum_rotation_pass=s['max_rotation_error_rad'] <= baseline['max_rotation_error_rad'],
        tracking_pass=s['accepted'] == 179 and not s['lost'])
assert systems == 10778
assert all(v['accuracy_count_pass'] and v['tracking_pass'] for v in adoption['checks'].values())
assert not all(all(v.values()) for v in adoption['checks'].values())
assert all(not v['maximum_position_pass'] for v in adoption['checks'].values())
write(b/'adoption.json', adoption)
finalpins = {n: sha(body) for n, body in epochs['final'].items()}
write(b/'final-source-sha256.json', finalpins)
with tarfile.open(b/'final-source.tar.gz', 'x:gz') as stream:
    for n in finalpins:
        stream.add(r/n, arcname=n, recursive=False)
patch = []
for n, body in epochs['final'].items():
    patch.extend(difflib.unified_diff(epochs['before'].get(n, b'').decode().splitlines(True), body.decode().splitlines(True), fromfile='a/'+n, tofile='b/'+n))
with (b/'change.patch').open('x') as stream:
    stream.write(''.join(patch))
for n in ('before-source.tar.gz', 'before-source-sha256.json', 'candidate-source.tar.gz', 'candidate-source-sha256.json',
          'diagnostic-source.tar.gz', 'final-source.tar.gz', 'final-source-sha256.json', 'design.json',
          'diagnostic-decision.json', 'adoption.json', 'change.patch'):
    copy(b/n, p/n)
for case, label in zip(cases, ('room', 'desk2', 'rpy'), strict=True):
    folder = p/'recorded'/case
    copy(b/'recorded'/case/'freeze.json', folder/'freeze.json')
    for n in ('sensor-only', 'results'):
        copy(b/'recorded'/case/(n+'.json'), folder/(n+'.json.gz'), True)
    for n in ('freeze.json', 'audit.json'):
        copy(b/('raw-'+label)/case/n, folder/('raw-'+n))
for name in ('controls-first', 'diagnosis', 'analytic', 'analytic-raw', 'fusion-controls', 'final-bias-controls',
             'compatibility-depth', 'compatibility-rgbd', 'compatibility-timing'):
    for source in sorted((b/name).rglob('*')):
        if source.is_file():
            target = p/'checks'/name/source.relative_to(b/name)
            compress = source.stat().st_size > 100000 and source.suffix == '.json'
            if compress:
                target = target.with_name(target.name+'.gz')
            copy(source, target, compress)
for log in sorted(Path('/tmp').glob('rustdrive-fusion-*.log')):
    copy(log, p/'logs'/log.name)
copy(Path(__file__), p/'seal.py')
write(p/'core-output-sha256.json', core)
validation = dict(native_tests=56, core_tests=371, total_rust_tests=427, scenario_replays=49,
    core_outputs_unchanged=245, compatibility_outcomes_identical=1440, raw_recorded_normal_systems=systems,
    raw_recorded_model_generations=537, raw_reference_view_warps=124,
    offline_counterfactual_model_generations=1611, malformed_transport_cases=64,
    accepted_synthetic_outcomes_identical_after_rejection=12,
    heldout_accuracy_claim=False, complete_physical_accuracy=False, default_adoption=False,
    prior_evidence_entries_unchanged=historical)
write(p/'validation.json', validation)
summary = dict(schema_version=1, implemented='optional visible_mean fusion sampling and offline fusion decomposition',
    default='nearest unchanged', binary_sha256=binary, source_members=len(finalpins),
    before_binary_sha256=sha((b/'before-binary').read_bytes()), recorded_scores=scores,
    default_adoption=False, physical_accuracy_improvement=False, data_role='three already viewed FR1 regression windows',
    diagnostic='constructed nearest-Z bias reduced; reference-transform probes do not isolate recorded drift',
    uncertainty='bounded heuristic frame support; no calibrated physical noise or map covariance',
    subjective_maturity_percent=20, goal_30_percent_complete=False, validation=validation)
write(p/'summary.json', summary)
inventory = {str(f.relative_to(p)): dict(bytes=f.stat().st_size, sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()}
write(p/'files.json', inventory)
print(json.dumps(dict(packet_files=len(inventory), packet_bytes=sum(v['bytes'] for v in inventory.values()),
    source_members=len(finalpins), prior_entries=historical, raw_normal_systems=systems, default_adoption=False)), flush=True)
