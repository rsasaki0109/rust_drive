from pathlib import Path
import difflib,gzip,hashlib,json,re,tarfile
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-camera-fit-v1';p=r/'assets/projective-camera-fit-v1'
cases=('tum-fr1-room-temporal','tum-fr1-desk2-independent','tum-fr1-rpy-heldout')
def sha(x):return hashlib.sha256(x).hexdigest()
def doc(x):return json.loads(x.read_text())
def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def copy(src,dst,compress=False):
    dst.parent.mkdir(parents=True,exist_ok=True)
    with dst.open('xb') as f:f.write(gzip.compress(src.read_bytes(),mtime=0) if compress else src.read_bytes())
def archive(path):
    with tarfile.open(path) as t:return {m.name:t.extractfile(m).read() for m in t.getmembers() if m.isfile()}
assert not p.exists()
old_entries=0
for inventory in (r/'assets').glob('*/files.json'):
    for n,v in doc(inventory).items():
        body=(inventory.parent/n).read_bytes();assert len(body)==v['bytes'] and sha(body)==v['sha256'],(inventory,n);old_entries+=1
assert old_entries==1774
epochs={}
for label in ('before','candidate','normalized','normalized-fixed'):
    epochs[label]=archive(b/(label+'-source.tar.gz'));pins=doc(b/(label+'-source-sha256.json'))
    assert set(pins)==set(epochs[label]) and all(sha(epochs[label][n])==h for n,h in pins.items())
assert doc(b/'before-source-sha256.json')==doc(r/'assets/projective-visibility-v1/final-source-sha256.json')
for archive_name,folder in (('controls-first-source','controls-first'),('audit-repaired-source','audit-first-repaired')):
    data=archive(b/(archive_name+'.tar.gz'));pins=doc(b/folder/'freeze.json')['source_sha256']
    assert set(data)==set(pins) and all(sha(data[n])==h for n,h in pins.items())
    epochs[archive_name]=data
names=set(epochs['normalized-fixed'])|{'docs/projective-camera-fit.md'}
epochs['final']={n:(r/n).read_bytes() for n in sorted(names)}
for n,body in epochs['normalized-fixed'].items():
    if n.endswith(('.rs','.py')):assert body==epochs['final'][n],n
for n,body in epochs['before'].items():
    if n.endswith('.rs') or n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):
        assert all(data[n]==body for data in (epochs['candidate'],epochs['normalized'],epochs['normalized-fixed'],epochs['final'])),n
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes())
assert binary==sha((b/'before-binary').read_bytes())==doc(b/'design.json')['binary_sha256']==doc(r/'assets/projective-visibility-v1/summary.json')['binary_sha256']
for name,expected in (('native',59),('verified-core',371)):
    log=Path('/tmp/rustdrive-camera-fit-'+name+'.log').read_text()
    assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',log)))==expected and 'FAILED' not in log
    if name=='verified-core':assert log.count('"verified": true')==49
core=doc(r/'assets/projective-visibility-v1/core-output-sha256.json')
assert len(core)==245 and all(sha((r/n).read_bytes())==h for n,h in core.items())
assert doc(b/'audit-repair.json')['repaired_first_audit_passed']
assert doc(b/'syntax-repair.json')['new_numeric_trials_before_fix']==0
assert 'TypeError' in Path('/tmp/rustdrive-camera-fit-audit-first.log').read_text()
assert 'SyntaxError' in Path('/tmp/rustdrive-camera-fit-normalized-diagnosis.log').read_text()
gauge=doc(b/'scale-gauge-first/summary.json')
assert gauge['counterexample_reproduced'] and gauge['initial_observable'] and not gauge['physical_scale_identifiable']
assert gauge['smaller_scale_cost_m2']<gauge['original_cost_m2'] and gauge['fit_status']=='at_bound'
for n,h in doc(b/'scale-gauge-first/freeze.json')['source_sha256'].items():
    assert any(n in data and sha(data[n])==h for data in epochs.values()),n
current_controls=doc(b/'normalized-controls/controls.json')
assert current_controls['noisy_rotation_scale_rejected'] and current_controls['normalized_scale_invariant']
outcomes={};systems=0
for label,folder,epoch,audit_folder,expected_systems in (
    ('physical','diagnosis','candidate','final-first-audit',390),
    ('raw_scaled','normalized-diagnosis','normalized-fixed','normalized-audit',406)):
    summary=doc(b/folder/'summary.json');freeze=doc(b/folder/'freeze.json');audit=doc(b/audit_folder/'raw-audit.json')
    assert not summary['trial_eligible'] and not summary['exported_trial_profile'] and not (b/folder/'trial-profile.json').exists()
    assert not summary['physical_calibration_certified'] and not summary['default_changed']
    assert freeze.get('fit_objective','physical')==label and not freeze['native_binary_executed'] and freeze['original_scoring_clocks_unchanged']
    assert all(sha(epochs[epoch][n])==h for n,h in freeze['source_sha256'].items())
    assert audit['pass_independent_camera_audit'] and audit['finite_difference_systems']==expected_systems and audit['validation_pose_systems']==90
    assert audit['fit_objective']==label and not audit['trial_eligible'];systems+=expected_systems
    for n,h in doc(b/audit_folder/'freeze.json')['source_sha256'].items():assert sha(epochs['final'][n])==h,n
    training=[];fits={};folds={};validation_flags={};samples=0
    for c in cases:
        receipts=doc(b/folder/c/'samples.json')['pairs'];assert len(receipts)==(30 if c in cases[:2] else 15)
        cf=doc(b/folder/c/'freeze.json');assert cf['manifest_sha256']==sha((r/'data'/c/'manifest.json').read_bytes())
        for receipt in receipts:
            cache=b/folder/c/receipt['cache_file'];assert sha(cache.read_bytes())==receipt['cache_sha256'] and receipt['frozen_samples']>=200;samples+=1
        result=doc(b/folder/(c+'-validation.json'));flags=dict(complete_pose_probes=result['original']['summary']['all_pose_probes_observable'] and result['candidate']['summary']['all_pose_probes_observable'])
        for metric in ('mean_robust_cost_m2','rms_m','median_position_correction_m','median_rotation_correction_rad'):
            old,new=result['original']['summary'][metric],result['candidate']['summary'][metric]
            flags[metric+'_nonincreasing']=old is not None and new is not None and new<=old
        assert flags==result['checks']==summary['validation'][c]['checks'];validation_flags[c]=flags
        if c in cases[:2]:
            result=doc(b/folder/(c+'-fit.json'));fits[c]=result['full'];folds[c]=result['leave_one_pair_out'];assert len(folds[c])==15
    assert samples==75
    joint=doc(b/folder/'joint-fit.json')
    import numpy as np
    agreement=float(np.max(abs(np.asarray(fits[cases[0]]['parameters'])-fits[cases[1]]['parameters'])))
    deviations={c:max(float(np.max(abs(np.asarray(f['parameters'])-fits[c]['parameters']))) for f in folds[c]) for c in cases[:2]}
    expected=dict(all_fits_converged_without_bound=joint['status']=='converged' and all(fits[c]['status']=='converged' and all(f['status']=='converged' for f in folds[c]) for c in cases[:2]),
        cross_record_agreement=agreement<=.01,leave_one_pair_out_stable=all(v<=.01 for v in deviations.values()),validation_nonincreasing=all(all(v.values()) for v in validation_flags.values()))
    assert expected==summary['checks'] and not all(expected.values())
    assert summary['maximum_between_record_log_difference']==agreement and summary['maximum_leave_one_pair_out_log_deviation']==deviations
    outcomes[label]=summary
assert systems==796
same=0
for c in cases:
    a,batches=doc(b/'diagnosis'/c/'samples.json')['pairs'],doc(b/'normalized-diagnosis'/c/'samples.json')['pairs'];assert a==batches
    for v in a:
        assert (b/'diagnosis'/c/v['cache_file']).read_bytes()==(b/'normalized-diagnosis'/c/v['cache_file']).read_bytes();same+=1
assert same==doc(b/'sample-compatibility.json')['identical_sample_batches']==75
finalpins={n:sha(body) for n,body in epochs['final'].items()};write(b/'final-source-sha256.json',finalpins)
with tarfile.open(b/'final-source.tar.gz','x:gz') as t:
    for n in finalpins:t.add(r/n,arcname=n,recursive=False)
patch=[]
for n,body in epochs['final'].items():patch.extend(difflib.unified_diff(epochs['before'].get(n,b'').decode().splitlines(True),body.decode().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
with (b/'change.patch').open('x') as f:f.write(''.join(patch))
for n in ('before-source.tar.gz','before-source-sha256.json','candidate-source.tar.gz','candidate-source-sha256.json','normalized-source.tar.gz','normalized-source-sha256.json',
    'normalized-fixed-source.tar.gz','normalized-fixed-source-sha256.json','controls-first-source.tar.gz','audit-repaired-source.tar.gz','final-source.tar.gz','final-source-sha256.json',
    'design.json','numerical-design.json','normalized-design.json','audit-repair.json','syntax-repair.json','sample-compatibility.json','probe-scale-gauge.py','change.patch'):
    copy(b/n,p/n)
for folder in ('controls-first','controls','diagnosis','audit-first','audit-first-repaired','scale-gauge-first','normalized-controls','final-first-audit','normalized-diagnosis','normalized-audit'):
    for src in sorted((b/folder).rglob('*.json')):
        dst=p/'checks'/folder/src.relative_to(b/folder);compress=src.stat().st_size>100000
        if compress:dst=dst.with_name(dst.name+'.gz')
        copy(src,dst,compress)
for log in sorted(Path('/tmp').glob('rustdrive-camera-fit-*.log')):copy(log,p/'logs'/log.name)
copy(Path(__file__),p/'seal.py');write(p/'core-output-sha256.json',core)
validation=dict(native_tests=59,core_tests=371,total_rust_tests=430,scenario_replays=49,unchanged_core_outputs=245,
    native_source_and_binary_unchanged=True,legacy_native_replays_performed_this_turn=False,
    historical_legacy_1440_receipt_retained=True,distinct_raw_pair_batches=75,identical_frozen_sample_caches=75,
    calibration_systems_independently_checked=796,validation_pose_systems_independently_checked=180,
    attempted_profiles=2,exported_profiles=0,operational_trajectories_run=0,new_heldout_data=False,
    original_failed_audit_and_syntax_epochs_retained=True,prior_evidence_entries_unchanged=old_entries)
write(p/'validation.json',validation)
summary=dict(schema_version=1,implemented='offline focal/depth correction, raw-unit scale gauge repair and independent full-fit audit',
    source_members=len(finalpins),binary_sha256=binary,native_tracker_changed=False,default_camera_changed=False,
    outcomes=outcomes,physical_calibration_certified=False,trajectory_trial_eligible=False,exported_profiles=0,
    data_role='same viewed FR1 records; disjoint training/validation frame endpoints, temporally correlated',
    limitation='conditional two-parameter fixed-correspondence model, provisional noise normalization; no measured camera certificate or root accuracy increase',
    subjective_maturity_percent=20,goal_30_percent_complete=False,validation=validation)
write(p/'summary.json',summary)
inventory={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()}
write(p/'files.json',inventory)
print(json.dumps(dict(packet_files=len(inventory),packet_bytes=sum(v['bytes'] for v in inventory.values()),source_members=len(finalpins),
    prior_entries=old_entries,calibration_systems=systems,validation_pose_systems=180,exported_profiles=0,default_camera_changed=False)),flush=True)
