from pathlib import Path
import json,gzip,hashlib,tarfile,difflib,re,ast
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-information';p=r/'assets/projective-information-v1';cases=('tum-fr1-room-temporal','tum-fr1-desk2-independent')
def sha(x):return hashlib.sha256(x).hexdigest()
def doc(x):return json.loads(x.read_text())
def write(path,value):
 path.parent.mkdir(parents=True,exist_ok=True)
 with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def copy(src,dst,zip=False):
 dst.parent.mkdir(parents=True,exist_ok=True)
 with dst.open('xb') as f:f.write(gzip.compress(src.read_bytes(),mtime=0) if zip else src.read_bytes())
def archive(path):
 with tarfile.open(path) as t:return {m.name:t.extractfile(m).read() for m in t.getmembers() if m.isfile()}
assert not p.exists()
assert doc(b/'replay-proof.json')['complete_outcomes_identical']==1080 and doc(b/'cold-source-replay.json')['outcomes_identical']==360
for label,n in [('unit',45),('core-complete',371)]:
 log=Path('/tmp/rustdrive-information-'+label+'.log').read_text();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',log)))==n and 'FAILED' not in log
 if label=='core-complete':assert log.count('"verified": true')==49
entries=0
for m in (r/'assets').glob('*/files.json'):
 for n,v in doc(m).items():
  body=(m.parent/n).read_bytes();assert len(body)==v['bytes'] and sha(body)==v['sha256'];entries+=1
assert entries==1231
before=archive(b/'before-source.tar.gz');candidate=archive(b/'candidate-source.tar.gz');cold={}
for label,data in [('before',before),('candidate',candidate)]:
 pins=doc(b/(label+'-source-sha256.json'));assert set(pins)==set(data)
 for n,h in pins.items():assert sha(data[n])==h
for name,folder in [('plant','heldout'),('rpy','heldout-rpy')]:
 cold[name]=archive(b/folder/'source.tar.gz');d=doc(b/folder/'design.json');assert set(cold[name])==set(d['source_sha256'])
 for n,h in d['source_sha256'].items():assert sha(cold[name][n])==h,(name,n)
assert not doc(b/'heldout/qualification-failure.json')['pixels_decoded']
assert len(doc(b/'heldout/qualification-failure.json')['failing_fixed_window_indices'])==6
names=set(candidate)|set(cold['rpy'])|{'scripts/check-projective-information-calibration.py','scripts/repair-projective-heldout-inventory.py','scripts/check-projective-calibration-math.py','scripts/check-projective-report-write.py','docs/projective-information.md','data/tum-fr1-rpy-heldout/SOURCE.md'}
pins={n:sha((r/n).read_bytes()) for n in sorted(names)}
for n in names:
 if n.endswith('.rs'):assert pins[n]==sha(candidate[n])==sha(cold['rpy'][n]),n
for n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):assert pins[n]==sha(before[n])
cal=b/'calibration';f=doc(cal/'freeze.json')
for n,h in f['source_sha256'].items():assert sha((cal/'first-source.py').read_bytes() if n=='scripts/check-projective-information-calibration.py' else cold['rpy'][n])==h,n
assert doc(cal/'serialization-repair.json')['recomputed_original_values_match']
for mode in ('depth','timing'):
 v=doc(cal/'heldout-results.json')['results'][mode];assert v['nominal_coverage']==0 and v['empirically_inflated_coverage']==1 and not v['empirical_gate_pass'] and not v['calibrated_probability_claim']
assert doc(b/'calibration-helper-regression/summary.json')['unchanged_numeric_results']
assert doc(b/'report-write-controls/summary.json')['pass_atomic_report_controls']
assert doc(b/'calibration-math-final/summary.json')['pass_calibration_math']
assert doc(b/'inventory-regression/summary.json')['manifest_matches_repaired_original']
assert doc(b/'scorer-regression-final/summary.json')['known_updates_checked']==716
with tarfile.open(b/'final-source.tar.gz','w:gz') as t:
 for n in sorted(names):t.add(r/n,arcname=n)
write(b/'final-source-sha256.json',pins)
patch=[]
for n in sorted(names):patch.extend(difflib.unified_diff(before.get(n,b'').decode().splitlines(True),(r/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
(b/'change.patch').write_text(''.join(patch))
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes());scores={};geometry=photo=0
for mode,expected in [('depth',74),('timing',61)]:
 folder=b/'heldout-rpy'/mode;freeze=doc(folder/'freeze.json');ledger=doc(folder/'sensor-only.json');report=doc(folder/'results.json');raw=doc(folder/'raw-audit.json')
 assert freeze['binary_sha256']==binary and not ledger['ground_truth_parsed'] and report['summary']['root_accurate']==expected and report['summary']['accepted']==179 and not report['summary']['lost']
 assert len(ledger['frames'])==180 and [x['update'] for x in ledger['frames']]==[x['update'] for x in report['frames']]
 assert raw['pass_raw_mathematics'] and not raw['numeric_reference_poses_decoded'];scores[mode]=report['summary'];geometry+=raw['geometric_systems']
 if mode=='timing':photo+=raw['geometric_systems']
 copy(folder/'freeze.json',p/'heldout/rpy'/mode/'freeze.json');copy(folder/'raw-audit.json',p/'heldout/rpy'/mode/'raw-audit.json')
 for n in ('sensor-only','results'):copy(folder/(n+'.json'),p/'heldout/rpy'/mode/(n+'.json.gz'),True)
assert (geometry,photo)==(7333,3829)
for mode,prior in [('default','assets/projective-depth-rust-v1'),('rgbd-default','assets/projective-affine-v1/recorded'),('timing','assets/projective-timing-v1/recorded')]:
 for c in cases:
  folder=b/mode/c;old=r/prior/c/'sensor-only.json.gz';assert doc(folder/'sensor-only.json')==json.loads(gzip.decompress(old.read_bytes()))
  freeze=doc(folder/'freeze.json');assert freeze['binary_sha256']==binary and freeze['information_guard']=='physical_spectrum_v1'
  for n,h in freeze['source_sha256'].items():assert sha(candidate[n])==h,n
  copy(folder/'freeze.json',p/'regression'/mode/c/'freeze.json');write(p/'regression'/mode/c/'replay.json',dict(outcomes_identical=180,canonical_ledger=str(old.relative_to(r)),current_ledger_sha256=sha((folder/'sensor-only.json').read_bytes()),summary=doc(folder/'results.json')['summary']))
for label,folder,files in [('plant','heldout',('design.json','source.tar.gz','qualification-failure.json','range-transfer.json')),('rpy','heldout-rpy',('design.json','source.tar.gz','acquisition.json','range-transfer.json','inventory-repair-started.json','environment.json','results.json'))]:
 for n in files:copy(b/folder/n,p/'heldout'/label/n)
copy(r/'data/tum-fr1-rpy-heldout/manifest.json',p/'heldout/rpy/manifest.json')
for n in ('freeze.json','serialization-repair.json','math-check.json','first-source.py','first-heldout-results.partial'):copy(cal/n,p/'calibration'/n)
for n in ('viewed-samples','heldout-results'):copy(cal/(n+'.json'),p/'calibration'/(n+'.json.gz'),True)
for folder,target,keys in [('final-controls','controls',('summary.json','controls.json')),('final-exposure','exposure',('summary.json','controls.json')),('lifecycle','lifecycle',('summary.json','sequence.json'))]:
 for n in keys:copy(b/folder/n,p/target/(n+'.gz' if n!='summary.json' else n),n!='summary.json')
controls=doc(b/'final-controls/summary.json');assert controls['pass_timing_controls'] and controls['accepted']['plane uncorrected']==2 and controls['accurate']['plane compensated']==8 and controls['raw_failure_witnesses']==7
for folder in ('report-write-controls','calibration-math-final','inventory-regression','scorer-regression-final','calibration-helper-regression'):copy(b/folder/'summary.json',p/'checks'/folder/'summary.json')
for f in (b/'analytic').glob('*.json'):copy(f,p/'analytic'/f.name)
copy(b/'analytic-raw/analytic-audit.json',p/'analytic/raw-audit.json')
for name in ('before-source.tar.gz','before-source-sha256.json','candidate-source.tar.gz','candidate-source-sha256.json','final-source.tar.gz','final-source-sha256.json','change.patch','design.json','replay-proof.json','cold-source-replay.json'):copy(b/name,p/name)
outputs=doc(r/'assets/projective-timing-v1/core-output-sha256.json')
for n,h in outputs.items():assert sha((r/n).read_bytes())==h,n
write(p/'core-output-sha256.json',outputs)
for name in ('unit','clippy','build','core-complete','default','rgbd-default','timing-recorded','verified-controls','final-controls','final-exposure','analytic','analytic-raw','lifecycle','heldout-acquire','rpy-acquire','rpy-inventory-repair','rpy-run','rpy-raw','rpy-score','calibration','calibration-evaluate','calibration-repaired','calibration-verified','report-write','calibration-math-current','inventory-regression','scorer-final','calibration-helper-final','cold-source-replay'):copy(Path('/tmp/rustdrive-information-'+name+'.log'),p/'logs'/(name+'.log'))
for name in ('check-rustdrive-heldout-inventory-final.py','check-rustdrive-heldout-scorer-final.py','check-rustdrive-calibration-helper-final.py','replay-rustdrive-rpy-source.py','repair-rustdrive-conditional-json.py','verify-rustdrive-repaired-conditional-json.py','download-rustdrive-plant-ranges.py','download-rustdrive-rpy-ranges.py'):copy(Path('/tmp')/name,p/'checks'/name)
write(p/'summary.json',dict(schema_version=1,feature='Mandatory physical directional-information guard and complete-report publication',binary_sha256=binary,source_members=len(names),weak_planar_acceptances_prevented=6,legacy_outcomes_identical=1080,cold_source_replay_outcomes_identical=360,first_plant_metadata_qualification_pass=False,first_rpy_scores=scores,heldout_geometric_systems=geometry,heldout_image_systems=photo,heldout_model_generations=358,conditional_calibration_pass=False,quality_failure_exit_status=1,goal_30_percent_complete=False,subjective_maturity_percent=20))
write(p/'validation.json',dict(native_tests=45,core_tests=371,scenario_replays=49,core_outputs_unchanged=245,historical_evidence_entries_unchanged=1231,legacy_outcomes_identical=1080,frozen_source_outcomes_identical=360,raw_heldout_without_numeric_truth=True,heldout_normal_systems=geometry+photo,raw_synthetic_failure_witnesses=7,temporal_provenance_mutations=4,information_failure_mutations=3,atomic_report_controls=6,calibration_axes=6,known_scorer_updates=716,conditional_model_frozen_before_heldout_labels=True,conditional_calibration_pass=False,goal_30_percent_complete=False,subjective_maturity_percent=20))
copy(Path(__file__),p/'seal.py')
files={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()};write(p/'files.json',files)
print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()),source_members=len(names),heldout_normal_systems=geometry+photo,rust_tests=416)))
