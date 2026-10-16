from pathlib import Path
import json,gzip,hashlib,tarfile,difflib,re
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-timing';p=r/'assets/projective-timing-v1';cases=('tum-fr1-room-temporal','tum-fr1-desk2-independent')
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
assert doc(b/'final-raw/summary.json')['pass_raw_mathematics']
gate=doc(b/'adoption.json');assert gate['implementation_integrity_pass'] and not gate['default_adoption_pass']
for label,n in [('unit',43),('core',371)]:
 log=Path('/tmp/rustdrive-timing-'+label+'.log').read_text();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',log)))==n and 'FAILED' not in log
 if label=='core':assert log.count('"verified": true')==49
entries=0
for m in (r/'assets').glob('*/files.json'):
 for n,v in doc(m).items():
  body=(m.parent/n).read_bytes();assert len(body)==v['bytes'] and sha(body)==v['sha256'];entries+=1
assert entries==1170
before=archive(b/'before-source.tar.gz');candidate=archive(b/'candidate-source.tar.gz')
for label,contents in [('before',before),('candidate',candidate)]:
 pins=doc(b/(label+'-source-sha256.json'));assert set(pins)==set(contents)
 for n,h in pins.items():assert sha(contents[n])==h
names=set(candidate)|{'docs/projective-timing.md'}
for c in cases:
 names.update(doc(b/'recorded'/c/'freeze.json')['source_sha256']);names.update(doc(b/'final-raw'/c/'freeze.json')['source_sha256'])
pins={n:sha((r/n).read_bytes()) for n in sorted(names)}
for n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):assert pins[n]==sha(before[n])
for n in pins:
 if n.endswith('.rs'):assert pins[n]==sha(candidate[n]),'runtime unchanged after candidate source freeze'
with tarfile.open(b/'final-source.tar.gz','w:gz') as t:
 for n in sorted(names):t.add(r/n,arcname=n)
write(b/'final-source-sha256.json',pins)
changes=[]
for n in sorted(names):changes.extend(difflib.unified_diff(before.get(n,b'').decode().splitlines(True),(r/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
(b/'change.patch').write_text(''.join(changes))
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes());scores={};geometry=photo=generations=predictions=0
for c,count in zip(cases,(80,171)):
 folder=b/'recorded'/c;freeze=doc(folder/'freeze.json');report=doc(folder/'results.json');ledger=doc(folder/'sensor-only.json')
 assert freeze['binary_sha256']==binary and freeze['timing']=='previous_motion' and freeze['motion_model']=='constant_velocity' and freeze['step_control']=='disabled' and not freeze['ground_truth_operational'] and freeze['pose_resets']==0
 assert not ledger['ground_truth_parsed'] and report['summary']['accepted']==179 and report['summary']['root_accurate']==count and not report['summary']['lost'];scores[c]=report['summary']
 assert [x['update'] for x in ledger['frames']]==[x['update'] for x in report['frames']]
 for n,h in freeze['source_sha256'].items():assert sha(candidate[n])==h,('candidate',n)
 af=doc(b/'final-raw'/c/'freeze.json');a=doc(b/'final-raw'/c/'audit.json');assert a['pass_raw_mathematics'] and not af['ground_truth_decoded'] and af['timing']=='previous_motion'
 assert af['original_freeze_sha256']==sha((folder/'freeze.json').read_bytes()) and af['original_sensor_ledger_sha256']==sha((folder/'sensor-only.json').read_bytes())
 for n,h in af['source_sha256'].items():assert pins[n]==h,('audit',n)
 predictions+=sum('prediction' in row['update'] for row in ledger['frames']);geometry+=a['summary']['raw_iteration_equations']+a['summary']['returned_pose_equations'];photo+=a['summary']['raw_photometric_equations'];generations+=a['summary']['fused_model_generations']
 copy(folder/'freeze.json',p/'recorded'/c/'freeze.json')
 for n in ('sensor-only','results'):copy(folder/(n+'.json'),p/'recorded'/c/(n+'.json.gz'),True)
 for n in ('freeze.json','audit.json'):copy(b/'final-raw'/c/n,p/'raw'/c/n)
assert geometry==7969 and predictions==generations==358
for n in ('summary.json','analytic-audit.json'):copy(b/'final-raw'/n,p/'raw'/n)
for mode,oldfolder in [('default','assets/projective-depth-rust-v1'),('rgbd-default','assets/projective-affine-v1/recorded')]:
 for c in cases:
  current=b/mode/c;old=r/oldfolder/c/'sensor-only.json.gz'
  assert doc(current/'sensor-only.json')==json.loads(gzip.decompress(old.read_bytes()))
  assert doc(current/'freeze.json')['timing']=='disabled' and doc(current/'freeze.json')['motion_model']=='stationary' and doc(current/'freeze.json')['binary_sha256']==binary
  copy(current/'freeze.json',p/'defaults'/mode/c/'freeze.json');write(p/'defaults'/mode/c/'replay.json',dict(outcomes_identical=180,canonical_ledger=str(old.relative_to(r)),ledger_sha256=sha((current/'sensor-only.json').read_bytes()),summary=doc(current/'results.json')['summary']))
failed=b/'raw'/cases[0]
for n in ('freeze.json','first-failure.json'):copy(failed/n,p/'first-raw'/n)
for n,h in doc(failed/'freeze.json')['source_sha256'].items():assert sha(candidate[n])==h,('initial-audit',n)
copy(Path('/tmp/rustdrive-timing-raw.log'),p/'first-raw/failure.log')
copy(Path('/tmp/diagnose-rustdrive-timing-audit.py'),p/'rounding/diagnose.py')
copy(Path('/tmp/rustdrive-timing-rounding-diagnostic.log'),p/'rounding/diagnostic.log')
rounding=json.loads(Path('/tmp/rustdrive-timing-rounding-diagnostic.log').read_text());assert rounding['original_failure_frame_reconstructed'] and rounding['max_independent_warp_matrix_difference']<4e-16
write(p/'rounding/summary.json',rounding)
for n in ('before-source.tar.gz','before-source-sha256.json','candidate-source.tar.gz','candidate-source-sha256.json','final-source.tar.gz','final-source-sha256.json','change.patch','design.json','replay-proof.json','adoption.json'):copy(b/n,p/n)
for folder,field in [('verified-controls','pass_timing_controls'),('verified-lifecycle','pass_lifecycle')]:
 summary=doc(b/folder/'summary.json');assert summary[field] and summary['binary_sha256']==binary
 checker='scripts/check-projective-timing.py' if folder=='verified-controls' else 'scripts/check-projective-rgbd-lifecycle.py';assert summary['checker_sha256']==pins[checker]
 target='controls' if folder=='verified-controls' else 'lifecycle';copy(b/folder/'summary.json',p/target/'summary.json')
 name='controls.json' if folder=='verified-controls' else 'sequence.json';copy(b/folder/name,p/target/(name+'.gz'),True)
control=doc(b/'verified-controls/summary.json');assert control['accurate']['plane compensated']==8 and control['accurate']['plane uncorrected']==2 and len(control['rejected_corruptions'])==4
s=doc(b/'analytic/summary.json');assert s['analytic_controls_pass'] and s['binary_sha256']==binary and len(s['rejected_transport_cases'])==34
for f in (b/'analytic').glob('*.json'):copy(f,p/'analytic'/f.name)
outputs=doc(r/'assets/projective-motion-v1/core-output-sha256.json');assert len(outputs)==245
for n,h in outputs.items():assert sha((r/n).read_bytes())==h,n
write(p/'core-output-sha256.json',outputs)
for name in ('core','unit','clippy','build','recorded','default','rgbd-default','final-raw','verified-controls','verified-lifecycle','analytic','analytic-raw'):
 copy(Path('/tmp/rustdrive-timing-'+name+'.log'),p/'logs'/(name+'.log'))
write(p/'summary.json',dict(schema_version=1,feature='Causal own-acquisition-time RGB/depth image compensation',binary_sha256=binary,source_members=len(names),scores=scores,geometric_systems=geometry,image_systems=photo,prediction_reports=predictions,model_generations=generations,temporal_image_compensation_implemented=True,legacy_outcomes_identical=720,default_adoption_pass=False,own_time_plane_accurate={'compensated':8,'uncorrected':2,'frames':8},viewed_development=True,ground_truth_operational=False,subjective_maturity_percent=20))
write(p/'validation.json',dict(native_tests=43,core_tests=371,scenario_replays=49,core_outputs_identical=245,historical_evidence_entries_unchanged=1170,geometric_systems=geometry,image_systems=photo,prediction_reports=predictions,model_generations=generations,raw_without_numeric_truth=True,temporal_provenance_mutations_rejected=4,transport_rejections=34,legacy_outcomes_identical=720,independent_warp_tolerance=1e-12,normal_equation_tolerances_unchanged=True,default_adoption_pass=False,subjective_maturity_percent=20))
copy(Path(__file__),p/'seal.py')
files={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()};write(p/'files.json',files)
print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()),source_members=len(names),normal_systems=geometry+photo,prediction_reports=predictions)))
