from pathlib import Path
import json,gzip,hashlib,tarfile,difflib,re
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-motion';p=r/'assets/projective-motion-v1';cases=('tum-fr1-room-temporal','tum-fr1-desk2-independent')
def sha(x):return hashlib.sha256(x).hexdigest()
def doc(x):return json.loads(x.read_text())
def write(path,value):
 path.parent.mkdir(parents=True,exist_ok=True)
 with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def copy(src,dst,zip=False):
 dst.parent.mkdir(parents=True,exist_ok=True)
 with dst.open('xb') as f:f.write(gzip.compress(src.read_bytes(),mtime=0) if zip else src.read_bytes())
assert not p.exists()
for mode in ('depth','rgbd'):
 assert doc(b/('raw-'+mode)/'summary.json')['pass_raw_mathematics']
 gate=doc(b/(mode+'-adoption.json'));assert gate['implementation_integrity_pass'] and not gate['default_adoption_pass']
for label,n in [('unit-final',41),('core-final',371)]:
 log=Path('/tmp/rustdrive-motion-'+label+'.log').read_text();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',log)))==n and 'FAILED' not in log
 if label=='core-final':assert log.count('"verified": true')==49
entries=0
for m in (r/'assets').glob('*/files.json'):
 for n,v in doc(m).items():
  body=(m.parent/n).read_bytes();assert len(body)==v['bytes'] and sha(body)==v['sha256'];entries+=1
assert entries==1090
names=set(doc(b/'before-source-sha256.json'))|{'integrations/projective-depth/src/motion.rs','scripts/projective_motion_raw.py','scripts/check-projective-motion.py','scripts/diagnose-projective-motion-timing.py','docs/projective-motion.md'}
for mode in ('depth','rgbd'):
 for c in cases:names.update(doc(b/('final-'+mode)/c/'freeze.json')['source_sha256']);names.update(doc(b/('raw-'+mode)/c/'freeze.json')['source_sha256'])
pins={n:sha((r/n).read_bytes()) for n in sorted(names)}
for n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):assert pins[n]==doc(b/'before-source-sha256.json')[n]
with tarfile.open(b/'final-source.tar.gz','w:gz') as t:
 for n in sorted(names):t.add(r/n,arcname=n)
(b/'final-source-sha256.json').write_text(json.dumps(pins,indent=2)+'\n')
with tarfile.open(b/'before-source.tar.gz') as t:before={m.name:t.extractfile(m).read().decode() for m in t.getmembers() if m.isfile()}
changes=[]
for n in sorted(names):changes.extend(difflib.unified_diff(before.get(n,'').splitlines(True),(r/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
(b/'change.patch').write_text(''.join(changes))
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes());scores={};geometry=photo=predictions=generations=0
for mode,expected in [('depth',(67,159)),('rgbd',(75,169))]:
 scores[mode]={}
 for c,count in zip(cases,expected):
  folder=b/('final-'+mode)/c;freeze=doc(folder/'freeze.json');report=doc(folder/'results.json');ledger=doc(folder/'sensor-only.json')
  assert freeze['binary_sha256']==binary and freeze['motion_model']=='constant_velocity' and freeze['step_control']=='disabled' and not freeze['ground_truth_operational'] and freeze['pose_resets']==0
  assert not ledger['ground_truth_parsed'] and report['summary']['accepted']==179 and report['summary']['root_accurate']==count and not report['summary']['lost'];scores[mode][c]=report['summary']
  for n,h in freeze['source_sha256'].items():assert pins[n]==h,(mode,c,n)
  af=doc(b/('raw-'+mode)/c/'freeze.json');a=doc(b/('raw-'+mode)/c/'audit.json');assert a['pass_raw_mathematics'] and not af['ground_truth_decoded'] and af['motion_model']=='constant_velocity'
  assert af['original_freeze_sha256']==sha((folder/'freeze.json').read_bytes()) and af['original_sensor_ledger_sha256']==sha((folder/'sensor-only.json').read_bytes())
  for n,h in af['source_sha256'].items():assert pins[n]==h,(mode,c,n)
  predictions+=sum('prediction' in row['update'] for row in ledger['frames']);geometry+=a['summary']['raw_iteration_equations']+a['summary']['returned_pose_equations'];photo+=a['summary'].get('raw_photometric_equations',0);generations+=a['summary']['fused_model_generations']
  copy(folder/'freeze.json',p/'recorded'/mode/c/'freeze.json')
  for n in ('sensor-only','results'):copy(folder/(n+'.json'),p/'recorded'/mode/c/(n+'.json.gz'),True)
  for n in ('freeze.json','audit.json'):copy(b/('raw-'+mode)/c/n,p/'raw'/mode/c/n)
 for n in ('summary.json','analytic-audit.json'):copy(b/('raw-'+mode)/n,p/'raw'/mode/n)
 copy(b/(mode+'-adoption.json'),p/(mode+'-adoption.json'))
assert (geometry,photo,predictions,generations)==(14908,7408,716,716)
for mode,folder in [('depth','default'),('rgbd','rgbd-default')]:
 for c in cases:
  current=b/folder/c;old=r/('assets/projective-depth-rust-v1' if mode=='depth' else 'assets/projective-affine-v1/recorded')/c/'sensor-only.json.gz'
  assert doc(current/'sensor-only.json')==json.loads(gzip.decompress(old.read_bytes()))
  assert doc(current/'freeze.json')['motion_model']=='stationary' and doc(current/'freeze.json')['binary_sha256']==binary
  copy(current/'freeze.json',p/'defaults'/mode/c/'freeze.json');write(p/'defaults'/mode/c/'replay.json',dict(outcomes_identical=180,canonical_ledger=str(old.relative_to(r)),ledger_sha256=sha((current/'sensor-only.json').read_bytes()),summary=doc(current/'results.json')['summary']))
with tarfile.open(b/'first-source.tar.gz') as t:first={m.name:t.extractfile(m).read() for m in t.getmembers() if m.isfile()}
for mode in ('depth','rgbd'):
 folder=b/mode/cases[0];f=doc(folder/'freeze.json');assert f['binary_sha256']==sha((b/'first-binary').read_bytes())
 for n,h in f['source_sha256'].items():assert sha(first[n])==h,(mode,n)
 copy(folder/'freeze.json',p/'first'/mode/'freeze.json');copy(folder/'sensor-only.json',p/'first'/mode/'sensor-only.json.gz',True)
 copy(Path('/tmp/rustdrive-motion-'+mode+'.log'),p/'first'/mode/'numerical-audit-failure.log')
for n in ('before-source.tar.gz','before-source-sha256.json','first-source.tar.gz','first-source-sha256.json','final-source.tar.gz','final-source-sha256.json','change.patch','design.json','replay-proof.json'):copy(b/n,p/n)
for folder,target in [('final-analytic','analytic-depth'),('rgbd-analytic','analytic-rgbd')]:
 s=doc(b/folder/'summary.json');assert s['analytic_controls_pass'] and s['binary_sha256']==binary and len(s['rejected_transport_cases'])==30
 for f in (b/folder).glob('*.json'):copy(f,p/target/f.name)
life=doc(b/'final-lifecycle/summary.json');assert life['pass_motion_controls'] and life['binary_sha256']==binary and life['checker_sha256']==pins['scripts/check-projective-motion.py'] and len(life['rejected_corruptions'])==4
copy(b/'final-lifecycle/summary.json',p/'lifecycle/summary.json');copy(b/'final-lifecycle/sequence.json',p/'lifecycle/sequence.json.gz',True)
timing=doc(b/'timing-final/summary.json');assert not timing['numeric_truth_decoded']
for n,h in timing['source_sha256'].items():assert pins[n]==h,n
for f in (b/'timing-final').glob('*.json'):copy(f,p/'timing'/f.name)
outputs=doc(r/'assets/projective-step-control-v1/core-output-sha256.json');assert len(outputs)==245
for n,h in outputs.items():assert sha((r/n).read_bytes())==h,n
write(p/'core-output-sha256.json',outputs)
for name,label in {'core-check':'core-final','native-unit':'unit-final','native-clippy':'clippy-final','native-build':'build-final','recorded-depth':'final-depth','recorded-rgbd':'final-rgbd','depth-default':'default','rgbd-default':'rgbd-default','raw-depth':'raw-depth','raw-rgbd':'raw-rgbd','lifecycle':'final-lifecycle','timing':'timing-final'}.items():copy(Path('/tmp/rustdrive-motion-'+label+'.log'),p/'logs'/(name+'.log'))
write(p/'summary.json',dict(schema_version=1,feature='Causal acquisition-time SE3 initializer and timing displacement diagnostic',binary_sha256=binary,source_members=len(names),scores=scores,geometric_systems=geometry,image_systems=photo,prediction_reports=predictions,model_generations=generations,depth_default_outcomes_identical=360,rgbd_default_outcomes_identical=360,default_adoption_pass=False,temporal_image_compensation_implemented=False,viewed_development=True,ground_truth_operational=False,subjective_maturity_percent=20))
write(p/'validation.json',dict(native_tests=41,core_tests=371,scenario_replays=49,core_outputs_identical=245,historical_evidence_entries_unchanged=1090,geometric_systems=geometry,image_systems=photo,prediction_reports=predictions,model_generations=generations,raw_without_numeric_truth=True,causal_fault_controls_pass=True,prediction_mutations_rejected=4,transport_rejections=30,both_defaults_unchanged=True,default_adoption_pass=False,subjective_maturity_percent=20))
copy(Path(__file__),p/'seal.py')
files={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()};write(p/'files.json',files)
print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()),source_members=len(names),normal_systems=geometry+photo,prediction_reports=predictions)))
