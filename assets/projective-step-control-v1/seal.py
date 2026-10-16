from pathlib import Path
import json,gzip,hashlib,tarfile,re,difflib
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-step-control';p=r/'assets/projective-step-control-v1';cases=('tum-fr1-room-temporal','tum-fr1-desk2-independent')
def sha(x):return hashlib.sha256(x).hexdigest()
def doc(x):return json.loads(x.read_text())
def write(path,value):
 path.parent.mkdir(parents=True,exist_ok=True)
 with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def copy(src,dst,compressed=False):
 dst.parent.mkdir(parents=True,exist_ok=True)
 with dst.open('xb') as f:f.write(gzip.compress(src.read_bytes(),mtime=0) if compressed else src.read_bytes())
assert not p.exists()
raw=doc(b/'raw/summary.json');assert raw['pass_raw_mathematics'] and not raw['ground_truth_decoded']
gate=doc(b/'adoption-gate.json');assert gate['implementation_integrity_pass'] and not gate['default_adoption_pass'];assert not any(c['default_adoption_pass'] for c in gate['cases'].values())
for label,count in [('unit-explicit',39),('core-final',371)]:
 log=Path('/tmp/rustdrive-step-'+label+'.log').read_text();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',log)))==count and 'FAILED' not in log
 if label=='core-final':assert log.count('"verified": true')==49
entries=0
for m in (r/'assets').glob('*/files.json'):
 for n,v in doc(m).items():
  x=(m.parent/n).read_bytes();assert sha(x)==v['sha256'] and len(x)==v['bytes'];entries+=1
assert entries==1015
names=set(doc(b/'before-source-sha256.json'))|{'integrations/projective-depth/src/step_control.rs','scripts/projective_step_raw.py','scripts/check-projective-step-control.py','docs/projective-step-control.md'}
for c in cases:
 names.update(doc(b/'maintained'/c/'freeze.json')['source_sha256']);names.update(doc(b/'raw'/c/'freeze.json')['source_sha256'])
pins={n:sha((r/n).read_bytes()) for n in sorted(names)}
for n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):assert pins[n]==doc(b/'before-source-sha256.json')[n]
with tarfile.open(b/'final-source.tar.gz','w:gz') as t:
 for n in sorted(names):t.add(r/n,arcname=n)
(b/'final-source-sha256.json').write_text(json.dumps(pins,indent=2)+'\n')
with tarfile.open(b/'before-source.tar.gz') as t:before={m.name:t.extractfile(m).read().decode() for m in t.getmembers() if m.isfile()}
changes=[]
for n in sorted(names):
 if n in before or n in ('integrations/projective-depth/src/step_control.rs','scripts/projective_step_raw.py','scripts/check-projective-step-control.py','docs/projective-step-control.md'):
  changes.extend(difflib.unified_diff(before.get(n,'').splitlines(True),(r/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
(b/'change.patch').write_text(''.join(changes))
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes());scores={};geometry=photo=reports=trials=reduced=halts=0
for c,expected in zip(cases,(83,67)):
 folder=b/'maintained'/c;freeze=doc(folder/'freeze.json');s=doc(folder/'results.json')['summary'];scores[c]=s
 assert freeze['binary_sha256']==binary and freeze['step_control']=='fixed_support' and s['accepted']==179 and s['root_accurate']==expected and not s['lost']
 for n,h in freeze['source_sha256'].items():assert pins[n]==h,(c,n)
 af=doc(b/'raw'/c/'freeze.json');assert af['step_control']=='fixed_support'
 for n,h in af['source_sha256'].items():assert pins[n]==h,(c,n)
 ledger=doc(folder/'sensor-only.json');assert not ledger['ground_truth_parsed'] and ledger==doc(b/'recorded'/c/'sensor-only.json')
 assert af['original_sensor_ledger_sha256']==sha((folder/'sensor-only.json').read_bytes()) and af['original_freeze_sha256']==sha((folder/'freeze.json').read_bytes())
 copy(folder/'freeze.json',p/'recorded'/c/'freeze.json')
 for n in ('sensor-only','results'):copy(folder/(n+'.json'),p/'recorded'/c/(n+'.json.gz'),True)
 copy(b/'recorded'/c/'freeze.json',p/'first'/c/'freeze.json')
 for n in ('sensor-only','results'):copy(b/'recorded'/c/(n+'.json'),p/'first'/c/(n+'.json.gz'),True)
 for n in ('freeze.json','audit.json'):copy(b/'raw'/c/n,p/'raw'/c/n)
 a=doc(b/'raw'/c/'audit.json')['summary'];geometry+=a['raw_iteration_equations']+a['returned_pose_equations'];photo+=a['raw_photometric_equations'];reports+=a['step_controls'];trials+=a['step_trials'];reduced+=a['reduced_steps'];halts+=a['halted_levels']
 for name,folder,oldpath in [('depth','maintained-default',r/'assets/projective-depth-rust-v1'/c/'sensor-only.json.gz'),('rgbd','maintained-rgbd-default',r/'assets/projective-affine-v1/recorded'/c/'sensor-only.json.gz')]:
  current=b/folder/c;assert doc(current/'sensor-only.json')==json.loads(gzip.decompress(oldpath.read_bytes()))
  assert doc(current/'freeze.json')['binary_sha256']==binary and doc(current/'freeze.json')['step_control']=='disabled'
  copy(current/'freeze.json',p/'defaults'/name/c/'freeze.json')
  write(p/'defaults'/name/c/'replay.json',dict(outcomes_identical=180,canonical_ledger=str(oldpath.relative_to(r)),current_ledger_sha256=sha((current/'sensor-only.json').read_bytes()),summary=doc(current/'results.json')['summary']))
assert geometry==7978 and reports==7620 and trials==8901 and reduced==869 and halts==0
analytic=doc(b/'final-analytic/summary.json');assert analytic['binary_sha256']==binary and analytic['step_control']=='fixed_support' and len(analytic['rejected_transport_cases'])==29
assert len(doc(b/'raw/analytic-audit.json')['rejected_corruptions'])==13
for folder,key in [('final-boundary','pass_step_control'),('final-exposure','pass_exposure_controls'),('final-lifecycle','pass_lifecycle')]:
 s=doc(b/folder/'summary.json');assert s[key] and s['binary_sha256']==binary
 assert s['checker_sha256']==pins[{'final-boundary':'scripts/check-projective-step-control.py','final-exposure':'scripts/check-projective-rgbd-exposure.py','final-lifecycle':'scripts/check-projective-rgbd-lifecycle.py'}[folder]]
 for f in (b/folder).glob('*.json'):copy(f,p/({'final-boundary':'boundary','final-exposure':'exposure','final-lifecycle':'lifecycle'}[folder])/(f.name+'.gz' if f.name!='summary.json' else f.name),f.name!='summary.json')
for n in ('before-source.tar.gz','before-source-sha256.json','first-source.tar.gz','first-source-sha256.json','final-source.tar.gz','final-source-sha256.json','change.patch','design.json','adoption-correction.json','replay-proof.json'):copy(b/n,p/n)
for folder,target in [('final-analytic','analytic'),('depth-analytic','depth-analytic'),('rgbd-analytic','rgbd-analytic')]:
 for f in (b/folder).glob('*.json'):copy(f,p/target/f.name)
for n in ('summary.json','analytic-audit.json'):copy(b/'raw'/n,p/'raw'/n)
copy(b/'adoption-gate.json',p/'adoption-gate.json');copy(b/'supervision/summary.json',p/'supervision.json')
outputs=doc(r/'assets/projective-affine-v1/core-output-sha256.json');assert len(outputs)==245
for n,h in outputs.items():assert sha((r/n).read_bytes())==h
write(p/'core-output-sha256.json',outputs)
for name,label in {'core-check':'core-final','native-unit':'unit-explicit','native-clippy':'clippy-explicit','native-build':'build-explicit','first-recorded':'recorded','maintained-recorded':'maintained','depth-default':'maintained-default','rgbd-default':'maintained-rgbd-default','raw':'raw','boundary':'final-boundary','lifecycle':'final-lifecycle','exposure':'final-exposure'}.items():copy(Path('/tmp/rustdrive-step-'+label+'.log'),p/'logs'/(name+'.log'))
write(p/'summary.json',dict(schema_version=1,feature='Explicit fixed-support nonlinear RGB-D step control; not adopted',source_members=len(names),binary_sha256=binary,scores=scores,geometry_systems=geometry,image_systems=photo,step_reports=reports,trial_costs=trials,initial_costs=reports,reduced_steps=reduced,halted_levels=halts,depth_default_outcomes_identical=360,rgbd_default_outcomes_identical=360,guarded_outcomes_identical_to_first=360,implementation_integrity_pass=True,default_adoption_pass=False,ground_truth_operational=False,viewed_development=True,subjective_maturity_percent=20))
write(p/'validation.json',dict(native_tests=39,core_tests=371,scenario_replays=49,core_outputs_unchanged=245,earlier_evidence_entries_unchanged=1015,geometry_systems=geometry,image_systems=photo,step_reports=reports,raw_trial_costs=trials,model_generations=358,raw_without_numeric_truth=True,raw_mutations_rejected=13,boundary_forgery_rejected=True,transport_faults_rejected=29,both_defaults_unchanged=True,default_adoption_pass=False,subjective_maturity_percent=20))
copy(Path(__file__),p/'seal.py')
files={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()};write(p/'files.json',files)
print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()),source_members=len(names),geometry_systems=geometry,image_systems=photo,raw_costs=reports+trials)))
