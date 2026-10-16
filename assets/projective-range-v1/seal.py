from pathlib import Path
import difflib,gzip,hashlib,json,re,tarfile
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-range-v1';p=r/'assets/projective-range-v1'
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
historical=0
for m in (r/'assets').glob('*/files.json'):
 for n,v in doc(m).items():
  body=(m.parent/n).read_bytes();assert len(body)==v['bytes'] and sha(body)==v['sha256'],(m,n);historical+=1
assert historical==1466
epochs={}
for label in ('before','variance','whitened'):
 data=archive(b/(label+'-source.tar.gz'));pins=doc(b/(label+'-source-sha256.json'));assert set(data)==set(pins)
 for n,h in pins.items():assert sha(data[n])==h,n
 epochs[label]=data
diagnostic=archive(b/'diagnostic-source.tar.gz');diagnostic_pins=doc(b/'diagnosis/freeze.json')['source_sha256']
assert set(diagnostic)==set(diagnostic_pins)
for n,h in diagnostic_pins.items():assert sha(diagnostic[n])==h,n
names=set(epochs['whitened'])|{'docs/projective-range.md','artifacts/projective-range-v1/replay-defaults.py'}
epochs['final']={n:(r/n).read_bytes() for n in sorted(names)}
for n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):
 assert all(data[n]==epochs['before'][n] for data in epochs.values())
for n,body in epochs['final'].items():
 if n.endswith('.rs'):assert body==epochs['whitened'][n],n
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes())
for log,expected in [('complete-core',371),('whitened-unit',53)]:
 body=Path('/tmp/rustdrive-range-'+log+'.log').read_text()
 assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',body)))==expected and 'FAILED' not in body
 if log=='complete-core':assert body.count('"verified": true')==49
core=doc(r/'assets/projective-surface-v1/core-output-sha256.json')
assert len(core)==245 and all(sha((r/n).read_bytes())==h for n,h in core.items())
compat={mode:doc(b/('compatibility-'+mode)/'summary.json') for mode in ('depth','rgbd','timing')}
assert all(x['pass_exact_compatibility'] and not x['numeric_reference_poses_decoded'] for x in compat.values())
assert sum(x['identical_outcomes'] for x in compat.values())==1440
for mode in compat:
 f=doc(b/('compatibility-'+mode)/'freeze.json');assert f['binary_sha256']==binary
 for n,h in f['source_sha256'].items():assert sha(epochs['final'][n])==h,n
scores={};systems=0
for mode,folder,counts in [('variance','recorded',[79,74,115]),('whitened','whitened-recorded',[77,75,107])]:
 scores[mode]={}
 for c,label,expected in zip(cases,('room','desk2','rpy'),counts,strict=True):
  rawfolder=b/'recorded-raw'/c if mode=='variance' else b/('whitened-raw-'+label)/c
  freeze=doc(b/folder/c/'freeze.json');ledger=doc(b/folder/c/'sensor-only.json');result=doc(b/folder/c/'results.json');raw=doc(rawfolder/'audit.json');rawfreeze=doc(rawfolder/'freeze.json')
  assert not ledger['ground_truth_parsed'] and len(ledger['frames'])==180
  assert [x['update'] for x in ledger['frames']]==[x['update'] for x in result['frames']]
  s=result['summary'];assert s['root_accurate']==expected and s['accepted']==179 and not s['lost'] and not s['complete_accuracy_pass']
  assert raw['pass_raw_mathematics'] and not raw['summary']['ground_truth_decoded']
  assert raw['summary']['frames']==180 and raw['summary']['returned_pose_equations']==179 and raw['summary']['raw_iteration_equations']==s['independently_checked_normal_steps']
  assert rawfreeze['original_sensor_ledger_sha256']==sha((b/folder/c/'sensor-only.json').read_bytes())
  for f in (freeze,rawfreeze):
   for n,h in f['source_sha256'].items():assert sha(epochs[mode][n])==h,(mode,n)
  assert freeze['binary_sha256']==sha((b/(mode+'-binary')).read_bytes())
  scores[mode][c]=s;systems+=raw['summary']['raw_iteration_equations']+179
assert systems==19838
adoption=doc(b/'adoption.json');assert not adoption['default_changed']
for mode in scores:
 assert not adoption['modes'][mode]['pass_default_adoption']
 for c,threshold in zip(cases,(65,161,74),strict=True):
  pin=adoption['baseline'][c];path=r/pin['file'];assert sha(path.read_bytes())==pin['sha256']
  base=json.loads(gzip.decompress(path.read_bytes()))['summary'];s=scores[mode][c]
  flags=dict(accuracy_count_pass=s['root_accurate']>=threshold,maximum_position_pass=s['max_position_error_m']<=base['max_position_error_m'],maximum_rotation_pass=s['max_rotation_error_rad']<=base['max_rotation_error_rad'],tracking_pass=s['accepted']==179 and not s['lost'])
  assert flags==adoption['modes'][mode]['checks'][c]
assert doc(b/'design.json')['adoption']==doc(b/'whitened-design.json')['adoption']
assert doc(b/'epoch-compatibility.json')['synthetic_outcomes_identical']==28
for folder,epoch in [('range-controls','variance'),('whitened-controls','whitened'),('final-variance-controls','final')]:
 assert doc(b/folder/'summary.json')['pass_range_controls']
 f=doc(b/folder/'freeze.json')
 for n,h in f['source_sha256'].items():assert sha(epochs[epoch][n])==h,(folder,n)
assert doc(b/'diagnostic-decision.json')['all_four_bins_monotone_on_all_three_viewed_records']
assert not doc(b/'diagnosis/summary.json')['measurement_noise_calibration_established']
finalpins={n:sha(body) for n,body in epochs['final'].items()}
write(b/'final-source-sha256.json',finalpins)
with tarfile.open(b/'final-source.tar.gz','x:gz') as t:
 for n in sorted(finalpins):t.add(r/n,arcname=n,recursive=False)
patch=[]
for n,body in epochs['final'].items():patch.extend(difflib.unified_diff(epochs['before'].get(n,b'').decode().splitlines(True),body.decode().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
with (b/'change.patch').open('x') as f:f.write(''.join(patch))
for name in ('before-source.tar.gz','before-source-sha256.json','diagnostic-source.tar.gz','variance-source.tar.gz','variance-source-sha256.json','whitened-source.tar.gz','whitened-source-sha256.json','final-source.tar.gz','final-source-sha256.json','change.patch','design.json','whitened-design.json','diagnostic-decision.json','adoption.json','epoch-compatibility.json'):
 copy(b/name,p/name)
for mode,folder in [('variance','recorded'),('whitened','whitened-recorded')]:
 for c,label in zip(cases,('room','desk2','rpy'),strict=True):
  out=p/'recorded'/mode/c;copy(b/folder/c/'freeze.json',out/'freeze.json')
  for n in ('sensor-only','results'):copy(b/folder/c/(n+'.json'),out/(n+'.json.gz'),True)
  rawfolder=b/'recorded-raw'/c if mode=='variance' else b/('whitened-raw-'+label)/c
  for n in ('freeze.json','audit.json'):copy(rawfolder/n,out/('raw-'+n))
for folder in ('diagnosis','analytic','analytic-raw','range-controls','whitened-analytic','whitened-analytic-raw','whitened-controls','final-variance-analytic','final-variance-raw','final-variance-controls','compatibility-depth','compatibility-rgbd','compatibility-timing'):
 for src in sorted((b/folder).rglob('*')):
  if src.is_file():
   target=p/'checks'/folder/src.relative_to(b/folder);compress=src.stat().st_size>100000 and src.suffix=='.json'
   if compress:target=target.with_name(target.name+'.gz')
   copy(src,target,compress)
copy(b/'replay-defaults.py',p/'checks/replay-defaults.py')
for log in sorted(Path('/tmp').glob('rustdrive-range-*.log')):copy(log,p/'logs'/log.name)
copy(Path(__file__),p/'seal.py');write(p/'core-output-sha256.json',core)
validation=dict(native_tests=53,core_tests=371,total_rust_tests=424,scenario_replays=49,core_outputs_unchanged=245,compatibility_outcomes_identical=1440,
 raw_recorded_normal_systems=systems,raw_recorded_model_generations=1074,independently_checked_weighted_plane_failures=2,
 heldout_accuracy_claim=False,complete_physical_accuracy=False,default_adoption=False,prior_evidence_entries_unchanged=historical)
write(p/'validation.json',validation)
summary=dict(schema_version=1,implemented='optional depth_variance_plane and depth_whitened_plane',default='target_plane unchanged',
 before_binary_sha256=sha((b/'before-binary').read_bytes()),variance_binary_sha256=sha((b/'variance-binary').read_bytes()),binary_sha256=binary,
 source_members=len(finalpins),recorded_scores=scores,all_recorded_protocols_pass=False,default_adoption=False,
 data_role='three already viewed FR1 regressions, no fresh heldout claim',
 uncertainty='relative scalar axial variance hypothesis, not measured/calibrated sensor or fused-history uncertainty',
 subjective_maturity_percent=20,goal_30_percent_complete=False,validation=validation)
write(p/'summary.json',summary)
inventory={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()}
write(p/'files.json',inventory)
print(json.dumps(dict(packet_files=len(inventory),packet_bytes=sum(x['bytes'] for x in inventory.values()),source_members=len(finalpins),raw_systems=systems,previous_entries=historical,default_adoption=False)),flush=True)
