from pathlib import Path
import difflib,gzip,hashlib,json,re,tarfile
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-midpoint-v1';p=r/'assets/projective-surface-v1'
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
assert historical==1338
epochs={}
for label in ('before','midpoint'):
 data=archive(b/(label+'-source.tar.gz'));pins=doc(b/(label+'-source-sha256.json'))
 assert set(data)==set(pins)
 for n,h in pins.items():assert sha(data[n])==h,n
 epochs[label]=data
names=set(epochs['midpoint'])|{'docs/projective-surface.md','scripts/check-projective-surface-math.py','artifacts/projective-midpoint-v1/replay-defaults.py'}
for folder in ('recorded','bidirectional-recorded','recorded-raw','bidirectional-recorded-raw'):
 for c in cases:names.update(doc(b/folder/c/'freeze.json')['source_sha256'])
epochs['final']={n:(r/n).read_bytes() for n in sorted(names)}
for n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):
 assert epochs['before'][n]==epochs['midpoint'][n]==epochs['final'][n]
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes())
assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',Path('/tmp/rustdrive-midpoint-final-core.log').read_text())))==371
assert Path('/tmp/rustdrive-midpoint-final-core.log').read_text().count('"verified": true')==49
assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',Path('/tmp/rustdrive-midpoint-bidirectional-unit.log').read_text())))==49
core=doc(r/'assets/projective-information-v1/core-output-sha256.json')
assert len(core)==245 and all(sha((r/n).read_bytes())==h for n,h in core.items())
compat={mode:doc(b/('compatibility-'+mode)/'summary.json') for mode in ('depth','rgbd','timing')}
assert all(x['pass_exact_compatibility'] and not x['numeric_reference_poses_decoded'] for x in compat.values())
assert sum(x['identical_outcomes'] for x in compat.values())==1440
for mode in compat:
 f=doc(b/('compatibility-'+mode)/'freeze.json');assert f['binary_sha256']==binary
 for n,h in f['source_sha256'].items():assert sha(epochs['final'][n])==h,n
scores={};systems=0
for mode,folder,epoch,counts in [('midpoint','recorded','midpoint',[66,86,75]),('bidirectional','bidirectional-recorded','final',[64,159,75])]:
 scores[mode]={}
 for c,expected in zip(cases,counts,strict=True):
  freeze=doc(b/folder/c/'freeze.json');ledger=doc(b/folder/c/'sensor-only.json');result=doc(b/folder/c/'results.json');raw=doc(b/(folder+'-raw')/c/'audit.json');raw_freeze=doc(b/(folder+'-raw')/c/'freeze.json')
  assert len(ledger['frames'])==180 and not ledger['ground_truth_parsed']
  assert [x['update'] for x in ledger['frames']]==[x['update'] for x in result['frames']]
  s=result['summary'];assert s['accepted']==179 and s['root_accurate']==expected and not s['lost'] and not s['complete_accuracy_pass']
  assert raw['pass_raw_mathematics'] and not raw['summary']['ground_truth_decoded']
  assert raw['summary']['frames']==180 and raw['summary']['returned_pose_equations']==179
  assert raw['summary']['raw_iteration_equations']==s['independently_checked_normal_steps']
  assert raw_freeze['original_sensor_ledger_sha256']==sha((b/folder/c/'sensor-only.json').read_bytes())
  for f in (freeze,raw_freeze):
   for n,h in f['source_sha256'].items():assert sha(epochs[epoch][n])==h,(epoch,n)
  assert freeze['binary_sha256']==(binary if epoch=='final' else sha((b/'midpoint-binary').read_bytes()))
  scores[mode][c]=s;systems+=raw['summary']['raw_iteration_equations']+179
assert systems==21088
oldmath=(r/'scripts/check-projective-surface-math.py').read_text()
oldmath=oldmath.replace('ROOT, Client, audit_sequence, exp_rotation','ROOT, exp_rotation').replace('normal_equation, pyramid, reconstruct_update','normal_equation, pyramid')
oldmath=oldmath.replace("    parser.add_argument('--binary',type=Path,help='also check native plane rejection and atomic expiry')\n",'')
start=oldmath.index('    planes=[]\n');end=oldmath.index('    sources=(',start);oldmath=oldmath[:start]+oldmath[end:]
oldmath=oldmath.replace("    report=dict(pass_algebraic_symmetry=True,controls=rows,native_plane_controls=planes,\n        binary_sha256=sha(args.binary.read_bytes()) if args.binary else None,numeric_recorded_truth_decoded=False,", "    report=dict(pass_algebraic_symmetry=True,controls=rows,numeric_recorded_truth_decoded=False,")
assert sha(oldmath.encode())==doc(b/'surface-math/summary.json')['source_sha256']['scripts/check-projective-surface-math.py']
control=doc(b/'surface-controls/summary.json');assert control['pass_algebraic_symmetry'] and len(control['native_plane_controls'])==2
assert all(x['raw_failure_witness_checked'] and x['expiry_latched'] for x in control['native_plane_controls'])
assert len(doc(b/'raw-pair-probes/tum-fr1-room-temporal.json')['probes'])==15
finalpins={n:sha(body) for n,body in epochs['final'].items()}
write(b/'final-source-sha256.json',finalpins)
with tarfile.open(b/'final-source.tar.gz','x:gz') as t:
 for n in sorted(finalpins):t.add(r/n,arcname=n,recursive=False)
patch=[]
for n,body in epochs['final'].items():patch.extend(difflib.unified_diff(epochs['before'].get(n,b'').decode().splitlines(True),body.decode().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
with (b/'change.patch').open('x') as f:f.write(''.join(patch))
for name in ('before-source.tar.gz','before-source-sha256.json','midpoint-source.tar.gz','midpoint-source-sha256.json','final-source.tar.gz','final-source-sha256.json','change.patch','design.json','bidirectional-design.json'):
 copy(b/name,p/name)
for label,folder in [('midpoint','recorded'),('bidirectional','bidirectional-recorded'),('first-default','default')]:
 for c in cases:
  out=p/'recorded'/label/c;copy(b/folder/c/'freeze.json',out/'freeze.json')
  for n in ('sensor-only','results'):copy(b/folder/c/(n+'.json'),out/(n+'.json.gz'),True)
  if label!='first-default':
   for n in ('freeze.json','audit.json'):copy(b/(folder+'-raw')/c/n,out/('raw-'+n))
for folder in ('analytic','analytic-raw','bidirectional-analytic','bidirectional-analytic-raw','final-default-analytic','final-default-raw','final-timing-controls','surface-math','surface-controls','raw-pair-probes','compatibility-depth','compatibility-rgbd','compatibility-timing'):
 for src in sorted((b/folder).rglob('*')):
  if src.is_file():
   target=p/'checks'/folder/src.relative_to(b/folder)
   compress=src.stat().st_size>100000 and src.suffix=='.json'
   if compress:target=target.with_name(target.name+'.gz')
   copy(src,target,compress)
write(p/'checks/surface-math-extension.json',dict(first_source_reconstructed_to_original_sha256=sha(oldmath.encode()),reason='same algebraic checker extended with two native weak-plane and expiry controls; original results retained',native_algorithm_parameters_changed=False))
(p/'checks/surface-math-first-source.py').write_text(oldmath)
copy(b/'replay-defaults.py',p/'checks/replay-defaults.py')
copy(b/'direction-symmetry.json',p/'checks/first-direction-symmetry.json')
for log in sorted(Path('/tmp').glob('rustdrive-midpoint-*.log')):copy(log,p/'logs'/log.name)
copy(Path(__file__),p/'seal.py')
write(p/'core-output-sha256.json',core)
validation=dict(native_tests=49,core_tests=371,total_rust_tests=420,scenario_replays=49,core_outputs_unchanged=245,compatibility_outcomes_identical=1440,
 raw_recorded_normal_systems=systems,raw_recorded_model_generations=1074,independently_checked_weak_plane_failures=2,
 heldout_accuracy_claim=False,complete_physical_accuracy=False,default_adoption=False,prior_evidence_entries_unchanged=historical)
write(p/'validation.json',validation)
summary=dict(schema_version=1,implemented='optional symmetric_midpoint and bidirectional_midpoint measured surface fits',default='target_plane unchanged',
 before_binary_sha256=sha((b/'before-binary').read_bytes()),midpoint_binary_sha256=sha((b/'midpoint-binary').read_bytes()),binary_sha256=binary,
 source_members=len(finalpins),recorded_scores=scores,all_recorded_protocols_pass=False,default_adoption=False,
 data_role='three already viewed FR1 regression recordings, including previously viewed RPY',
 uncertainty='no calibrated covariance or probability claim; bidirectional matches share samples and fused history',
 subjective_maturity_percent=20,goal_30_percent_complete=False,validation=validation)
write(p/'summary.json',summary)
inventory={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()}
write(p/'files.json',inventory)
print(json.dumps(dict(packet_files=len(inventory),packet_bytes=sum(x['bytes'] for x in inventory.values()),source_members=len(finalpins),raw_systems=systems,previous_entries=historical,default_adoption=False)),flush=True)
