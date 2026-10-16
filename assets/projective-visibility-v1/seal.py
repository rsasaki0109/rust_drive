from pathlib import Path
import ast, difflib, gzip, hashlib, json, re, tarfile

r=Path('/workspace/rust_drive'); b=r/'artifacts/projective-visibility-v1'; p=r/'assets/projective-visibility-v1'
cases=('tum-fr1-room-temporal','tum-fr1-desk2-independent','tum-fr1-rpy-heldout')
def sha(body):return hashlib.sha256(body).hexdigest()
def doc(path):return json.loads(path.read_text())
def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def copy(source,target,compress=False):
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('xb') as f:f.write(gzip.compress(source.read_bytes(),mtime=0) if compress else source.read_bytes())
def archive(path):
    with tarfile.open(path) as t:return {m.name:t.extractfile(m).read() for m in t.getmembers() if m.isfile()}
assert not p.exists()
historical=0
for inventory in (r/'assets').glob('*/files.json'):
    for n,v in doc(inventory).items():
        body=(inventory.parent/n).read_bytes();assert len(body)==v['bytes'] and sha(body)==v['sha256'],(inventory,n);historical+=1
assert historical==1687
epochs={}
for label in ('before','candidate'):
    epochs[label]=archive(b/(label+'-source.tar.gz'));pins=doc(b/(label+'-source-sha256.json'))
    assert set(pins)==set(epochs[label]) and all(sha(epochs[label][n])==h for n,h in pins.items())
diagnostic=archive(b/'diagnostic-source.tar.gz');df=doc(b/'diagnosis/freeze.json')
assert set(diagnostic)==set(df['source_sha256']) and all(sha(diagnostic[n])==h for n,h in df['source_sha256'].items())
compat=doc(b/'epoch-compatibility.json');assert compat['diagnostic_math_unchanged']
for n,names in compat['verified_functions'].items():
    def selected(body):return {f.name:ast.dump(f,include_attributes=False) for f in ast.parse(body).body if isinstance(f,ast.FunctionDef) and f.name in names}
    assert selected(diagnostic[n])==selected((r/n).read_bytes()) and set(selected(diagnostic[n]))==set(names),n
names=set(epochs['candidate'])|{'docs/projective-visibility.md'}
epochs['final']={n:(r/n).read_bytes() for n in sorted(names)}
for n,body in epochs['candidate'].items():
    if n.endswith(('.rs','.py')):assert epochs['final'][n]==body,n
for n in ('integrations/projective-depth/Cargo.toml','integrations/projective-depth/Cargo.lock'):
    assert len({sha(data[n]) for data in epochs.values()})==1,n
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes())
assert sha((b/'candidate-binary').read_bytes())==binary
for name,expected in (('unit',59),('core',371)):
    log=Path('/tmp/rustdrive-visibility-'+name+'.log').read_text()
    assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',log)))==expected and 'FAILED' not in log
    if name=='core':assert log.count('"verified": true')==49
core=doc(r/'assets/projective-fusion-bias-v1/core-output-sha256.json')
assert len(core)==245 and all(sha((r/n).read_bytes())==h for n,h in core.items())
compatible=0
for mode in ('depth','rgbd','timing'):
    f=doc(b/('compatibility-'+mode)/'freeze.json');s=doc(b/('compatibility-'+mode)/'summary.json')
    assert f['binary_sha256']==binary and s['pass_exact_compatibility'] and not s['numeric_reference_poses_decoded']
    assert all(sha(epochs['final'][n])==h for n,h in f['source_sha256'].items())
    compatible+=s['identical_outcomes']
assert compatible==1440
analytic=doc(b/'analytic/summary.json')
assert analytic['binary_sha256']==binary and analytic['visibility']=='free_space_decay'
assert len(analytic['rejected_transport_cases'])==75
mutations=doc(b/'analytic-raw/analytic-audit.json')['rejected_corruptions']
assert all(n in mutations for n in ('false free-space visibility count','false visibility removal count','missing visibility evidence receipt'))
control=doc(b/'visibility-controls/summary.json')
assert control['pass_visibility_controls'] and control['identical_valid_outcomes_after_rejection']==9
assert control['dynamic']['legacy']['stale_foreground_cells_after_offset6']==144
assert control['dynamic']['decay']['stale_foreground_cells_after_offset6']==0
assert control['motion']['reference_warps']>=1 and control['motion']['max_position_error_m']<=.01 and control['motion']['max_rotation_error_rad']<=.01
assert 'retained ghost model with valid removal receipt' in control['rejected_corruptions']
math=doc(b/'math-controls/math-controls.json');assert math['known_linearized_axes']==6 and math['fixed_pixel_support']
for folder in ('visibility-controls','math-controls'):
    f=doc(b/folder/'freeze.json');assert all(sha(epochs['final'][n])==h for n,h in f['source_sha256'].items())
    if 'binary_sha256' in f:assert f['binary_sha256']==binary
assert doc(b/'diagnosis/summary.json')['pass_diagnostic_integrity']
paired_matches=0
for c in cases:
    d=doc(b/'diagnosis'/(c+'.json'));assert len(d['probes'])==15 and len(d['visibility'])==179
    for probe in d['probes']:
        assert probe['same_pixel_support_and_additive_fixed_system_verified']
        paired_matches+=probe['common_pairs']
    for mode in ('estimated','reference'):
        s=d['summary']['visibility'][mode]
        assert s['tested']==s['consistent']+s['occluded']+s['free_space']
        assert s['free_space_fraction']==s['free_space']/s['tested']
assert paired_matches==667106
adoption=dict(schema_version=1,design_sha256=sha((b/'design.json').read_bytes()),criterion=doc(b/'design.json')['adoption'],
    baseline={},checks={},default_changed=False,pass_default_adoption=False,physical_accuracy_claim=False)
scores={};systems=0;warps=0;visibility_totals={}
for c,label,expected,threshold in zip(cases,('room','desk2','rpy'),(67,161,74),(65,161,74),strict=True):
    folder=b/'recorded'/c;rawfolder=b/('raw-'+label)/c
    ledger,result,freeze=(doc(folder/n) for n in ('sensor-only.json','results.json','freeze.json'))
    raw,rawfreeze=doc(rawfolder/'audit.json'),doc(rawfolder/'freeze.json')
    assert not ledger['ground_truth_parsed'] and len(ledger['frames'])==180
    assert [v['source_index'] for v in ledger['frames']]==list(range(100,280))
    assert [v['update'] for v in ledger['frames']]==[v['update'] for v in result['frames']]
    s=result['summary'];assert s['root_accurate']==expected and s['accepted']==179 and not s['lost'] and not s['complete_accuracy_pass']
    assert raw['pass_raw_mathematics'] and raw['summary']['physical_information_checked'] and not raw['summary']['ground_truth_decoded']
    assert raw['summary']['frames']==180 and raw['summary']['fused_model_generations']==179 and raw['summary']['returned_pose_equations']==179
    assert raw['summary']['raw_iteration_equations']==s['independently_checked_normal_steps']
    assert rawfreeze['original_sensor_ledger_sha256']==sha((folder/'sensor-only.json').read_bytes())
    for f in (freeze,rawfreeze):assert all(sha(epochs['candidate'][n])==h for n,h in f['source_sha256'].items())
    assert freeze['binary_sha256']==binary and freeze['visibility']=='free_space_decay'
    systems+=raw['summary']['raw_iteration_equations']+179;warps+=raw['summary']['reference_view_warps'];scores[c]=s
    basefile=r/'assets/projective-surface-v1/recorded/first-default'/c/'results.json.gz'
    base=json.loads(gzip.decompress(basefile.read_bytes()))['summary']
    adoption['baseline'][c]=dict(file=str(basefile.relative_to(r)),sha256=sha(basefile.read_bytes()))
    adoption['checks'][c]=dict(accuracy_count_pass=s['root_accurate']>=threshold,
        maximum_position_pass=s['max_position_error_m']<=base['max_position_error_m'],
        maximum_rotation_pass=s['max_rotation_error_rad']<=base['max_rotation_error_rad'],tracking_pass=s['accepted']==179 and not s['lost'])
    totals={k:0 for k in ('tested','consistent','occluded','free_space','removed')}
    for row in ledger['frames'][1:]:
        v=row['update']['fusion']['visibility'];assert v['tested']==v['consistent']+v['occluded']+v['free_space'] and v['removed']<=v['free_space']
        for k in totals:totals[k]+=v[k]
    visibility_totals[c]=totals
assert systems==10741 and warps==124
assert all(v['accuracy_count_pass'] and v['tracking_pass'] and v['maximum_rotation_pass'] for v in adoption['checks'].values())
assert [v['maximum_position_pass'] for v in adoption['checks'].values()]==[False,False,True]
write(b/'adoption.json',adoption)
finalpins={n:sha(body) for n,body in epochs['final'].items()};write(b/'final-source-sha256.json',finalpins)
with tarfile.open(b/'final-source.tar.gz','x:gz') as t:
    for n in finalpins:t.add(r/n,arcname=n,recursive=False)
patch=[]
for n,body in epochs['final'].items():patch.extend(difflib.unified_diff(epochs['before'].get(n,b'').decode().splitlines(True),body.decode().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
with (b/'change.patch').open('x') as f:f.write(''.join(patch))
for n in ('before-source.tar.gz','before-source-sha256.json','candidate-source.tar.gz','candidate-source-sha256.json','diagnostic-source.tar.gz',
          'final-source.tar.gz','final-source-sha256.json','design.json','diagnostic-decision.json','epoch-compatibility.json','adoption.json','change.patch'):
    copy(b/n,p/n)
for c,label in zip(cases,('room','desk2','rpy'),strict=True):
    folder=p/'recorded'/c;copy(b/'recorded'/c/'freeze.json',folder/'freeze.json')
    for n in ('sensor-only','results'):copy(b/'recorded'/c/(n+'.json'),folder/(n+'.json.gz'),True)
    for n in ('freeze.json','audit.json'):copy(b/('raw-'+label)/c/n,folder/('raw-'+n))
for name in ('diagnosis','math-controls','analytic','analytic-raw','visibility-controls','compatibility-depth','compatibility-rgbd','compatibility-timing'):
    for source in sorted((b/name).rglob('*')):
        if source.is_file():
            target=p/'checks'/name/source.relative_to(b/name);compress=source.stat().st_size>100000 and source.suffix=='.json'
            if compress:target=target.with_name(target.name+'.gz')
            copy(source,target,compress)
for log in sorted(Path('/tmp').glob('rustdrive-visibility-*.log')):copy(log,p/'logs'/log.name)
copy(Path(__file__),p/'seal.py');write(p/'core-output-sha256.json',core)
validation=dict(native_tests=59,core_tests=371,total_rust_tests=430,scenario_replays=49,core_outputs_unchanged=245,
    compatibility_outcomes_identical=1440,raw_recorded_normal_systems=systems,raw_recorded_model_generations=537,raw_reference_view_warps=warps,
    diagnostic_common_matches=paired_matches,known_linearized_axes=6,malformed_transport_cases=75,
    valid_synthetic_outcomes_identical_after_rejection=9,controlled_stale_foreground_cells_removed=144,
    heldout_accuracy_claim=False,complete_physical_accuracy=False,default_adoption=False,prior_evidence_entries_unchanged=historical)
write(p/'validation.json',validation)
summary=dict(schema_version=1,implemented='optional free_space_decay and paired map-point/normal diagnostic',default='visibility disabled unchanged',
    binary_sha256=binary,before_binary_sha256=sha((b/'before-binary').read_bytes()),source_members=len(finalpins),recorded_scores=scores,
    visibility_totals=visibility_totals,default_adoption=False,complete_physical_accuracy=False,data_role='same three viewed FR1 regression windows',
    limitation='accepted-pose-dependent support heuristic; no calibrated free-space probability or unique physical drift cause',
    subjective_maturity_percent=20,goal_30_percent_complete=False,validation=validation)
write(p/'summary.json',summary)
inventory={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()}
write(p/'files.json',inventory)
print(json.dumps(dict(packet_files=len(inventory),packet_bytes=sum(v['bytes'] for v in inventory.values()),source_members=len(finalpins),
    previous_entries=historical,raw_normal_systems=systems,controlled_foreground_removed=144,default_adoption=False)),flush=True)
