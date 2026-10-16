from pathlib import Path
import json,gzip,hashlib,tarfile,re,difflib,urllib.parse
r=Path('/workspace/rust_drive');b=r/'artifacts/projective-affine';p=r/'assets/projective-affine-v1';cases=('tum-fr1-room-temporal','tum-fr1-desk2-independent')
def sha(body):return hashlib.sha256(body).hexdigest()
def doc(path):return json.loads(path.read_text())
def write(path,value):
 path.parent.mkdir(parents=True,exist_ok=True)
 with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')
def copy(src,dst,compress=False):
 dst.parent.mkdir(parents=True,exist_ok=True)
 with dst.open('xb') as f:f.write(gzip.compress(src.read_bytes(),mtime=0) if compress else src.read_bytes())
assert not p.exists()
raw=doc(b/'raw/summary.json');assert raw['pass_raw_mathematics'] and not raw['ground_truth_decoded']
gate=doc(b/'adoption-gate.json');assert gate['implementation_integrity_pass'] and not gate['default_adoption_pass'];assert gate['cases'][cases[0]]['default_adoption_pass'] and not gate['cases'][cases[1]]['default_adoption_pass']
unit=Path('/tmp/rustdrive-affine-unit-corrected.log').read_text();core=Path('/tmp/rustdrive-affine-core-final.log').read_text()
assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',unit)))==37 and 'FAILED' not in unit
assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed;',core)))==371 and 'FAILED' not in core and core.count('"verified": true')==49
entries=0
for m in (r/'assets').glob('*/files.json'):
 for name,v in doc(m).items():
  body=(m.parent/name).read_bytes();assert sha(body)==v['sha256'] and len(body)==v['bytes'];entries+=1
assert entries==955
source_names=set(doc(b/'before-source-sha256.json'))|{'scripts/check-projective-rgbd-exposure.py','docs/projective-rgbd-exposure.md','scripts/env.sh','scripts/check.sh','scripts/check-multireference-rgbd.py','scripts/multireference_rgbd_common.py','scripts/requirements-visual.txt'}
for c in cases:
 source_names.update(doc(b/'corrected'/c/'freeze.json')['source_sha256']);source_names.update(doc(b/'raw'/c/'freeze.json')['source_sha256'])
source_pins={n:sha((r/n).read_bytes()) for n in sorted(source_names)}
with tarfile.open(b/'final-source.tar.gz','w:gz') as t:
 for n in sorted(source_names):t.add(r/n,arcname=n)
(b/'final-source-sha256.json').write_text(json.dumps(source_pins,indent=2)+'\n')
with tarfile.open(b/'before-source.tar.gz') as t:before={m.name:t.extractfile(m).read().decode() for m in t.getmembers() if m.isfile()}
changes=[]
for n in sorted(set(before)|{'scripts/check-projective-rgbd-exposure.py','docs/projective-rgbd-exposure.md'}):
 changes.extend(difflib.unified_diff(before.get(n,'').splitlines(True),(r/n).read_text().splitlines(True),fromfile='a/'+n,tofile='b/'+n))
(b/'change.patch').write_text(''.join(changes))
binary=sha((r/'integrations/projective-depth/target/release/rustdriving-projective-depth').read_bytes())
scores={};geometric=image=0
for c,expected in zip(cases,(84,124)):
 folder=b/'corrected'/c;freeze=doc(folder/'freeze.json');s=doc(folder/'results.json')['summary'];scores[c]=s
 assert freeze['binary_sha256']==binary and s['accepted']==179 and s['root_accurate']==expected and not s['lost']
 for n,h in freeze['source_sha256'].items():assert source_pins[n]==h,(c,n)
 for n,h in doc(b/'raw'/c/'freeze.json')['source_sha256'].items():assert source_pins[n]==h,(c,n)
 ledger=doc(folder/'sensor-only.json');assert not ledger['ground_truth_parsed'];assert freeze['pose_resets']==0 and not freeze['ground_truth_operational']
 copy(folder/'freeze.json',p/'recorded'/c/'freeze.json')
 for n in ('sensor-only','results'):copy(folder/(n+'.json'),p/'recorded'/c/(n+'.json.gz'),True)
 for n in ('freeze.json','audit.json'):copy(b/'raw'/c/n,p/'raw'/c/n)
 a=doc(b/'raw'/c/'audit.json')['summary'];geometric+=a['raw_iteration_equations']+a['returned_pose_equations'];image+=a['raw_photometric_equations']
 original=json.loads(gzip.decompress((r/'assets/projective-depth-rust-v1'/c/'sensor-only.json.gz').read_bytes()))
 assert doc(b/'corrected-default'/c/'sensor-only.json')==original
 copy(b/'corrected-default'/c/'freeze.json',p/'default'/c/'freeze.json')
 write(p/'default'/c/'replay.json',dict(default_outcomes_identical=180,ledger_sha256=sha((b/'corrected-default'/c/'sensor-only.json').read_bytes()),original='assets/projective-depth-rust-v1/'+c+'/sensor-only.json.gz',summary=doc(b/'corrected-default'/c/'results.json')['summary']))
assert geometric==7963
exposure=doc(b/'corrected-exposure/summary.json');assert exposure['pass_exposure_controls'] and exposure['binary_sha256']==binary
assert exposure['checker_sha256']==source_pins['scripts/check-projective-rgbd-exposure.py']
assert doc(b/'before-exposure/summary.json')['within_known_motion_gate']==0
life=doc(b/'corrected-lifecycle/summary.json');assert life['pass_lifecycle'] and life['binary_sha256']==binary
analytic=doc(b/'corrected-analytic/summary.json');assert analytic['analytic_controls_pass'] and analytic['binary_sha256']==binary and len(analytic['rejected_transport_cases'])==27
assert len(doc(b/'raw/analytic-audit.json')['rejected_corruptions'])==10
for name in ('before-source.tar.gz','before-source-sha256.json','first-source.tar.gz','first-source-sha256.json','final-source.tar.gz','final-source-sha256.json','change.patch','design.json'):copy(b/name,p/name)
copy(b/'diagnostic-source.py',p/'diagnostic/source.py');copy(b/'diagnostic-equations.py',p/'diagnostic/equations.py');copy(b/'diagnostic-provenance.json',p/'diagnostic/provenance.json')
copy(r/'artifacts/projective-photo-diagnostic/diagnostic.json',p/'diagnostic/results.json.gz',True);copy(Path('/tmp/rustdrive-affine-diagnostic-final.log'),p/'diagnostic/log.txt')
copy(Path('/tmp/compare-rustdrive-exposure.py'),p/'diagnostic/compare-exposure.py')
for n in ('summary.json','controls.json'):copy(b/'before-exposure'/n,p/'before-exposure'/(n+'.gz' if n=='controls.json' else n),n=='controls.json')
copy(b/'recorded'/cases[0]/'freeze.json',p/'first'/cases[0]/'freeze.json');copy(b/'recorded'/cases[0]/'sensor-only.json',p/'first'/cases[0]/'sensor-only.json.gz',True)
copy(Path('/tmp/rustdrive-affine-recorded.log'),p/'first/symmetry-failure.log')
for f in (b/'corrected-analytic').glob('*.json'):copy(f,p/'analytic'/f.name)
for folder,files in [('corrected-exposure',('summary.json','controls.json')),('corrected-lifecycle',('summary.json','sequence.json'))]:
 for n in files:copy(b/folder/n,p/('exposure' if folder.endswith('exposure') else 'lifecycle')/(n+'.gz' if n!='summary.json' else n),n!='summary.json')
for n in ('summary.json','analytic-audit.json'):copy(b/'raw'/n,p/'raw'/n)
copy(b/'adoption-gate.json',p/'adoption-gate.json');copy(b/'supervision/summary.json',p/'supervision.json');copy(b/'multireference/summary.json',p/'multireference.json')
outputs=doc(r/'assets/projective-rgbd-v1/core-output-sha256.json');assert len(outputs)==245
for n,h in outputs.items():assert sha((r/n).read_bytes())==h,n
write(p/'core-output-sha256.json',outputs)
for name,log in {'core-check':'core-final','native-unit':'unit-corrected','native-clippy':'clippy-corrected','native-build':'build-corrected','recorded':'corrected','default-replay':'corrected-default','raw':'raw','exposure':'corrected-exposure','lifecycle':'corrected-lifecycle'}.items():copy(Path('/tmp/rustdrive-affine-'+log+'.log'),p/'logs'/(name+'.log'))
write(p/'summary.json',dict(schema_version=1,feature='Affine exposure elimination in maintained optional Rust RGB-D tracker',binary_sha256=binary,source_members=len(source_names),scores=scores,independent_geometry_systems=geometric,independent_image_systems=image,synthetic_exposure_before_pass=0,synthetic_exposure_after_pass=4,negative_exposure_controls=4,default_outcomes_identical=360,implementation_integrity_pass=True,default_adoption_pass=False,viewed_development=True,ground_truth_operational=False,subjective_maturity_percent=20))
write(p/'validation.json',dict(native_tests=37,core_tests=371,scenario_and_replay_checks=49,core_outputs_unchanged=245,earlier_asset_entries_unchanged=entries,raw_geometry_systems=geometric,raw_image_systems=image,raw_model_generations=358,raw_audit_without_numeric_truth=True,raw_analytic_corruptions_rejected=10,transport_faults_rejected=27,exposure_positive_controls=4,exposure_negative_controls=4,lifecycle_pass=True,default_changed=False,adopted=False))
copy(Path(__file__),p/'seal.py')
files={str(f.relative_to(p)):dict(bytes=f.stat().st_size,sha256=sha(f.read_bytes())) for f in sorted(p.rglob('*')) if f.is_file()};write(p/'files.json',files)
print(json.dumps(dict(files=len(files),bytes=sum(v['bytes'] for v in files.values()),source_members=len(source_names),geometry_systems=geometric,image_systems=image)))
