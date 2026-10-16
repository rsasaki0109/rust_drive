from pathlib import Path
import tarfile,json,hashlib,shutil,subprocess
r=Path('/workspace/rust_drive');trial=r/'artifacts/projective-information/heldout-rpy';out=Path('/tmp/rustdrive-rpy-source-replay');assert not out.exists();out.mkdir()
f=json.loads((trial/'design.json').read_text())
with tarfile.open(trial/'source.tar.gz') as archive:
 members=[m for m in archive.getmembers() if m.isfile()];assert {m.name for m in members}==set(f['source_sha256'])
 for m in members:
  assert not Path(m.name).is_absolute() and '..' not in Path(m.name).parts
  body=archive.extractfile(m).read();assert hashlib.sha256(body).hexdigest()==f['source_sha256'][m.name]
  target=out/m.name;target.parent.mkdir(parents=True,exist_ok=True)
  with target.open('xb') as stream:stream.write(body)
for name in ('manifest.json',):
 target=out/'data/tum-fr1-rpy-heldout'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(r/'data/tum-fr1-rpy-heldout'/name,target)
shutil.copytree(r/'data/tum-fr1-rpy-heldout/raw',out/'data/tum-fr1-rpy-heldout/raw')
folder=out/'artifacts/projective-information/heldout-rpy';folder.mkdir(parents=True);shutil.copy2(trial/'design.json',folder/'design.json')
binpath=out/'integrations/projective-depth/target/release/rustdriving-projective-depth';binpath.parent.mkdir(parents=True);shutil.copy2(r/'integrations/projective-depth/target/release/rustdriving-projective-depth',binpath)
subprocess.run(['/workspace/.rustdrive-tools/demo-venv/bin/python','scripts/projective-heldout.py','run','--recording','rpy'],cwd=out,check=True)
checks={}
for mode in ('depth','timing'):
 got=(folder/mode/'sensor-only.json').read_bytes();expected=(trial/mode/'sensor-only.json').read_bytes();assert got==expected,mode
 checks[mode]=dict(sensor_outcomes_identical=180,ledger_sha256=hashlib.sha256(got).hexdigest())
(r/'artifacts/projective-information/cold-source-replay.json').write_text(json.dumps(dict(pass_frozen_source_replay=True,outcomes_identical=360,checks=checks,original_native_binary_and_source=True,numeric_reference_poses_read=False),indent=2)+'\n')
print('all 360 cold sensor outcomes reproduce from the archived source',flush=True)
