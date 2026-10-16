from pathlib import Path
import importlib.util,sys,json,hashlib,types
import numpy as np
r=Path('/workspace/rust_drive');sys.path.insert(0,str(r/'scripts'));b=r/'artifacts/projective-information/calibration';source=r/'scripts/check-projective-information-calibration.py'
def sha(x):return hashlib.sha256(x).hexdigest()
f=json.loads((b/'freeze.json').read_text());assert sha(source.read_bytes())==f['source_sha256'][str(source.relative_to(r))]
def primitive(x):
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:primitive(v) for k,v in x.items()}
 if isinstance(x,list):return [primitive(v) for v in x]
 return x
s=importlib.util.spec_from_file_location('cal',source);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
def compare(path,value):assert json.loads(path.read_text())==primitive(value)
m.write=compare;m.json=types.SimpleNamespace(loads=json.loads,dumps=lambda value:json.dumps(primitive(value)));m.evaluate()
receipt=dict(original_source_sha256=sha(source.read_bytes()),freeze_sha256=sha((b/'freeze.json').read_bytes()),original_partial_sha256=sha((b/'first-heldout-results.partial').read_bytes()),original_failure_sha256=sha(Path('/tmp/rustdrive-information-calibration-evaluate.log').read_bytes()),console_failure_sha256=sha(Path('/tmp/rustdrive-information-calibration-repaired.log').read_bytes()),repair_source_sha256=sha(Path('/tmp/repair-rustdrive-conditional-json.py').read_bytes()),verification_source_sha256=sha(Path(__file__).read_bytes()),repair='convert NumPy scalar booleans/floats to JSON primitives only',numerical_model_or_inflation_changed=False,heldout_used_to_refit=False,recomputed_original_values_match=True)
with (b/'serialization-repair.json').open('x') as out:json.dump(receipt,out,indent=2);out.write('\n')
