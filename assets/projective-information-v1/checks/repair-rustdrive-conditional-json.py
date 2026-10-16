from pathlib import Path
import importlib.util,sys,json,hashlib
import numpy as np
r=Path('/workspace/rust_drive');sys.path.insert(0,str(r/'scripts'));b=r/'artifacts/projective-information/calibration'
def sha(x):return hashlib.sha256(x).hexdigest()
source=r/'scripts/check-projective-information-calibration.py';f=json.loads((b/'freeze.json').read_text())
assert sha(source.read_bytes())==f['source_sha256'][str(source.relative_to(r))]
with (b/'first-source.py').open('xb') as out:out.write(source.read_bytes())
partial=b/'heldout-results.json';assert partial.is_file() and not (b/'first-heldout-results.partial').exists()
partial_hash=sha(partial.read_bytes());partial.rename(b/'first-heldout-results.partial')
spec=importlib.util.spec_from_file_location('calibration',source);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);original=m.write

def primitive(x):
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:primitive(v) for k,v in x.items()}
 if isinstance(x,list):return [primitive(v) for v in x]
 return x
m.write=lambda path,value:original(path,primitive(value));m.evaluate()
(b/'serialization-repair.json').write_text(json.dumps(dict(original_source_sha256=sha(source.read_bytes()),freeze_sha256=sha((b/'freeze.json').read_bytes()),original_partial_sha256=partial_hash,original_failure_sha256=sha(Path('/tmp/rustdrive-information-calibration-evaluate.log').read_bytes()),repair_source_sha256=sha(Path(__file__).read_bytes()),repair='convert NumPy scalar booleans/floats to JSON primitives only',numerical_model_or_inflation_changed=False,heldout_used_to_refit=False),indent=2)+'\n')
