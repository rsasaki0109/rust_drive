from pathlib import Path
import importlib.util
import json
import sys
import numpy as np
r=Path('/workspace/rust_drive');sys.path.insert(0,str(r/'scripts'))
from projective_camera_fit import fit,system
from projective_depth_common import fresh,sha,write,require
b=r/'artifacts/projective-camera-fit-v1';out=fresh(b/'scale-gauge-first')
sources=(Path(__file__),r/'scripts/projective_camera_fit.py',r/'scripts/check-projective-camera.py',r/'scripts/projective_depth_common.py')
pins={str(f.relative_to(r)):sha(f.read_bytes()) for f in sources}
write(out/'freeze.json',dict(source_sha256=pins,noise_seed=1705,independent_axial_coordinate_noise_std_m=.003,reference_translation_m=0.,role='conditional gauge counterexample, not physical camera noise'))
spec=importlib.util.spec_from_file_location('check',r/'scripts/check-projective-camera.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
parameters=np.array([.018,-.023]);samples=m.synthetic(parameters,False);rng=np.random.default_rng(1705)
for s in samples:
    s['source']+=rng.normal(0.,.003,s['source'].shape)
    s['target']+=rng.normal(0.,.003,s['target'].shape)
original=system(samples,parameters);smaller=system(samples,parameters+[0.,-.01]);result=fit(samples)
require(original['observable'],'the original raw-distance system mistakes noise for scale information')
require(smaller['cost_m2']<original['cost_m2'],'shrinking uniform scale reduces the objective without a metric motion baseline')
write(out/'summary.json',dict(counterexample_reproduced=True,initial_observable=original['observable'],initial_condition=original['condition'],
    original_cost_m2=original['cost_m2'],smaller_scale_cost_m2=smaller['cost_m2'],fit_status=result['status'],fit_parameters=result['parameters'],
    physical_scale_identifiable=False,physical_noise_calibration=False))
require(all(sha((r/n).read_bytes())==h for n,h in pins.items()),'frozen gauge probe sources')
print(json.dumps(json.loads((out/'summary.json').read_text())),flush=True)
