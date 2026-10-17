#!/bin/sh
# Linux/amd64 runtime reproduction. Build/cache inputs and wheels on the host first.
set -eu
rowzip_output=${1:?expected an existing new artifact directory with wheels/}
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$rowzip_output/container-deps"
python - "$rowzip_output" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path.cwd().resolve();out=Path(sys.argv[1]).resolve()
out.relative_to(root/'artifacts')
assert out.is_dir() and not (out/'container-baseline').exists() and not (out/'validation').exists()
pins=json.loads((root/'assets/camera-rowzip-v1/wheel-sha256.json').read_text())
for name,item in pins.items():
    data=(out/'wheels'/name).read_bytes();assert len(data)==item['bytes'] and hashlib.sha256(data).hexdigest()==item['sha256']
PY
python -m pip install --no-index --no-deps --target "$rowzip_output/container-deps" --find-links "$rowzip_output/wheels" numpy==2.3.5 Pillow==12.3.0
python - "$rowzip_output" <<'PY'
import json,platform,sys
from importlib.metadata import version
from pathlib import Path
out=Path(sys.argv[1]);value=dict(platform=platform.platform(),python=platform.python_version(),libc=platform.libc_ver(),numpy=version('numpy'),pillow=version('Pillow'),os_release=Path('/etc/os-release').read_text(),cpu_quota=Path('/sys/fs/cgroup/cpu.max').read_text().strip(),network='none',cpu_hardware='same host as builder; not independent hardware')
(out/'container-environment.json').write_text(json.dumps(value,indent=2)+'\n')
PY
python -B assets/camera-depthwise-v1/verify.py
python -B integrations/onnx/camera_depthwise_reproduce.py --output "$rowzip_output/container-baseline"
python -B integrations/onnx/camera_rowzip_check.py controls --output "$rowzip_output/controls"
python -B integrations/onnx/camera_rowzip_check.py run --controls "$rowzip_output/controls" --environment "$rowzip_output/container-environment.json" --output "$rowzip_output/validation"
