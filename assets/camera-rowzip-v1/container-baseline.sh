#!/bin/sh
set -eu
cd /workspace/rust_drive
python -m pip install --no-index --no-deps --target artifacts/camera-rowzip-v1/container-deps --find-links artifacts/camera-rowzip-v1/wheels numpy==2.3.5 Pillow==12.3.0
python - <<'PY'
import json,platform
from importlib.metadata import version
from pathlib import Path
p=Path('artifacts/camera-rowzip-v1/container-environment.json')
p.write_text(json.dumps(dict(platform=platform.platform(),python=platform.python_version(),libc=platform.libc_ver(),numpy=version('numpy'),pillow=version('Pillow'),os_release=Path('/etc/os-release').read_text(),cpu_quota=Path('/sys/fs/cgroup/cpu.max').read_text().strip(),network='none',cpu_hardware='same host; not independent hardware'),indent=2)+'\n')
PY
python -B assets/camera-depthwise-v1/verify.py
python -B integrations/onnx/camera_depthwise_reproduce.py --output artifacts/camera-rowzip-v1/container-baseline
