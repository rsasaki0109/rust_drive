#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
rne_directory="$(cd .. && pwd)/RobotNativeEngine"
if [[ "$(git -C "$rne_directory" rev-parse HEAD)" != "$(cat integrations/rne/rne-revision.txt)" ]]; then
  echo 'Pinned RNE required: bash scripts/setup-rne.sh' >&2
  exit 2
fi
command -v blender >/dev/null || { echo 'Install Blender to render the 3D replay' >&2; exit 2; }
python3 -c 'import PIL' >/dev/null || { echo 'Activate the documented Pillow environment' >&2; exit 2; }
rne_trace_directory="${3:-artifacts/rne-3d}"
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --scenario "${2:-scenarios/route-handover-fast.json}" --plant dynamic --seed 7 --output "$rne_trace_directory"
cargo run --release --locked --bin rustdrive -- replay \
  --log "$rne_trace_directory/sensors.jsonl" --output "$rne_trace_directory/replay"
python3 scripts/render_demo_3d.py "$rne_trace_directory/run.json" --output "${1:-artifacts/rne-3d/demo.gif}" "${@:4}"
