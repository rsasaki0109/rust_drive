#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
rne_plant="${1:-dynamic}"
case "$rne_plant" in kinematic|dynamic) ;; *) echo 'Plant must be kinematic or dynamic' >&2; exit 2 ;; esac
rne_directory="$(cd .. && pwd)/RobotNativeEngine"
if [[ "$(git -C "$rne_directory" rev-parse HEAD)" != "$(cat integrations/rne/rne-revision.txt)" ]]; then
  echo 'Pinned RNE required: bash scripts/setup-rne.sh' >&2
  exit 2
fi
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --scenario "${3:-scenarios/mission.json}" --plant "$rne_plant" --seed 7 --output "artifacts/rne-$rne_plant"
cargo run --release --locked --bin rustdrive -- replay \
  --log "artifacts/rne-$rne_plant/sensors.jsonl" --output "artifacts/rne-$rne_plant/replay"
if ! python3 -c 'import PIL' >/dev/null 2>&1; then
  echo 'GIF rendering requires Pillow: python3 -m pip install -r scripts/requirements-demo.txt' >&2
  exit 2
fi
python3 scripts/render_demo.py "artifacts/rne-$rne_plant/run.json" --output "${2:-artifacts/rne-$rne_plant/demo.gif}"
