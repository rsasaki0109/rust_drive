#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
cargo run --release --locked --bin rustdriving -- run --scenario scenarios/mission.json --seed 7 --output artifacts/demo
cargo run --release --locked --bin rustdriving -- replay --log artifacts/demo/sensors.jsonl --output artifacts/demo/replay
if ! python3 -c 'import PIL' >/dev/null 2>&1; then
  echo 'GIF rendering requires Pillow: python3 -m pip install -r scripts/requirements-demo.txt' >&2
  exit 2
fi
python3 scripts/render_demo.py artifacts/demo/run.json --output "${1:-artifacts/demo/demo.gif}"
