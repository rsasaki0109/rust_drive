#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
dense_output="${1:-artifacts/dense-traffic-demo}"
for seed in 7 1 42; do
  cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
    --scenario scenarios/traffic-dense-queue.json --plant dynamic --seed "$seed" \
    --output "$dense_output/seed-$seed"
  cargo run --release --locked --bin rustdriving -- replay \
    --log "$dense_output/seed-$seed/sensors.jsonl" --output "$dense_output/seed-$seed/replay"
done
python3 scripts/check-dense-traffic.py --input "$dense_output/seed-7" "$dense_output/seed-1" "$dense_output/seed-42" \
  --output "$dense_output/acceptance.json"
python3 scripts/render_demo_3d.py "$dense_output/seed-7/run.json" \
  --output "${2:-$dense_output/demo.gif}" --samples 12 --threads 3 \
  --environment urban --camera traffic --traffic-lanes --traffic-models hatchback sedan van
