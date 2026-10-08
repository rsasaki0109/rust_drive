#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo build --workspace --release --locked
for scenario in mission blocked lidar-fault gnss-fault occluded-crossing cut-in multiple-blocked opposing-crossings route-direct route-detour route-south route-handover route-no-path route-reopen; do
  target/release/rustdrive run --scenario "scenarios/$scenario.json" --seed 7 --output "artifacts/check/$scenario"
  target/release/rustdrive replay --log "artifacts/check/$scenario/sensors.jsonl" --output "artifacts/check/$scenario/replay"
done
