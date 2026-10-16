#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo build --workspace --release --locked
for scenario in mission blocked lidar-fault gnss-fault occluded-crossing cut-in multiple-blocked opposing-crossings route-direct route-detour route-south route-handover route-handover-fast route-no-path route-reopen gnss-spike gnss-burst gnss-persistent-bias gnss-burst-traffic gnss-burst-traffic-hold traffic-lead-stop traffic-follower-brake traffic-queue traffic-follower-deadline traffic-fleet-queue signal-red-green signal-red-stop signal-stale-stop signal-stale-recovery signal-two-stops signal-approach-change stop-sign-single stop-sign-two stop-sign-signal stop-sign-obstacle stop-sign-gnss-recovery intersection-crossing intersection-successive intersection-blocked intersection-gnss-recovery intersection-stop-sign intersection-fast-wide intersection-slow-narrow intersection-two-zones intersection-delay-two intersection-lidar-recovery intersection-cadence-five-hz intersection-late-conflict intersection-late-delayed; do
  target/release/rustdriving run --scenario "scenarios/$scenario.json" --seed 7 --output "artifacts/check/$scenario"
  target/release/rustdriving replay --log "artifacts/check/$scenario/sensors.jsonl" --output "artifacts/check/$scenario/replay"
done
