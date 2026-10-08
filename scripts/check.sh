#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo build --workspace --release --locked
for scenario in mission blocked lidar-fault gnss-fault; do
  target/release/rustdrive run --scenario "scenarios/$scenario.json" --seed 7 --output "artifacts/check/$scenario"
done
