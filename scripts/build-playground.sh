#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
cargo build --release --locked --target wasm32-unknown-unknown --manifest-path integrations/playground/Cargo.toml
mkdir -p web/public
cp integrations/playground/target/wasm32-unknown-unknown/release/rustdriving_playground.wasm web/public/drive.wasm
npm --prefix web run build
