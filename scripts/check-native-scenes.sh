#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
cargo build --release --locked --bin rustdrive --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
python3 scripts/check-native-scenes.py "$@"
