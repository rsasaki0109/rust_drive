#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
cargo build --workspace --release --locked --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
cargo +1.95.0 build --manifest-path integrations/rne/Cargo.toml --release --locked --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
python3 scripts/check_hazards.py "$@"
