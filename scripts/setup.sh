#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
if ! command -v rustup >/dev/null 2>&1; then
  command -v curl >/dev/null
  rustdrive_installer=$(mktemp)
  trap 'rm -f "$rustdrive_installer"' EXIT
  curl --proto '=https' --tlsv1.2 --fail --location https://sh.rustup.rs -o "$rustdrive_installer"
  sh "$rustdrive_installer" -y --no-modify-path --profile minimal --default-toolchain 1.90.0 --component rustfmt --component clippy
fi
rustup toolchain install 1.90.0 --profile minimal --component rustfmt --component clippy
cargo fetch --locked
cargo build --workspace --release --locked --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
cargo test --workspace --locked --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
