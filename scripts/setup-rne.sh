#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/env.sh
command -v git >/dev/null
command -v rustup >/dev/null || { echo 'Install Rust first: bash scripts/setup.sh' >&2; exit 2; }
rne_revision=$(cat integrations/rne/rne-revision.txt)
rne_directory="$(cd .. && pwd)/RobotNativeEngine"
if [[ ! -e "$rne_directory" ]]; then
  mkdir "$rne_directory"
  git -C "$rne_directory" init
  git -C "$rne_directory" remote add origin https://github.com/rsasaki0109/RobotNativeEngine.git
  git -C "$rne_directory" sparse-checkout init --cone
  git -C "$rne_directory" sparse-checkout set crates third_party
  git -C "$rne_directory" fetch --depth 1 origin "$rne_revision"
  git -C "$rne_directory" checkout --detach FETCH_HEAD
fi
if [[ "$(git -C "$rne_directory" rev-parse HEAD)" != "$rne_revision" ]]; then
  echo "Expected RNE $rne_revision in $rne_directory. Existing checkout is preserved; inspect it before switching revisions." >&2
  exit 2
fi
rustup toolchain install 1.95.0 --profile minimal --component rustfmt --component clippy
cargo +1.95.0 fetch --locked --manifest-path integrations/rne/Cargo.toml
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
cargo +1.95.0 test --locked --manifest-path integrations/rne/Cargo.toml --jobs "${RUSTDRIVE_BUILD_JOBS:-4}"
