#!/usr/bin/env bash
# Source this if Cargo is not already on PATH. Never replaces an existing toolchain binding.
if ! command -v cargo >/dev/null 2>&1; then
  rustdrive_tools="${RUSTDRIVE_TOOLS_DIR:-/workspace/.rustdrive-tools}"
  if [[ ! -x "$rustdrive_tools/cargo/bin/cargo" ]]; then
    rustdrive_tools="${RUSTDRIVE_TOOLS_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/rustdrive}"
  fi
  export RUSTUP_HOME="${RUSTUP_HOME:-$rustdrive_tools/rustup}"
  export CARGO_HOME="${CARGO_HOME:-$rustdrive_tools/cargo}"
  export PATH="$CARGO_HOME/bin:$PATH"
  unset rustdrive_tools
fi
