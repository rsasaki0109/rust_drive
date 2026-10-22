#!/usr/bin/env bash
# Keep the command's failure status and expose its log tail in check annotations.
set -uo pipefail

if [[ $# != 1 || ! -f "$1" ]]; then
  printf 'usage: ci-report.sh SCRIPT\n' >&2
  exit 2
fi

ci_log=$(mktemp) || exit 2
trap 'rm -f -- "$ci_log"' EXIT
bash --noprofile --norc -e -o pipefail "$1" 2>&1 | tee "$ci_log"
ci_status=("${PIPESTATUS[@]}")
ci_exit=${ci_status[0]}
if (( ci_exit == 0 )); then
  ci_exit=${ci_status[1]}
fi
if (( ci_exit != 0 )); then
  python3 - "$ci_log" "$ci_exit" <<'PY' || true
import sys
from pathlib import Path

tail = "\n".join(Path(sys.argv[1]).read_text(errors="replace").splitlines()[-80:])
message = f"Command exited with status {sys.argv[2]}:\n{tail[-12000:]}"
message = message.replace("%", "%25").replace("\r", "%0D").replace("\n", "%0A")
print(f"::error title=CI command failed::{message}")
PY
fi
exit "$ci_exit"
