# Camera-result freshness evidence v1

This packet records the offline `rustdriving-camera-guard` implementation and
its two-clock admission/recovery checks. The existing detector, tracker,
one-process image sequencer, lockfiles and prior packets are unchanged.

The implementation separates source acquisition from receive/poll time, latches
expiry once, excludes queued pre-fault results and requires consecutive fresh
acquisitions before publishing available confirmed/coasting hypotheses.
An object's observation age is bounded independently of the latest image.
Availability grants no vehicle-control permission.

- 10 new Rust tests, plus all 11 existing tracker tests, pass locally.
- 46 synthetic sequences / 1,029 events and four injected-control sequences on
  eight unchanged recorded detector reports run twice identically.
- All 50 extracted backend histories exactly reproduce the standalone tracker.
  Independent motion/lifecycle and min-cost-flow checks pass; the flow solver
  agrees with exhaustive enumeration on 96 small graphs.
- 30 fatal input/output-budget controls and two existing-output protections pass.
- Required core checks pass; all 49 scenario replays retain the 245 pinned outputs.

The recorded inputs are the previously viewed TUM Freiburg3 sitting_xyz RGB
indices 102–109, 1341845824.165751–1341845824.398039 s (0.232288 s).
Saved detector measurements and original acquisition times are unchanged.
**All receive times, faults, queued packets and missing deliveries are injected.**
No new inference, images, weights, depth, poses or object-identity labels enter
this check. The final deadline example remains held with only one available
post-deadline acquisition. This is failure-behavior verification, with no
measured real sensor-fault rate, automotive availability or identity accuracy.

`verification/` contains generated traces, both outputs, independent backend
replays, raw rejection diagnostics and the eight source reports. Sparse files,
symlinks and the FIFO used by rejection checks have descriptors instead of
redistributed special files. `source/` and `source-sha256.json` preserve the
reviewed sources; `checks/` preserves local Rust/core validation logs.
`protocol-verification.json` records immutable-source/output checks and a replay
from this published packet. `files.json` indexes packet files except itself.

Reproduce from the repository root with:

```bash
source scripts/env.sh
cargo test --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/camera-tracking/freshness_check.py --output artifacts/camera-freshness-new
```

The checker uses only Python's standard library and local saved reports. CI is
configured, without an observed remote run for this change. The
[contract and runnable saved example](../../docs/camera-freshness.md) explain
clock responsibility, budgets and error behavior. Source code carries the
[Apache-2.0 license](LICENSE-code.txt); derived TUM measurements carry the
separate [CC BY 4.0 attribution and source limitations](LICENSE-data.txt).
Subjective maturity relative to Autoware/Apollo/openpilot remains about **20%**;
the 30% waypoint is unmet.
