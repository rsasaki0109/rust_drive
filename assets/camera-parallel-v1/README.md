# Parallel camera inference evidence v1

This packet records an optional CPU optimization with unchanged model bytes,
view geometry, thresholds, per-view f32 computations, merge and vehicle rules.
One immutable tract model plan is shared across at most four scoped workers;
every worker is joined and original view order is restored before integration.
Old serial executables, source pins and lockfiles are unchanged.

The paired local measurement order is serial / parallel / parallel / serial,
fixed before inference. Serial first-image processing is 2.260491 / 2.394728 s;
parallel is 0.625560 / 0.578763 s. Ratio of means is **3.865×**, a **74.13%**
processing reduction on this four-CPU-quota host. All four 200 ms deadline gates
still fail: every run infers one image, skips seven expired images and exposes
zero available results. Timing uses the unchanged controller and boundary;
model loading/preflight precede the acquisition schedule, and the final poll
and audit JSON publication are excluded. There is no inference warm-up.

Sixteen previously viewed images are fixed: TUM RGB indices 102–109 and the
same eight BDD road frames. Each runs twice. All per-view pixel hashes,
measurements, ordering, merged hypotheses, ranks, vehicle decisions and work
counts match immutable serial output exactly, excluding measured durations.
The packet contains **144 actual model forwards**: 16 from the four timed
replays and 128 from 32 image-parity executions. BDD's inherited 78 TP / 31 FP /
60 FN and failed accuracy gates remain unchanged through exact output parity;
there is no new accuracy score or fresh holdout.

- Five new Rust tests pass: concurrent ordering, cleanup after errors and
  multiple panics, single-view execution, and bounded worker creation. The
  binary's 39 test executions include 34 existing shared tests.
- Twenty-one existing timing/source controls and four image-entry failures
  pass, with two existing-output protections. One further control confirms the
  new producer is rejected by old pinned serial-report admission.
- Every actual timed guard result reproduces the unchanged standalone guard.
  Ages, checkpoints, deadline counts and unchanged detections are independently
  audited. No real backend association occurs in these late-result trials.
- Required core checks pass: 371 tests, 49 scenario replays and 245 unchanged outputs.

`verification/benchmark/` preserves the frozen ABBA plan, executable receipts,
all four raw timing outputs and trial metadata. `verification/parity/` preserves
all 32 real-image reports. Guard replays, the distinct-producer rejection and
the complete verification summary remain alongside them. `controls/` retains
synthetic fixtures and input failures; special/sparse files have descriptors.
`source/`, `source-sha256.json` and `checks/` preserve code/docs and local checks.
`environment.json` records nonsecret host details observed after the experiment,
without measuring load/reservation. `files.json` indexes files except itself.

Reproduce from the repository root with the pinned model, TUM cache and
previously prepared BDD data:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/onnx/camera_parallel_check.py controls \
  --output artifacts/camera-parallel-controls-new
python3 integrations/onnx/camera_parallel_check.py run \
  --controls artifacts/camera-parallel-controls-new \
  --output artifacts/camera-parallel-verification-new
```

The standard-library checker preserves hardware-dependent speed/deadline
observations separately from correctness. CI is configured; no remote run has
been observed for this change. See the [implementation and compatibility
contract](../../docs/camera-parallel.md). The parallel producer has its own
algorithm/source receipts; old serial admission remains strict. Concurrent
scratch memory is higher, without a measured peak-memory bound or universal
speedup/real-time guarantee. Live-camera transport, independent object-identity
accuracy and vehicle integration remain unverified.

Code carries [Apache-2.0](LICENSE-code.txt). Derived measurements carry their
separate [TUM CC BY 4.0 attribution](LICENSE-TUM-data.txt) or
[BDD Regents terms](LICENSE-BDD-data.txt); original RGB/depth images, poses and
weights are not redistributed. Subjective maturity relative to
Autoware/Apollo/openpilot remains **about 20%**, with the 30% waypoint unmet.
