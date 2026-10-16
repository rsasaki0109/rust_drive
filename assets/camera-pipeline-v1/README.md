# Complete offline camera pipeline evidence v1

`rustdriving-camera-pipeline` connects actual image analysis, image tracking and
two-clock freshness in one command with one CPU model plan. Clock/hash-excluded
image events skip inference. The detector, tracker, freshness guard, old
executables and both workspace lockfiles retain their prior sources/pins.

The fixed input remains the previously viewed TUM Freiburg3 sitting_xyz RGB
indices 102–109, 1341845824.165751–1341845824.398039 s (0.232288 s).
Original acquisition text and RGB identities are unchanged. All delivery times
and failure events are **supplied replay controls**. CPU inference duration
does not advance replay time. No live camera, measured end-to-end latency,
automotive availability, real sensor-fault rate or object-identity score is
established. Detection quality remains unchanged.

- Four recorded protocols run twice: healthy, delayed-after-poll, explicit
  failure/recovery and deadline expiry. They perform **216 actual model forwards**.
- Each run loads one CPU model plan. Two excluded image deliveries per four
  cases skip inference. All-rejected controls open no model and run no inference.
- Every admitted detector report matches saved real inference, excluding only
  measured inference/load durations. Every freshness result matches its saved
  previous result exactly. Standalone guard and tracker replays are identical.
- Independent admission/motion/lifecycle and min-cost-flow checks pass. The
  flow solver agrees with exhaustive enumeration on 96 small graphs.
- Four new Rust tests pass; the 105 ONNX test executions include repeated shared
  modules and are not 105 unique tests. Ten earlier freshness tests execute again.
- Twenty-eight fatal-input controls pass, including 27 before model access,
  together with two existing-output protections and model-free positive checks.
- Required core checks pass: 371 tests, 49 scenario replays and 245 pinned outputs.

The final deadline protocol intentionally remains held with only one fresh
post-deadline acquisition. Availability grants no vehicle-control permission.
Zero objects and an available camera result are separate states.

`recorded/` retains full repeated analysis/freshness reports, original acquisition
metadata, input hashes, dry inspections, exported actual measurements and
standalone replays. `controls/` retains synthetic PNG fixtures and rejected-input
diagnostics; special/sparse files are replaced by descriptors. `source/` and
`source-sha256.json` freeze the reviewed source state. `checks/` retains local
Rust/core verification logs. `protocol-verification.json` records preservation
of prior evidence and inspection/replay from this published location.
`files.json` indexes every packet file except itself.

Reproduce from the repository root with the pinned model and original image
cache present:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/onnx/camera_pipeline_check.py controls \
  --output artifacts/camera-pipeline-controls-new
python3 integrations/onnx/camera_pipeline_check.py run \
  --controls artifacts/camera-pipeline-controls-new \
  --output artifacts/camera-pipeline-recorded-new
```

The checker uses only Python's standard library. The
[contract and preparation example](../../docs/camera-pipeline.md) explain
clock ownership, budgets and errors. CI is configured, without an observed
remote run for this change. Code carries the [Apache-2.0 license](LICENSE-code.txt);
derived TUM measurements carry separate
[CC BY 4.0 attribution and source limitations](LICENSE-data.txt). Original
RGB/depth images, poses and weights are not redistributed. Subjective maturity
relative to Autoware/Apollo/openpilot remains **about 20%**, with 30% unmet.
