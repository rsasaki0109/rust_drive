# Measured camera replay evidence v1

This packet records `rustdriving-camera-latency`, an offline serial replay of
original image acquisition offsets against actual host monotonic time. It
retains the unchanged four-view YOLOX-S analyzer, tracker, freshness policy,
prior executables and lockfiles.

**The observed 200 ms deadline gate fails in both runs.** First-image processing
takes 2.309618 s and 2.273303 s. Each run infers one image, skips seven expired
images and exposes zero available results across the fixed eight-frame input.
There are eight actual model forwards across both runs (four per inferred
image), with no accepted real tracking backend update. These timing failures
are retained without changing the prior 0.2-second policy.

The same previously viewed TUM Freiburg3 sitting_xyz RGB indices 102–109 span
1341845824.165751–1341845824.398039 s (0.232288 s). Original clock text, image
hashes and represented acquisition intervals are preserved. Release scheduling
uses existing files; this does not measure camera transport or live sensing.
Preflight warms file caches and model loading precedes schedule start; startup
times are recorded separately. There is no inference warm-up. The reported
age includes queue time, re-read/decode/analysis and the sample tracker update,
through the checkpoint before the final poll. Final poll execution and audit
JSON serialization/publication are excluded.

- Eight new Rust tests pass, covering timely/late frames, skipping, post-tracking
  expiry, recovery, probe isolation, state atomicity and source schedule bounds.
  The binary's 34 test executions include 26 shared existing tests.
- Twenty-one fatal-input controls pass (20 before model access), with two
  existing-output protections and a model-free non-image trace.
- The independent auditor reconstructs clocks, ages, start admission and gate
  counts, checks measured model duration against full processing and an external
  process timer, and reproduces all guard results exactly in another process.
- Actual model output matches previous immutable real inference, excluding only
  measured durations. No association result or object-identity score is claimed
  for these measured failures, where no detection reaches the tracking backend.
- Required core checks pass: 371 tests, 49 scenario replays, 245 unchanged outputs.

`measured/` contains both raw measurements, original metadata, source hashes
and guard replays. Timing identity is not required across repeated wall-clock
runs. `controls/` retains synthetic fixtures and rejection diagnostics, with
special/sparse files represented by descriptors. `source/` and
`source-sha256.json` preserve the reviewed code/docs. `checks/` retains local
validation logs. `environment.json` records nonsecret host information observed
after measurement; host load and resource reservation were not measured.
`protocol-verification.json` records prior evidence preservation and published
relative-path inspection/guard replay. `files.json` indexes all files except itself.

Reproduce from the repository root with the original image cache and pinned
model present:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/onnx/camera_latency_check.py controls \
  --output artifacts/camera-latency-controls-new
python3 integrations/onnx/camera_latency_check.py run \
  --controls artifacts/camera-latency-controls-new \
  --output artifacts/camera-latency-measured-new
```

The checker uses only Python's standard library. See the
[clock/measurement contract](../../docs/camera-latency.md). CI preserves observed
deadline results and checks the audit without requiring a hardware-dependent
timing pass; no remote run has been observed for this change. Two local runs
do not establish a latency distribution, real-time guarantee, automotive
performance or driving safety. Object-identity accuracy remains unmeasured.
The [Apache-2.0 code license](LICENSE-code.txt) and separate
[TUM CC BY 4.0 attribution](LICENSE-data.txt) apply. Original RGB/depth images,
poses and weights are not redistributed. Subjective maturity relative to
Autoware/Apollo/openpilot remains **about 20%**, with 30% unmet.
