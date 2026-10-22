# Historical confidence and exact current-host camera replay

The merged research CI failed while comparing the original camera pipeline to
saved freshness outputs. A controlled local experiment reproduces that failure
by selecting tract's ordinary AVX2/FMA fallback instead of AVX512. The same
weights, images, detector/controller source and acquisition schedule produce
slightly different confidences. The first observed change is
`0.9397782683372498` → `0.9397783279418945` (one f32 ULP).

Across eight healthy TUM images, 47 scalar fields differ: 40 copies of detector
confidence and seven copies of last-observed confidence. The maximum difference
is 1.1920929e-7. Every coordinate, source/delivery time, class, identity,
decision, lifecycle and projection remains exact. A separate system-libm
dispatch probe produces no differences. The fallback probe is an isolated
diagnostic build with two documented library dispatch changes, not a production
runtime or a speed comparison.

The original failed remote artifact's first numeric difference remains
unavailable because its download destination is blocked. A separate runner
passed the old exact comparison. Those observations and the controlled fallback
reproduction support a portability repair; they do not establish a physical
camera fault or a perception improvement.

## Comparison contract

`integrations/onnx/camera_history.py` applies the **existing 1e-5 absolute f32
historical bound** from `bdd_tiles_check.py` only to `confidence` and
`last_observed_confidence`. It records every accepted difference, original
numbers, little-endian f32 bits and absolute difference. Nonfinite/out-of-range
confidences and crossings of the original 0.3 score threshold are rejected.
All other fields, keys, list lengths/order and value types remain exact.

This changes historical confidence compatibility, not the detector threshold,
NMS, accuracy scoring or the 200 ms freshness policy. The native detector,
parallel worker implementation, tracking and freshness sources are unchanged.
The existing legacy report-admission source pins also remain unchanged.

Current-host comparisons continue to require exact results:

- The original pipeline runs each protocol twice and checks all non-timing
  fields, then independently replays its guard and tracking outputs exactly.
- The parallel check now also executes the original serial image tool on each
  of the 16 images. Both parallel repeats must match that current serial output
  exactly, including every confidence, after the existing producer metadata and
  timing projection.
- The native CPU-build check similarly adds a current normal-build reference
  for each image. Both native repeats must match it exactly, with only measured
  durations excluded.
- Measured replay independently reconstructs every clock and replays the actual
  current-host guard output exactly. Historical compatibility never substitutes
  for that equality.

The serial-reference binaries and the complete auditor source closure are
pinned before new inference. A change to an image, weight, receipt, identity,
coordinate or policy is still fatal. Historical comparisons are explicitly
reported separately from exact comparisons; old exact-parity flags are true
only when exact equality actually holds.

## Reproduce

With the existing pinned models, image caches and release binaries prepared:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 -m unittest discover -s integrations/onnx -p test_camera_history.py -v
python3 integrations/onnx/camera_pipeline_check.py controls --output artifacts/camera-ci-example/pipeline-controls
python3 integrations/onnx/camera_pipeline_check.py run \
  --controls artifacts/camera-ci-example/pipeline-controls --output artifacts/camera-ci-example/pipeline
python3 integrations/onnx/camera_parallel_check.py controls --output artifacts/camera-ci-example/parallel-controls
python3 integrations/onnx/camera_parallel_check.py run \
  --controls artifacts/camera-ci-example/parallel-controls --output artifacts/camera-ci-example/parallel
```

All output paths must be new. The 12 regression tests include the complete
recorded fallback trace, exact native guard replay, one-ULP clock/coordinate
corruptions, threshold crossing, excessive confidence drift and malformed
values. Native replay requires the independently built tracking binary; CI
builds it before running the tests. The other controls continue to exercise
fatal inputs, staging and output preservation without valid model inference.

| Local validation | Actual model forwards |
|---|---:|
| System-libm dispatch probe | 32 |
| Isolated AVX2/FMA fallback probe | 32 |
| Original pipeline: four protocols twice | 216 |
| Measured serial replay | 8 |
| Serial/parallel comparison with current references | 208 |
| Normal/native-build comparison with current references | 208 |
| Total | 704 |

The additional references are correctness work and are excluded from benchmark
timings. No new performance improvement is claimed. The original 200 ms gates
and inherited accuracy failures remain in the reports.

[The evidence packet](../assets/camera-ci-v1/README.md) retains raw observations,
the dispatch probe design/build failure and repair, source snapshots, accepted
difference ledgers, controls and verification logs. Large reports are losslessly
gzipped. Original images, weights and executables are excluded. The earlier
evidence packets remain byte-for-byte intact. This maintenance does not add
driving capability or fresh accuracy evidence; subjective maturity versus
Autoware, Apollo and openpilot remains about 20%, with 30% unmet.
