# Measured camera processing and deadline admission

`rustdriving-camera-latency` replays original recorded image acquisition
intervals against **actual host monotonic processing time**. It connects the
unchanged analyzer, tracker and freshness guard, skips images already too old
at processing start, and checks freshness again after detection and tracking.
The existing supplied-clock pipeline remains reproducible.

The first local CPU measurement **fails the existing 200 ms age limit**:

| Run | First image read/analysis/probe | Age limit | Inferred / skipped images | Available results | Deadline gate |
|---|---:|---:|---:|---:|---|
| First | 2.309618 s | 0.200 s | 1 / 7 | 0 / 8 | FAIL |
| Repeat | 2.273303 s | 0.200 s | 1 / 7 | 0 / 8 | FAIL |

Each inferred image performs four actual model forwards. The remaining seven
images are already beyond their age limit when processing can reach them, so
they skip inference. All eight image deadlines are missed in each run. No
fresh detection enters the tracker and no object IDs are generated. The
eight actual model forwards across these two runs are not eight inferred RGB
images. These failures are retained without relaxing the prior policy.

## Run a measurement

Use the cached TUM Freiburg3 `sitting_xyz` recording and the
[pinned YOLOX-S model](../integrations/onnx/README.md). The existing optional
image fetch is `python3 scripts/fetch-visual-datasets.py tum-fr3-sitting-visual`.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-latency
mkdir -p artifacts/camera-latency-example
python3 integrations/onnx/prepare_sequence.py \
  --dataset data/tum-fr3-sitting-visual --first 102 --count 8 \
  --camera-id tum-fr3-sitting-rgb \
  --output artifacts/camera-latency-example/input.json
integrations/onnx/target/release/rustdriving-camera-latency \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --sequence artifacts/camera-latency-example/input.json \
  --output artifacts/camera-latency-example/result.json
```

Use fresh output paths each time. Input uses the existing
[image-sequence schema](camera-sequence.md): `schema_version:1`, camera identity,
fixed dimensions, optional tracker `config` and 1–32 events with original
`timestamp_s` decimal strings. Event kinds are `image`, `missing` and `failed`.
Paths are relative to the sequence file. The total source interval is bounded
to **2 seconds**. Policy stays at the existing default `max_age_s:0.2` and
`recovery_samples:2`, fixed before measurement.

`--inspect-sequence FILE --output NEW` verifies the input and emits original
clock text/image identities and release offsets without loading a model.
`--executable-receipt --output NEW` records source/executable hashes without
reading images or weights.

## Clock, scheduling and measurement boundary

Every image is hash/geometry checked before model access. One model plan is
loaded for an image-bearing trace. Manifest/preflight wall time, model-load
wall time and total startup are recorded separately. The measured schedule
starts after those stages. Preflight warms the file cache; there is **no
inference warm-up**. This is a local processing diagnostic with those conditions.

Original decimal source times are parsed with Rust's correctly rounded parser.
Release offsets subtract the first parsed acquisition time, preserving the
original represented intervals and timestamp text. `std::time::Instant` starts
the host clock at that first scheduled release. Serial FIFO processing waits
until each original acquisition offset when necessary. The inputs are existing
files; release scheduling does not emulate or measure camera transport.

For every image the implementation:

1. Observes the real processing-start clock and probes a clone of the unchanged
   guard. Probes consume no IDs, image-cache entries or recovery samples.
2. Re-reads/verifies and analyzes the image only when that probe accepts it.
   An already stale image supplies no invented detections.
3. Observes the actual completion clock and submits the actual detector payload
   to the real guard. An image that became late during analysis is excluded.
4. Observes the clock after this tracking/guard update and performs a final
   poll, so a deadline crossed during tracking also withholds output.

The reported age at this last checkpoint includes queue age, re-read/decode,
analysis, conversion and the sample's tracker/guard update. **Execution of the
last poll and final audit JSON serialization/publication are excluded.** The
last poll's readiness is the availability result for the recorded checkpoint.
This batch report supplies no live result stream or vehicle-control permission.

Checkpoints must advance by at least 1 microsecond. If needed, the process
yields until the host clock actually advances; it never manufactures a larger
timestamp. Detector/checkpoint errors leave the complete previous guard state
unchanged. Explicit failures retire IDs; fresh post-fault recovery and each
object's age limits follow the unchanged [freshness contract](camera-freshness.md).

## Output and validation

`yolox_s_measured_serial_replay_v1` records original source text/offsets, source
and executable receipts, each processing checkpoint, optional full analyzer
report and both guard dispatches/results. `summary` separates inferred/skipped
images, late results, available results and observed `deadline_gate_pass`.
The gate is null for a trace with no images. Execution success and deadline
success are separate: exit 0 can carry `deadline_gate_pass:false`.

Existing regular-file/image budgets and 512 KiB sequence / 4 MiB report limits
apply. The unchanged `AtomicReport` reserves fresh staging before source reads,
bounds serialization and publishes complete synced bytes without replacement.
Handled errors exit 2 and remove staging. Abrupt termination can leave hidden
staging; no power-loss durability or cross-platform claim is made.

```sh
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/onnx/camera_latency_check.py controls \
  --output artifacts/camera-latency-controls-new
python3 integrations/onnx/camera_latency_check.py run \
  --controls artifacts/camera-latency-controls-new \
  --output artifacts/camera-latency-measured-new
```

Eight new Rust tests cover timely delivery, analysis/track-update deadline
crossings, expired-work skipping, recovery, probe isolation, error atomicity,
non-image events and original epoch offsets/window limits. Shared tests execute
again; the 34 test executions for this binary are not 34 new tests.
Twenty-one fatal-input controls pass (20 before model loading), with two
existing-output protections and a model-free non-image trace.

The standard-library Python auditor reconstructs checkpoint ages, start-time
admission and deadline counts independently. It checks actual model proposals
against immutable saved inference, checks full model time against measured
processing, bounds reported duration with an external process timer and
reproduces every guard result in a separate process. No tracking backend update
occurs in the measured failures; independent lifecycle/assignment machinery
therefore sees zero real backend frames. Synthetic timely/recovery tests and
the unchanged previous tracking evidence cover those branches separately.

The [evidence packet](../assets/camera-latency-v1/files.json) retains both raw
measurements, all failures and source pins. The CPU research CI job reruns the
auditor and preserves timing outcomes; it requires correct execution/validation
without asserting a hardware-dependent deadline pass. No remote run has been
observed for this change.

The same previously viewed indoor slice spans only 0.232288 s, RGB indices
102–109. There are no live camera, sensor transport, automotive host or
object-identity labels in this measurement. Times vary with hardware/load and
these two runs do not establish a latency distribution or real-time guarantee.
Derived measurements retain [TUM CC BY 4.0 attribution](../assets/camera-latency-v1/LICENSE-data.txt);
original RGB/depth images, poses and weights are not redistributed. Detection
quality and vehicle integration are unchanged. OSS-relative maturity remains
**about 20%**, with the 30% waypoint unmet.
