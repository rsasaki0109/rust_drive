# Images through detection, tracking and freshness

`rustdriving-camera-pipeline` runs the unchanged YOLOX-S image analyzer,
image-space tracker and two-clock freshness guard in **one offline command**.
It uses one CPU model plan for admitted images, retains complete detector and
tracking lineage, and skips inference for images excluded by clock/hash
admission. The existing separate tools and their pinned sources are unchanged.

The freshness clock is **caller-supplied replay time**. Delivery times and
faults in the verified examples are injected. Actual CPU inference duration
does not advance replay time. This is neither live camera ingestion nor a
measurement of end-to-end latency, automotive availability or control safety.

## Run an existing recording

The example uses the locally cached TUM Freiburg3 `sitting_xyz` RGB recording
and the [pinned YOLOX-S model](../integrations/onnx/README.md). If the image cache
is absent, the existing opt-in fetch command is
`python3 scripts/fetch-visual-datasets.py tum-fr3-sitting-visual`.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-pipeline
mkdir -p artifacts/camera-pipeline-example
python3 integrations/onnx/prepare_sequence.py \
  --dataset data/tum-fr3-sitting-visual --first 102 --count 8 \
  --camera-id tum-fr3-sitting-rgb \
  --output artifacts/camera-pipeline-example/images.json
python3 integrations/onnx/prepare_camera_pipeline.py \
  --sequence artifacts/camera-pipeline-example/images.json \
  --replay-lag-s 0.005 \
  --output artifacts/camera-pipeline-example/input.json
integrations/onnx/target/release/rustdriving-camera-pipeline \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --sequence artifacts/camera-pipeline-example/input.json \
  --output artifacts/camera-pipeline-example/result.json
```

Choose fresh output paths on subsequent runs. The existing preparation helper
preserves original RGB timestamp text, file hashes and consecutive indices.
The new helper adds an authored 5 ms delivery lag using decimal arithmetic;
it reads no images, weights, depth, poses or labels. Its lag is not measured
execution time. Original missing events become polls and explicit failures
remain failures. Both helpers use Python's standard library.

Optional `--inspect-sequence FILE --output NEW` verifies all image bytes,
decoded geometry, both clocks and dry admission without model access. It
emits no simulated object measurements or runtime tracking output.
`--executable-receipt --output NEW` records executable/source hashes without
reading a model or images.

## Input and behavior

The version-1 JSON sequence has `camera_id`, fixed `width`/`height`, optional
tracker `config`, optional freshness `policy` and 1–32 `events`. The existing
[freshness policy and clock contract](camera-freshness.md) apply. Both times
are **decimal strings**, parsed with Rust's correctly rounded float parser.
An image event has this form:

```json
{
  "now_s": "1341845824.170751",
  "input": {
    "kind": "image",
    "acquired_s": "1341845824.165751",
    "path": "frame.png",
    "sha256": "<64 lowercase hex digits>"
  }
}
```

`kind:"tick"` and `kind:"failed"` need only `now_s`. Paths are relative to the
sequence file unless absolute. Acquisition and delivery seconds must share a
caller-managed time domain. Nonfinite/malformed clock text is a batch error;
finite but unusable acquisition times produce explicit rejection decisions.
Delivery time must advance by at least 1 microsecond in [0, 1e12].

Preflight verifies **every supplied image**, including those whose admission
is rejected. It probes clocks and image-reuse admission with empty payloads;
these probes emit no object detections. Only admitted image events load/use
the model. An all-rejected-image trace still verifies its images but opens no
model and performs no inference. Admitted images are re-read and re-verified
before inference, keeping one decoded image at a time.

Each admitted image runs the unchanged four-view analyzer. Its original f32
measurements enter the unchanged tracker/guard as f64; no measurements are
invented for discarded images. Runtime admission must agree with preflight,
otherwise the batch fails without publication. Polls project existing
hypotheses without refreshing their observations or backend acquisition time.
Expiry/failure clears active IDs and holds output until enough fresh post-fault
acquisitions arrive. Healthy zero-detection images can restore camera-result
availability, independently of object confirmation.

`ready` is camera-result availability and grants no vehicle-control permission.
Object boxes/velocities remain pixels and pixels/second, with unchanged
observation time/confidence and an individual age bound. No metric depth,
ego-motion compensation, calibrated uncertainty or planning integration is added.

## Output, bounds and errors

`yolox_s_camera_pipeline_v1` output records the sequence and compiled-source
hashes, effective configuration/policy, `freshness_clock:"caller_supplied_replay"`
and `control_permission:false`. Execution counters distinguish preflight image
reads, inference reads, skipped image events, actual forwards and model loads.
Each event preserves original clock text and image identity, optional complete
`analysis`, an `inference_skipped` flag and the full `freshness` result with all
backend updates. A tick/failure has no analysis and is not a skipped image.

Sequence input is bounded to 512 KiB / 32 events. Existing image budgets are
16 MiB encoded and 16 million decoded pixels; regular non-symlink inputs are
required. Existing `AtomicReport` reserves fresh sibling staging before source
reads and limits serialization to **4 MiB**. A long valid sequence can exceed
the aggregate report budget. Only a complete synced report is published without
replacing a file or dangling symlink. Handled errors exit 2 and remove staging.
Abrupt termination can leave staging; Linux checks do not establish power-loss
durability or cross-platform behavior.

Semantic image exclusions are retained as results and skip inference.
Malformed input, image corruption/provenance errors or model failures abort
the batch. Exit 0 means the trace was processed, including traces ending held.

## Verification and limits

```sh
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/onnx/camera_pipeline_check.py controls \
  --output artifacts/camera-pipeline-controls-new
python3 integrations/onnx/camera_pipeline_check.py run \
  --controls artifacts/camera-pipeline-controls-new \
  --output artifacts/camera-pipeline-recorded-new
```

The fixed real-image slice remains the previously viewed TUM RGB indices
102–109, 1341845824.165751–1341845824.398039 s (**0.232288 s**).
The four existing freshness protocols are frozen before new inference: healthy,
delayed after a poll, explicit failure/recovery and deadline expiry. Every
protocol runs twice, with one model plan per run and **216 actual model forwards**
in total. Two excluded image deliveries per four cases skip inference. The
deadline example finishes held with only one post-deadline acquisition.

Historical detector/freshness comparisons allow the existing absolute f32
confidence bound of 1e-5 only for `confidence` and `last_observed_confidence`.
Coordinates, clocks, identities, decisions, policy, order and all other fields
remain exact, and crossing the original score threshold is rejected. Every
accepted confidence difference is recorded. Current-host repeated executions
remain exact after excluding only inference/load durations. Exported actual
analysis reports reproduce the standalone guard and tracker exactly. Independent admission,
motion/lifecycle and min-cost-flow checks verify all backend updates; the flow
solver agrees with exhaustive enumeration on 96 small graphs.

Four new Rust tests cover both epoch clocks, model-free polls/failure, invalid
delivery/policy preflight and incomplete-preflight rejection. Existing shared
tests execute again, including the 10 freshness tests; these are not new tests.
Twenty-eight fatal-input controls pass (27 before model loading and one bad
model-size case), alongside existing-output protection, repeated model-free
all-rejected execution and dry reused-image/out-of-order checks. The checker
uses only Python's standard library and existing synthetic PNG fixtures.
The independent `Camera pipeline contract` workflow reruns these checks without
waiting for unrelated research evaluations. It also tests a recorded AVX2/FMA
confidence-drift fixture and exact replay in the original native guard.
[CPU portability repair, evidence and CI status](camera-ci.md).

The [evidence packet](../assets/camera-pipeline-v1/files.json) retains full
analysis/state outputs, original clock metadata, replays, rejected controls,
source pins and local validation logs. Images, depth, poses and weights are
not redistributed. Derived TUM data carry
[CC BY 4.0 attribution and source limitations](../assets/camera-pipeline-v1/LICENSE-data.txt).
Independent object-identity accuracy, actual camera failure rates and sustained
automotive availability remain unmeasured. Detection quality is unchanged.
Subjective maturity relative to mature driving OSS remains **about 20%**;
the 30% waypoint is unmet.
