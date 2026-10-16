# CPU operator profiling and guarded nearest-neighbor expansion

The optional `rustdriving-camera-repeat2x` executable replaces two expensive
generic Resize operators in the pinned YOLOX-S graph with ordinary tract
axis, broadcast and reshape operators. The original weights, input pixels,
four view rectangles, thresholds, decoder, merge and vehicle-consistency rule
are preserved. Original serial, parallel and native-build executables remain
separate, reproducible baselines.

## Implementation and limits

An operator profiler calls the unmodified tract evaluator, measures each
operation with `Instant`, and accounts for unattributed time separately.
The original four-view profile attributes about 75% of inference time to
matrix multiplication and 10% to the two Resize calls. Kernel names identify
the selected tract implementation; this is measurement, not a replacement
inference engine.

The lowering accepts only `Resize_165` and `Resize_187`, with static f32 NCHW
inputs `[1,256,20,20]` and `[1,128,40,40]`, explicit nearest/asymmetric/floor
semantics, exact constant `[1,1,2,2]` scales and absent or empty ROI. Other
attributes, sizes, shapes or graph variants are rejected. Expansion copies
each source value to its exact 2×2 output block, without arithmetic. A separate
coordinate oracle covers signed zero, subnormals, infinities and NaN payloads.
The model is still verified against its existing 35,858,002-byte SHA-256 pin.

The diagnostic executes an optimized and an independent original plan on
every view in alternating order. It compares all **714,000 output f32 values
per view bit for bit**, with little-endian f32 hashes, and checks every decoded
measurement and final hypothesis against an independently decoded ordinary
reference. Schema v2 publishes both sets of detections, merges and decisions.
JSON text from typed f32 fields and promoted
f64 fields is compared by represented f32 bits, with no tolerance. The failed
first auditor, its source and an adjacent-f32 rejection are retained.

The ordinary and lowered plans must agree exactly on the current CPU. The
parallel auditor also captures a current ordinary-parallel reference for every
image and requires exact agreement from both lowered repeats and every inferred
timed frame, excluding measured durations. Reference work follows the timed
trials. Only comparisons to historical reports use the existing
[bounded confidence rule](camera-ci.md); all historical boxes, classes, order,
decisions and policy remain exact. Accepted confidence differences are recorded
separately and cannot excuse a current-plan mismatch. Regression controls reject
a one-ULP current confidence change even with historical comparison enabled.

The parallel executable shares one lowered immutable plan across at most four
scoped workers. It uses the original measured-clock controller and freshness
guard. Its distinct producer identity and source receipt are rejected by the
old serial-report consumer. Diagnostic reports have `control_permission:false`.
There are no camera controls in the driving stack.

## Local results

All sixteen previously viewed images (eight TUM indoor, eight BDD dashcam)
run twice; no new holdout or accuracy scoring is claimed.

| Measurement | Original | Lowered | Observed change |
|---|---:|---:|---:|
| Mean serial four-view inference, 32 paired images | 2.180055 s | 2.026524 s | 7.04% less time |
| Parallel processing, first ABBA pair | 0.650533 s | 0.631123 s | Faster locally |
| Parallel processing, second ABBA pair | 0.674117 s | 0.599007 s | Faster locally |
| Parallel mean across two pairs | 0.662325 s | 0.615065 s | 7.14% less time / 1.077× |

Profiler observation overhead is included only on the lowered side of the
serial comparison; it is not a deadline measurement. Actual parallel processing
uses the existing [measured replay boundaries](camera-latency.md): read/decode,
four views, merge, consistency and tracking through the post-tracking probe.
Model loading and preflight precede the replay clock; final poll execution and
JSON publication are excluded. There is no inference warmup. Measurements are
from one Linux x86-64 host with four CPUs of quota and two timed samples per
executable. Load, peak memory and other hardware remain unmeasured. Neither
speedup nor a deadline outcome is required for checker execution success.
The earlier frozen v1 run measured a 15.35% parallel reduction; the v2 run above
measures 7.14%. Both runs remain available. These small samples do not establish
a stable speedup across loads or hardware.

All four original **200 ms deadline gates still fail**: each trace infers one
image, skips seven expired images, and has zero available results out of eight.
Independent standalone guard replay verifies every timing decision. Separately,
32 complete lowered parallel image reports preserve the original outputs.
Unchanged BDD quality remains **78 TP / 31 FP / 60 FN**, precision 71.56%,
recall 56.52%; aggregate and per-class quality gates still fail. These results
do not establish new object-identity accuracy, live-camera timing or driving
integration. Subjective OSS-relative maturity remains about 20%, with 30% unmet.

## Reproduce

First cache the pinned model, original images and baseline executables using
the [parallel-mode instructions](camera-parallel.md). Then, from the root:

```sh
source scripts/env.sh
cargo build --release --locked --offline --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-profile --bin rustdriving-camera-repeat2x
python3 integrations/onnx/camera_operator_check.py controls \
  --output artifacts/operator-example/controls
python3 integrations/onnx/camera_operator_check.py run \
  --controls artifacts/operator-example/controls --output artifacts/operator-example/validation
python3 integrations/onnx/camera_repeat_check.py controls \
  --output artifacts/repeat2x-example/controls
python3 integrations/onnx/camera_repeat_check.py run \
  --controls artifacts/repeat2x-example/controls --output artifacts/repeat2x-example/validation
```

All output paths must be new. Operator validation makes 264 model forwards;
parallel validation makes 208, including 64 current-reference forwards. Input
controls load no valid model. Both Python auditors use only the standard
library. Current-host comparisons are strict; historical confidence differences
are separately bounded and recorded. Other differences still fail validation.

For a single image, run the profiler with `--repeat2x` and optionally
`--reference-first`, or use the lowered parallel executable:

```sh
integrations/onnx/target/release/rustdriving-camera-repeat2x \
  --model artifacts/camera-model-s/yolox_s.onnx --image IMAGE --output NEW.json
integrations/onnx/target/release/rustdriving-camera-repeat2x \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --sequence assets/camera-sequence-v1/sequence.json --output NEW-SEQUENCE.json
```

`--inspect-sequence FILE` and `--executable-receipt` work without a model.
Compiled-source receipts contain 27 files for the final profiler, 31 for
lowered parallel images, and 40 for the complete lowered replay executable.
The first exploratory profiler predates the lowering and records 26 files;
its original source is archived separately.

## Evidence and CI status

[The first evidence packet](../assets/camera-operators-v1/README.md) preserves the
first profile and auditor failure, final raw reports, source and executable
receipts, controls, logs and independent audits. Large JSON reports are
deterministically gzipped with original-byte hashes; images, weights and native
binaries are not included. Optimization work made 424 actual model forwards,
including both exploratory eight-forward runs. Its source and reports remain
frozen. [The v2 packet](../assets/camera-operators-v2/README.md) records the current
comparison protocol and replays v1 against its own archived source in a
temporary tree. It never rewrites the prior evidence to match the current code.

The prior merged [CI run](https://github.com/rsasaki0109/rust_drive/actions/runs/38911860666)
failed in the original pipeline's exact historical freshness comparison. A
separate real AVX2/FMA reproduction in [the CI packet](../assets/camera-ci-v1/README.md)
isolated confidence drift while retaining exact boxes and decisions; the
original remote numeric leaf remains unconfirmed. The checker repair in
[PR #21](https://github.com/rsasaki0109/rust_drive/pull/21) passed all ten jobs in
[the full CI run](https://github.com/rsasaki0109/rust_drive/actions/runs/38929172191).
No timing, accuracy or freshness gate changed. This optimization adds its own
current-host exact checks to the research CI; that new remote run is distinct
from the passing checker-repair run.

Verify both optimization packets without images, weights or inference:

```sh
python3 -B assets/camera-operators-v2/verify.py
python3 -m unittest discover -s integrations/onnx -p test_camera_operators.py -v
```
