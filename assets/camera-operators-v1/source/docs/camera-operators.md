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
measurement and final hypothesis. JSON text from typed f32 fields and promoted
f64 fields is compared by represented f32 bits, with no tolerance. The failed
first auditor, its source and an adjacent-f32 rejection are retained.

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
| Mean serial four-view inference, 32 paired images | 2.200343 s | 2.035672 s | 7.48% less time |
| Parallel processing, first ABBA pair | 0.706520 s | 0.585206 s | Faster locally |
| Parallel processing, second ABBA pair | 0.640254 s | 0.554865 s | Faster locally |
| Parallel mean across two pairs | 0.673387 s | 0.570035 s | 15.35% less time / 1.181× |

Profiler observation overhead is included only on the lowered side of the
serial comparison; it is not a deadline measurement. Actual parallel processing
uses the existing [measured replay boundaries](camera-latency.md): read/decode,
four views, merge, consistency and tracking through the post-tracking probe.
Model loading and preflight precede the replay clock; final poll execution and
JSON publication are excluded. There is no inference warmup. Measurements are
from one Linux x86-64 host with four CPUs of quota and two timed samples per
executable. Load, peak memory and other hardware remain unmeasured. Neither
speedup nor a deadline outcome is required for checker execution success.

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
parallel validation makes 144. Input controls load no valid model. Both Python
auditors use only the standard library. Numerical comparisons to the saved
historical detections remain strict; other CPU kernels may fail those checks.
Such failures must be investigated and are not silently accepted.

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

[The evidence packet](../assets/camera-operators-v1/README.md) preserves the
first profile and auditor failure, final raw reports, source and executable
receipts, controls, logs and independent audits. Large JSON reports are
deterministically gzipped with original-byte hashes; images, weights and native
binaries are not included. Optimization work made 424 actual model forwards,
including both exploratory eight-forward runs. The separate local CI
investigation made 432 additional forwards (216 before and 216 after the
checker-only diagnostic change).

The prior merged [CI run](https://github.com/rsasaki0109/rust_drive/actions/runs/38911860666)
failed in the original pipeline's exact historical freshness comparison before
reaching the new work. The same unchanged pipeline passed all four cases
locally. The exact remote numeric difference remains unconfirmed because the
artifact download destination is blocked in this environment. The checker now
prints the first differing JSON path and values while retaining exact equality.
No timing, accuracy or freshness gate was relaxed. New optimization checks run
independently of earlier research-step failures; remote success is not claimed.
