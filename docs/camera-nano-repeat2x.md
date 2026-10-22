# Guarded Nano Resize lowering

The optional `rustdriving-camera-nano-repeat2x` command replaces only two
nearest-neighbor expansions in the original pinned Nano model. It preserves
weights, 416×416 Triangle/raw-BGR preprocessing, decoding, thresholds, fixed
views, global merge, vehicle consistency and both existing clock controllers.
Source receipts are cached once during model loading, with initialization time
included in model-load/startup accounting. Original commands and evidence remain
unchanged. There is no vehicle-camera input or control permission.

## Scope and correctness

The pass requires exactly `Resize_201` with input `[1,128,13,13]` and
`Resize_226` with input `[1,64,26,26]`. Both must be static f32, use explicit
nearest/asymmetric/floor semantics, exact `[1,1,2,2]` scales and absent/empty ROI.
It uses standard axis insertion, broadcast and reshape, without a custom
runtime, quantization or changed weights. The source audit checks that the
existing S pass differs only in these Nano node/shape pins.

An initial 13-forward diagnosis checks four operator observations against
ordinary output, four original Nano serial views and one original Nano full
view. Generic Resize consumes about 18% of observed operator time. The observer
is the existing implementation; its sole source change adapts a 640-pixel unit
fixture to Nano's 416 pixels.

The subsequent fixed comparison makes 572 actual forwards. All sixteen already
viewed TUM/BDD images run twice in alternating observed/reference order. Across
128 view pairs, every one of 301,665 raw f32 values agrees bit for bit:
38,613,120 exact value comparisons. Current original Nano serial views and two
parallel repeats also preserve every represented measurement bit, merge and
decision. Original independent postprocessing checks remain in use.

## Timing and availability: mixed result, no default adoption

All clock trials retain the original eight indoor acquisitions, 200 ms age
bound and two-sample recovery. Each scheduler runs baseline/optimized/optimized/
baseline before current-reference and profile work; no warmup is used.

| Scheduler and trial | First processing | Available / original images | Inferred but late |
|---|---:|---:|---:|
| FIFO baseline first | 87.06 ms | 1 / 8 | 1 |
| FIFO optimized first | 78.41 ms | 1 / 8 | 0 |
| FIFO optimized repeat | 94.60 ms | 1 / 8 | 0 |
| FIFO baseline repeat | 99.01 ms | 1 / 8 | 1 |
| Latest baseline first | 81.94 ms | 3 / 8 | 0 |
| Latest optimized first | 154.32 ms | 0 / 8 | 0 |
| Latest optimized repeat | 117.46 ms | 1 / 8 | 0 |
| Latest baseline repeat | 107.70 ms | 1 / 8 | 0 |

FIFO's first-image mean is 7.02% shorter, while latest-ready's mean is 43.31%
longer. The latest path also loses availability. Eight independent native guard
replays verify every timestamp and output. Dropped acquisitions remain in the
source denominator and count unavailable. Both original policies remain exact;
results acquired before a fault barrier cannot recover tracking.

The separate 32 serial operator/reference pairs favor the lowered plan in 31
pairs, with 12.89% less aggregate forward time (median pair reduction 12.21%).
This compares an instrumented serial optimized forward with an ordinary serial
reference; it is not parallel processing latency or continuous availability.
All regressions and original samples are retained. These limited local CPU
measurements do not establish a consistent hardware-independent speedup.

No fresh BDD accuracy scoring is performed. The inherited detector still has
55 TP / 39 FP / 83 FN and fails the original quality gates. There is no new
holdout, identity score or driving integration. This path stays opt-in; the
default remains unchanged, and subjective maturity stays about 20% versus mature
driving OSS, with 30% unmet.

## Reproduce

Cache the original inputs using the [Nano instructions](camera-nano.md).
From the repository root:

```sh
source scripts/env.sh
python -m pip install -r scripts/requirements-visual.txt
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-nano --bin rustdriving-camera-latest \
  --bin rustdriving-camera-nano-profile --bin rustdriving-camera-nano-repeat2x
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python integrations/onnx/camera_nano_repeat_check.py controls --output artifacts/nano-repeat-example/controls
python integrations/onnx/camera_nano_repeat_check.py run \
  --controls artifacts/nano-repeat-example/controls --output artifacts/nano-repeat-example/validation
```

Every output must be new. A direct image or replay uses `--model FILE` with
`--image FILE`, `--sequence FILE` or `--latest-sequence FILE`, followed by
`--output NEW`. `--profile-image FILE [--reference-first]` makes four observed
and four independent ordinary forwards. `--inspect-sequence FILE` and
`--executable-receipt` require no model and retain the complete 55-file native
source closure. The standalone original observer is
`rustdriving-camera-nano-profile --model FILE --image FILE [--reference-first]
--output NEW`.

The [frozen packet](../assets/camera-nano-repeat2x-v1/README.md) preserves the
initial diagnosis and all 585 unique forwards. Its static verification and
regression tests need no model or original image:

```sh
python -B assets/camera-nano-repeat2x-v1/verify.py
python -m unittest discover -s integrations/onnx -p test_camera_nano_repeat.py -v
```
