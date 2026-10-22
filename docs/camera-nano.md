# Fixed Nano model with parallel camera views

`rustdriving-camera-nano` runs the already pinned official YOLOX Nano model
through the original Rust Nano detector. It adds the existing full image plus
three square views, concurrent execution, global merge, ambiguity-preserving
vehicle consistency and measured-clock replay. It is a separate offline
candidate with distinct producer/source receipts and `control_permission:false`.

The original 3,659,407-byte Nano weights, 416×416 Triangle/raw-BGR preprocessing,
3,549×85 outputs, score 0.3 and classwise NMS IoU 0.45 are unchanged. No graph
lowering, quantization, model tuning or warmup is used. The old S report type
supplies the common Rust measurement structure; inference and decoding delegate
to the original Nano library. Current serial and parallel views agree in every
represented f32 bit, box, class, order, merge and decision. All sixteen full
views also agree with the independently executed original Nano command.

## Observed tradeoff

All images are the same previously viewed eight TUM indoor and eight BDD road
images. There is no fresh holdout or new identity-ground-truth evaluation.

| Measurement | Lowered S baseline | Nano candidate |
|---|---:|---:|
| First indoor image processing, first pair | 0.542206 s | 0.089433 s |
| First indoor image processing, second pair | 0.527273 s | 0.086683 s |
| Ratio of first-image means | 1× | 6.073× faster locally |
| Inferred / skipped source images per replay | 1 / 7 | 4 / 4 |
| Available results at checks, out of eight | 0 | 1 |
| Original 200 ms deadline flag, both trials | Fails | Passes |
| BDD TP / FP / FN | 78 / 31 / 60 (inherited) | 55 / 39 / 83 (measured) |
| BDD precision / recall | 71.56% / 56.52% | 58.51% / 39.86% |

The timing comparison uses actual ABBA S/Nano/Nano/S executions on one Linux
host and the exact original acquisition schedule, 0.2 s age bound and two-sample
recovery. Processing covers read/decode through the post-tracking probe; model
loading/preflight and final JSON publication are outside that clock. These are
two timed samples of the first image per executable, without sustained load,
peak-memory or other-hardware measurement. Later inferred Nano images take
80–149 ms in the retained traces.

The unchanged deadline flag measures late checks. It does not require every
image to be inferred or every result to be available. Nano skips four images
and supplies one available result out of eight; it has not established useful
continuous camera availability. Both original BDD precision/recall and per-class
quality gates still fail. The current original single-view Nano run scores
35 TP / 14 FP / 103 FN: four views gain 20 net matches but add 25 false positives.
All misses and false detections are preserved. The S accuracy figures are
inherited from the original report, with no new S accuracy scoring here.

## Reproduce

Cache the original inputs using the existing
[Nano model](../integrations/onnx/README.md),
[S/parallel mode](camera-parallel.md) and
[BDD diagnostic](road-camera-evaluation.md) instructions. From the root:

```sh
source scripts/env.sh
python -m pip install -r scripts/requirements-visual.txt
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-nano --bin rustdriving-camera-detect --bin rustdriving-camera-repeat2x
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python integrations/onnx/camera_nano_check.py controls --output artifacts/nano-example/controls
python integrations/onnx/camera_nano_check.py run \
  --controls artifacts/nano-example/controls --output artifacts/nano-example/validation
```

Every output path must be new. The auditor uses the original independent NumPy
merge/consistency oracle and BDD class-aware matcher. It checks original pixels,
all model/source identities, current serial/parallel measurement bits, original
Nano full views, complete clock arithmetic and exact standalone guard replay.
Input controls perform no valid-model inference. Timing/quality findings remain
separate from correct execution. The number of timed forwards depends on actual
host processing; complete image comparison always makes 208 forwards.

For a single image or the original measured replay:

```sh
integrations/onnx/target/release/rustdriving-camera-nano \
  --model artifacts/camera-model/yolox_nano.onnx --image IMAGE --output NEW.json
integrations/onnx/target/release/rustdriving-camera-nano \
  --model artifacts/camera-model/yolox_nano.onnx \
  --sequence assets/camera-sequence-v1/sequence.json --output NEW-REPLAY.json
```

`--serial-image`, `--inspect-sequence` and `--executable-receipt` are also
available. Source receipts contain 12 files for an individual detector, 31 for
image analysis and 40 for the full replay. The old serial report consumer rejects
this distinct Nano producer. There is no camera input in the vehicle controller.

## Retained evidence

[The evidence packet](../assets/camera-nano-v1/README.md) preserves 248 unique
actual model forwards, both auditor failures and their source, all original
timings, complete raw reports, original annotation hashes, controls and checks.
The two auditor corrections handle typed-f32 versus promoted-f64 JSON text;
they keep exact f32 bits and never allow a numerical tolerance. Only declared
f32 detection fields are canonicalized. Clock values and decisions remain exact.
Captured observations are re-audited without repeating model inference, under
an explicit checker-only resume contract; no timing samples were replaced.

```sh
python -B assets/camera-nano-v1/verify.py
python -m unittest discover -s integrations/onnx -p test_camera_nano.py -v
```

These commands require no model or original images. Original annotation metadata
and its BDD data notice accompany the reports; images, weights and native
executables are excluded. Subjective maturity versus mature driving OSS remains
about 20%, with the 30% waypoint unmet.
