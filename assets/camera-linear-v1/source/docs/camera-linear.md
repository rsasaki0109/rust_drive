# YOLOX preprocessing compatibility and fresh road-image diagnostics

The optional `rustdriving-camera-linear` command implements YOLOX's raw BGR
letterbox with OpenCV-style uint8 `INTER_LINEAR` interpolation in safe Rust.
The previous `image::Triangle` resize smooths downsampled images differently.
Compatibility is useful, but the new preprocessing does **not** meet the
accuracy gates and is **not adopted as a default**.

| Sample / preprocessing | TP | FP | FN | Precision | Recall |
|---|---:|---:|---:|---:|---:|
| Previously viewed 8 / Triangle | 55 | 39 | 83 | 58.51% | 39.86% |
| Previously viewed 8 / Linear | 56 | 36 | 82 | 60.87% | 40.58% |
| Fresh 16 / Triangle | 99 | 60 | 169 | 62.26% | 36.94% |
| Fresh 16 / Linear | 101 | 62 | 167 | 61.96% | 37.69% |

The fresh sample contains 268 mapped reference objects. Linear gains seven
reference identities and loses five. Car matches rise from 77 to 81, while
person/rider matches fall from 14 to 13 and traffic-light matches fall from
three to two. Person/rider false positives rise from four to eleven. These
regressions remain visible in the complete per-image and per-class reports.
Precision ≥0.7, recall ≥0.6 and recall ≥0.5 in every present class all fail.
The predeclared adoption rule additionally requires no precision/recall
regression on either sample; fresh precision regresses. No threshold, label,
size filter, confidence policy or view inventory is changed to improve scores.

The 16 images were selected from the first lexicographic original BDD `val`
JPEG/annotation pairs in a complete pinned Git tree, excluding all cached JPEG
stems and the eight previous BDD IDs. Selection and evaluation rules were frozen
before acquiring or decoding the selected JPEGs. All 16 are retained. This is a
small convenience sample from a public mirror, with no representative-domain
or upstream-training non-overlap guarantee. It is now viewed development data;
the result is fixed-IoU diagnostics, **not BDD mAP**.

The implementation follows [YOLOX 0.3.0 preprocessing](https://github.com/Megvii-BaseDetection/YOLOX/blob/0.3.0/yolox/data/data_augment.py)
and the generic uint8 path in [OpenCV 4.13.0](https://github.com/opencv/opencv/blob/4.13.0/modules/imgproc/src/resize.cpp).
It uses half-pixel coordinates, 11-bit interpolation coefficients, ties-to-even
coefficient rounding, OpenCV's intermediate integer rounding, replicated
borders, identity-copy and exact 2× area special cases. Upstream f64 ratio and
truncated resize dimensions precede top-left 114 padding and raw BGR NCHW f32
conversion. Decoder coordinates continue to use the original f32 decoder;
this is a preprocessing compatibility claim, not full upstream inference parity.
The old comparison mode preserves its original f32 ratio and minimum-one-pixel
rule. Empty canonical resize dimensions fail, matching upstream rejection.

An independently installed `opencv-python-headless==4.13.0.92` reference uses
OpenCV 4.13.0 with IPP, OpenCL and optimized dispatch disabled. Every generated
input tensor matches: 48 unique seeded cases cover portrait/landscape, fractional
resize, single-pixel axes, identity, 2× area, shrink and enlargement, totaling
19,260,999 f32 values. All 24 actual images also pass full-image preprocessing
checks. For every real production crop, the native RGB hash and input tensor
hash match OpenCV. Diagnostic lossless PNG exports preserve the exact Rust
JPEG-decoded pixels, avoiding a comparison between different JPEG decoders.
Those exports and the original images remain ignored and are not in evidence.
IPP/OpenCL and other OpenCV builds have not been validated.

The model, four views, guarded Resize2x/depthwise implementation, decoder,
global merge and vehicle consistency remain fixed. Two interleaved executions
per image/mode use reversed order on repeat: 384 forwards. Two current original
ONNX graph references per image add 192 forwards. Total: **576 unique forwards**,
144 image reports and 69 pinned producer-source files. All 192 repeat tensor
hash pairs and 192 optimized/original-graph tensor hash pairs agree, together
with every decoded f32 measurement, merged result and consistency decision.
These are separately computed canonical f32 little-endian SHA-256 comparisons;
raw model tensor arrays are not redistributed. Independent NumPy postprocessing
reconstructs every report, and original pinned annotations supply the scores.
Model weights and score/NMS/IoU thresholds stay unchanged.

The first saved-result audit exposed a Python tuple/list equality error in the
gained/lost reference-ID summary. The exact initial checker, failing audit and
single representation correction are retained. Re-auditing all saved reports
changes no inference report, score, summary bytes or adoption decision and
executes zero further forwards. The initial fetch encountered HTTP 503; a
same-URL retry completed the unchanged selection. The fetcher now bounds
transient retries, with tests confirming that HTTP 403 is not retried.
An initial model-free control attempt completed the same 48 generated cases
before stopping on the incomplete fetch receipt. The final control session
repeats them and checks all 24 real images; this is 48 unique generated cases,
not 96 unique cases and not additional model inference.

The native command passes 22 tests (three new preprocessing tests and nineteen
reused checks). Fifteen Python checks cover receipt corruption, output parity,
unfiltered false negatives, fixed quality gates and bounded download retries.
Six invalid native inputs publish no success; an existing output is preserved.
The required root check passes 371 tests and 49 scenario replays, preserving
all 245 pinned core outputs, 157 previous sources and 78 previous evidence
indexes / 6,510 entries. Fmt, all-target Clippy and workflow lint pass locally.
The new workflow has no observed remote execution.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-linear
python -m pip install -r scripts/requirements-visual.txt opencv-python-headless==4.13.0.92
python -B integrations/onnx/fetch.py --output artifacts/camera-model
python -B integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
python -B integrations/onnx/camera_linear_fetch.py
python -B integrations/onnx/camera_linear_check.py controls --output artifacts/camera-linear-repro/controls
python -B integrations/onnx/camera_linear_check.py run \
  --controls artifacts/camera-linear-repro/controls --output artifacts/camera-linear-repro/validation
python -B integrations/onnx/camera_linear_check.py audit \
  --controls artifacts/camera-linear-repro/controls --output artifacts/camera-linear-repro/validation
python -B assets/camera-linear-v1/verify.py
```

The last command needs neither original images, model weights nor OpenCV and
executes no inference. Native `--preprocess IMAGE --input-size N --output NEW`
reports input hashes without a model. Adding `--include-pixels` exports bounded
diagnostic pixels locally. `--model FILE --image FILE --preprocessing linear`
or `triangle` plus `--output NEW` performs four-view analysis; `--original-plan`
selects the separate original-graph reference. Outputs use existing complete,
no-clobber publication. Every report denies control permission.

This adds a functional preprocessing implementation, new road-image evaluation
and an upstream compatibility check. It supplies no live camera transport,
tracking-identity accuracy, metric depth, measured driving availability or
vehicle integration. The subjective maturity estimate against Autoware,
Apollo and openpilot remains **about 20%**, with **30% unmet**. Original BDD
annotations and derived evidence retain their separate research/educational/
not-for-profit data terms; Rust code remains Apache-2.0.
