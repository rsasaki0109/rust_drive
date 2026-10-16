# Bounded multi-view road-camera detection

The optional `rustdriving-camera-detect-tiled-s` binary reuses the pinned
[YOLOX-S model and preprocessing](yolox-s-camera.md). It runs the whole image
and overlapping square crops through one CPU plan, restores source-image pixel
coordinates, then applies classwise NMS. The existing nano and S binaries keep
their original behavior. This adapter emits image boxes for offline research.

The [fixed design](../assets/yolox-s-tiles-v1/design.json) selects at most four
views before inference: the full image and unique squares of side
`min(width,height)` at long-axis offsets zero, half the remaining gap rounded
down, and the full gap. A 1280 × 720 frame therefore adds 720-square crops at
x = 0, 280 and 560. Boxes within one source pixel of an interior crop edge are
discarded; actual image boundaries remain eligible. Confidence remains 0.3,
local/global NMS IoU 0.45 and reference-match IoU 0.5. Equal-score merge ordering
is deterministic, and each output records its source view and local rank.

Encoded input remains bounded to 16 MiB and decoded images to 16 million
pixels. At most 300 detections per view enter the merge, with a global limit of
300 outputs and at most 360,000 candidate comparisons. These limits and the
reused model plan do not establish a real-time bound.

## Measured result

All eight previously viewed BDD frames and all 138 original scoped references
remain in the diagnostic. No confidence threshold, category mapping, missing
object or quality gate changes. This is a viewed development regression,
without a fresh held-out generalization claim or BDD mAP measurement.

| Result | Single-view S | Four-view S |
| --- | ---: | ---: |
| Matched reference objects | 55 | 78 |
| False positives | 14 | 33 |
| Missed references | 83 | 60 |
| Precision | 79.71% | 70.27% |
| Recall | 39.86% | 56.52% |

All 55 earlier reference matches remain matched, with 23 additional matches.
Person/rider recall is 20/28, bicycle 1/3, car 48/72, motorcycle 0/3, bus 1/2,
truck 4/6 and traffic light 4/24. No train reference is present. The unchanged
gate still **fails**: overall recall is below 60%, and bicycle, motorcycle and
traffic-light recalls are below 50%. The precision floor is 70%.

The first native pass sums to approximately 2.18 seconds of inference per
image on this CPU host, excluding model loading, decoding, merging and scoring.
The single full-image view independently agrees with the historical S result.
Historical cross-host comparison retains the existing S replay's absolute
tolerance of 1e-5 after f32 canonicalization, with exact class/order/count identity; local repeats still
compare f32 values. Separate roundoff and excessive-difference controls protect
that distinction, and the archived original native reports remain unchanged.
Both complete passes reproduce f32 detections and source-view identities.
Separate NumPy merge calculations reproduce coordinates, edge rejections, NMS
and work counts; Pillow PNG controls verify the actual crop pixels for
landscape, portrait, square, odd-gap, duplicate-view and one-pixel cases.
Eight malformed input cases and existing-output protection are checked.

Five fixed reliability bins on 111 emitted scoped detections give descriptive
Brier score 0.15656 and ECE 0.07461. These are conditional on confidence ≥0.3
and emitted boxes, omit undetected objects and use viewed data. They do not
establish calibrated probabilities or close the held-out calibration gate.

The first scoring attempt rejected equivalent f32/f64 JSON threshold
representations before numeric label parsing. Its source and failure log are
preserved. An auditor-only repair accepts exactly the configured decimal or
its exact promoted f32 value; altered thresholds still fail. The original 16
native reports remain unchanged, and a subsequent complete native run uses
the repaired checker. Integrity success retains physical **quality exit 1**.

BDD data retains the [original Regents/BAIR research and educational terms](bdd-road-evaluation.md).
Raw images, labels and weights remain in ignored local storage. The derived
[results](../assets/yolox-s-tiles-v1/results.json), source archives, native
reports, controls and data notice retain provenance and failures. Improved
viewed recall alone does not raise the overall maturity estimate above about
20% or establish the 30% waypoint.

## Reproduction

Use new output directories and the existing pinned model/data acquisition
described in the [S instructions](yolox-s-camera.md#reproduction).

```sh
source scripts/env.sh
python -m pip install -r scripts/requirements-visual.txt
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-detect-tiled-s
python integrations/onnx/bdd_tiles_check.py controls \
  --binary integrations/onnx/target/release/rustdriving-camera-detect-tiled-s \
  --output artifacts/tiles-example/controls
python integrations/onnx/bdd_tiles_check.py run \
  --binary integrations/onnx/target/release/rustdriving-camera-detect-tiled-s \
  --controls artifacts/tiles-example/controls \
  --output artifacts/tiles-example/regression
# Expected exit 1: integrity succeeds, unchanged physical quality gates fail.
```

The standalone binary also exposes `--inspect-views --image FILE --output NEW`
and `--merge-fixture FILE --output NEW` without loading a model. Normal
inference uses `--model FILE --image FILE --output NEW`. Reports require fresh
files; no driving EKF, planner or control input is produced.
