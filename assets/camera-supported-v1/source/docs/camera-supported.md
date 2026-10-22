# Coarse-preserving supported grid additions

The optional `rustdriving-camera-supported` preserves the original four-view
result and appends only supplemental hypotheses supported by distinct small
crops. It removes some false positives from unrestricted grid inference, but
does **not** preserve the four-view precision. **No default adoption occurs.**

| Sample / policy | TP | FP | FN | Precision | Recall |
|---|---:|---:|---:|---:|---:|
| Viewed 40 / original four | 266 | 182 | 474 | 59.38% | 35.95% |
| Viewed 40 / unrestricted twelve | 337 | 316 | 403 | 51.61% | 45.54% |
| Viewed 40 / supported additions | 293 | 207 | 447 | 58.60% | 39.59% |
| Fresh 16 / original four | 118 | 72 | 151 | 62.11% | 43.87% |
| Fresh 16 / unrestricted twelve | 151 | 113 | 118 | 57.20% | 56.13% |
| Fresh 16 / supported additions | 134 | 90 | 135 | 59.82% | 49.81% |

All three policies consume exactly the same measured views. On the fresh sample,
support removes 23 unrestricted false positives while losing 17 unrestricted
matches. Against the four-view baseline it adds 16 matches and 18 false positives.
All 118 original matched reference identities remain; all 266 viewed matches
also remain, with 27 gained. This identity retention is an observation on these
images, not a consequence guaranteed by keeping box measurements: a confidence
ordered reference matcher can assign overlapping predictions differently.

Fresh person/rider matches rise from eight to ten, cars from 101 to 112 and
traffic lights from three to six. Person/rider false positives rise from six
to eight, cars from 43 to 56, trucks from eleven to twelve and traffic lights
from nine to eleven. Bicycle, motorcycle and bus recall remain zero. Original
precision ≥0.7, overall recall ≥0.6 and every present-class recall ≥0.5 all fail.
The predeclared relative rule requires at least four-view precision, precision
strictly above unrestricted grid, improved four-view recall and no present-class
recall regression on both groups. Both precision comparisons with four views
fail. No score threshold, labels, size filtering or category scope changes.

Selection of sixteen new original BDD `val` image/legacy-label pairs is frozen
before acquiring or decoding them. It uses the first lexicographic eligible
pairs in the complete pinned Git tree, excluding cached JPEG stems and all
forty earlier images. No label counts, pixels or predictions affect selection,
and no chosen image is replaced. The new sample contains 269 mapped reference
objects and is now viewed data. It remains a convenience sample, not BDD mAP,
representative domain coverage or proof of upstream-training non-overlap.

The policy is fixed before all scoring:

1. Validate the complete existing grid, every box and the original bounds, then
   compute the unchanged original four-view consistency result. Other image
   shapes use the complete original one-to-four-view prefix.
2. Keep every coarse box, class, confidence, order and origin. Only supplemental
   small-crop detections become new proposals; preserve the original one-pixel
   interior-edge rejection and transform them to source-image pixel coordinates.
3. Reject a proposal at IoU >0.45 with any coarse output, regardless of class.
4. Require same-class IoU >0.45 with the same original proposal in at least two
   distinct supplemental views, including its own. Each view supplies at most
   one highest-confidence/lower-rank witness. Support does not follow chains.
5. Reject supplemental car/bus/truck proposals if any supplemental view supplies
   an overlapping conflicting rigid-vehicle label. Person/bicycle overlap is
   retained as distinct hypotheses.
6. Apply the original confidence/class/left/view/rank order and classwise NMS to
   additions. Append after the unchanged coarse prefix, up to 300 total boxes.
   A full coarse output cannot be displaced by a new higher-confidence proposal.

These are correlated crops of one image, **not independent sensor evidence or
calibrated probabilities**. The original ≤16 complete views, ≤4 concurrent
workers, ≤300 boxes/view, 1,400,000 coarse-consistency work limit and 4 MiB
complete-report bound remain. Supplemental work has a separate 27,000,000 IoU
comparison cap. Any failed bound rejects publication; no partial success is
published. The existing scoped-thread waves, detector weights, Triangle/f32
preprocessing, guarded depthwise/Resize code and decoder remain unchanged.
There is no new runtime, dependency or unsafe code.

Two four-view and grid namespaces instantiate the same frozen implementations
with their different complete inventories. A narrow `clippy::duplicate_mod`
allowance documents that deliberate reuse. The coarse context's production
body is byte-identical to the old source; only its test import uses the local
namespace. Both sets of tests compile and run. The image producer declares all
75 embedded source files. Native fixture mode accepts bounded typed boxes and
claims zero image reads or model forwards; it does not attest to a model run.
Image mode owns the actual detector measurements and denies control permission.

The forty viewed images reuse 120 saved first/repeat/original-graph reports.
Native fixture processing executes zero model forwards; their 1,440 historical
forwards are **not counted again**. The new sixteen run twice plus one serial
original-graph reference each: 48 native image reports, twelve forwards each,
**576 new forwards**. Each emits original-four, unrestricted-grid and supported
results together. Every repeated and current-reference detection, lineage,
decision, witness and work counter agrees. This increment compares decoded f32
measurements and full postprocessing; it adds no raw model-tensor comparison.

Independent NumPy processing reconstructs every naive, coarse and supported
result. Original pinned annotations recompute all scores. Model-free lossless
pixel exports from the frozen Rust reader verify the new sixteen inputs and all
192 new crop hashes; forty previous pixel manifests are reused unchanged. The
exports and original images are excluded from evidence. Forty-two native tests
pass, including six new policy cases and 36 reused/adapted tests. Eleven Python
checks cover source receipts, unchanged audit rules, immutable coarse results,
witness uniqueness and mutation rejection. Eleven valid native fixtures and nine
invalid fixtures pass; existing output is preserved.

An explicit Python `math` import required by the copied image auditor was added
before any new model inference. The exact initial checker is recovered against
its first control-session hash. Model-free controls were rerun with the final
checker before inference; these are eleven unique valid fixtures, not 22 unique
cases, and neither session adds model forwards. Native compile/test namespace
fixes also precede model execution. No threshold tuning or inference rerun selects
the result. All first image observations remain preserved.

The required root check passes 371 tests and 49 replays, with all 245 core output
pins unchanged. All 177 preceding sources and 80 evidence indexes / 7,122 entries
remain unchanged. Fmt, all-target Clippy, workflow lint and saved-result audits
pass locally. The new workflow has no observed remote run; green integrity
checks do not establish passed accuracy gates. Evidence has a 187-file source
closure, reusing previous archives and copying ten new sources. Only sixteen
new original annotations are added under the separate BDD research/educational/
not-for-profit terms; implementation code remains Apache-2.0.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-supported --bin rustdriving-camera-linear
python -m pip install -r scripts/requirements-visual.txt
python -B integrations/onnx/fetch.py --output artifacts/camera-model
python -B integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
python -B integrations/onnx/camera_linear_fetch.py --output artifacts/camera-linear-v1/data
python -B integrations/onnx/camera_grid_fetch.py --output artifacts/camera-grid-v1/data
python -B integrations/onnx/camera_supported_fetch.py
python -B integrations/onnx/camera_supported_check.py controls --output artifacts/camera-supported-repro/controls
python -B integrations/onnx/camera_supported_check.py run \
  --controls artifacts/camera-supported-repro/controls --output artifacts/camera-supported-repro/validation
python -B integrations/onnx/camera_supported_check.py audit \
  --controls artifacts/camera-supported-repro/controls --output artifacts/camera-supported-repro/validation
python -B assets/camera-supported-v1/verify.py
```

The final command needs no images, weights, executables or OpenCV and runs no
model. Native image mode is `--model FILE --image FILE --output NEW`;
`--reference-image` selects the original graph. `--fixture FILE --output NEW`
applies the same policy to the explicit complete typed view contract. All output
uses existing complete no-clobber publication.

The implementation adds a usable conservative augmentation rule and measures
its remaining precision/recall tradeoff. It supplies no live ingestion,
tracking-identity accuracy, metric depth, measured availability, vehicle
integration or safety claim. Maturity relative to mature OSS remains
**about 20%** (subjective), with **30% unmet**.
