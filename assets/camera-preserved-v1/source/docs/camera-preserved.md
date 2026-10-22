# Preserve the640px supported baseline while adding detail measurements

Switching from640px to416px supplemental views removed46 baseline
measurements across the previous40 images, including8 person and24 car
measurements. Every original4 measurement still matched. The new component
keeps the complete640px supported result as an immutable prefix and appends
eligible416px supported detections. It never replaces class, confidence,
coordinate, order or source lineage of an anchor.

The first viewed40 result has735 mapped references:

| Policy | TP | FP | FN | Precision | Recall |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original4 |451|161|284|73.69%|61.36%|
|640px supported |465|188|270|71.21%|63.27%|
|416px supported |468|178|267|72.45%|63.67%|
| Preserved union |478|200|257|70.50%|65.03%|

All465 original supported matched references remain matched;13 more are gained,
with12 additional false positives. This repairs prediction removal and the
observed matched-reference loss, but regresses precision and still fails the
per-present-class gate. All56 input/output prefixes,168 native fixture results,
repeats and original-graph checks pass reconstruction. Rust6/Python12 checks,
6 valid/11 invalid native controls, all-target Clippy and371 root tests/49
scenario replays pass. Fresh16 quality remains unknown for the reason below.

`rustdriving-camera-preserved --fixture FILE --output NEW` is an explicit
model-free measured-box postprocessor. It accepts global source-image pixel
boxes and origins, validates both bounded streams, vetoes any proposal
overlapping an anchor at float32 IoU>0.45, applies classwise IoU>0.45 NMS only
among accepted additions and appends up to300 total boxes. A full anchor takes
precedence over all proposals. Each input stream has at most300 boxes and
unique view/rank origins; the comparison budget is180,000. Images are bounded
to16M pixels, input/output to4MiB. Publication is complete and cannot overwrite
an existing destination. This API does not attest to model execution or admit
external reports as a new inference producer.

The evaluation separately verifies the complete original S640 supported10 and
detail-supported12 native reports, exact decoded crop pixels, model/source
identities, original4 prefix and complete coarse result before deriving each
fixture. Existing two-crop support, confidence0.3, NMS0.45, all mapped classes,
all original boxes and match IoU0.5 remain unchanged. Independent NumPy code
reconstructs every result, lineage, decision and counter. Signed zero is
preserved, including its float32 total ordering in proposal ties. A preserved
prediction prefix does not mathematically guarantee greedy GT match retention;
the score and reference identities are checked separately.

Viewed40 reuses240 native parent reports/2640 old forwards, never recounted.
New16 is the first remaining lexicographic validation JPEG/label pairs in the
pinned complete BDD tree, excluding all88 prior images and cached JPEG stems
before acquisition. Both frozen producers execute twice optimized and once
with the original S graph on every new image:16*(10+12)*3=1056 new forwards.
All56 pairs are postprocessed three times:168 new fixture runs, zero forwards.
All outputs are pinned before new numeric labels are parsed. There is no
warmup, timing retry, image replacement or threshold sweep.

Adoption requires precision no worse than coarse4 and supported10, recall
strictly above both, no present-class recall regression versus either supported
parent and retention of every supported10 matched reference in both groups.
Fresh16 must also pass the unchanged70% precision/60% recall/50% present-class
recall gates. Any failure rejects adoption. These remain convenience samples,
not BDD mAP or proof of model-training non-overlap.

```sh
source scripts/env.sh
cargo test --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-preserved
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-preserved --bin rustdriving-camera-capacity --bin rustdriving-camera-detail --bin rustdriving-camera-linear
python -B -m unittest discover -s integrations/onnx -p test_camera_preserved.py -v
python -B assets/camera-preserved-v1/verify.py
python -B integrations/onnx/fetch_s.py --output artifacts/camera-model-s
python -B integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
python -B integrations/onnx/camera_capacity_fetch.py
python -B integrations/onnx/camera_detail_fetch.py
python -B integrations/onnx/camera_preserved_fetch.py
python -B integrations/onnx/camera_preserved_check.py controls --output artifacts/camera-preserved-check/controls
python -B integrations/onnx/camera_preserved_check.py run --controls artifacts/camera-preserved-check/controls --output artifacts/camera-preserved-check/validation
```

The frozen packet verifies without weights, images, network or inference. Raw
images, decoded pixel exports, weights, binaries and wheels are excluded.
Original annotation bytes retain the separate complete BDD data license.
The first fresh16 scoring attempt rejects an original zero-height traffic-light
annotation: image1de6d5ff-8aba6383, reference22, box
`[580.262269,94.262013,591.279907,94.262013]`. All1056 new forwards,168 native
fixtures, repeats, original-graph checks and prediction prefixes completed
before numeric-label scoring. The original checker, its first traceback,
protocol and annotation bytes are unchanged. No image or label is replaced,
clipped or dropped; no threshold is relaxed and no model is rerun.

The additive `camera_preserved_failure.py` auditor reconstructs all outputs,
requires the same original rejection and records fresh quality as unknown.
Only the fixed viewed40 group is scored. `adopt_candidate:false`; this is a
failed fresh-input protocol, not a passing fresh accuracy result. Use this
command after the original `run` exits1:

```sh
python -B integrations/onnx/camera_preserved_failure.py --controls artifacts/camera-preserved-check/controls --output artifacts/camera-preserved-check/validation
```

Remote CI is unobserved. No raw-tensor comparison, real-time/availability,
driving-control permission or physical safety claim follows. Root checks may
run concurrently with quality inference; wall times are not performance
comparisons. Maturity remains about20%, below30%.
