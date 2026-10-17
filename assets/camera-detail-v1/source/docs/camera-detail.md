# S640 inference with416px detail views

The previous S640 trial misses39 of62 traffic lights in its fresh16 coarse
result. Of those misses,35 have maximum box side below32 source pixels; only2
have any same-class IoU>=0.5 candidate in all10 measured views. The frozen
viewed diagnosis motivates416px supplemental crops with the unchanged S640
input, increasing object size in the tensor without a class-specific rule.
The size buckets describe errors; they never filter scoring references.

The first fresh16 trial has253 mapped references:

| Policy | TP | FP | FN | Precision | Recall |
| --- | ---: | ---: | ---: | ---: | ---: |
| S original4 |161|50|92|76.30%|63.64%|
|640px supported10 |163|61|90|72.77%|64.43%|
|416px unfiltered12 |179|84|74|68.06%|70.75%|
|416px supported12 |166|58|87|74.11%|65.61%|

Supported detail gains4 reference identities and loses1 versus the640px
supported baseline. Signal TP rises9 to11 of24; its45.83% recall still misses
the50% gate. Bicycle1/3 and truck5/12 also fail. Precision remains below the
original4 result. On viewed24, detail302TP/120FP/180FN improves baseline
302TP/127FP/180FN precision but has equal overall recall and regresses person
TP77 to76 and car180 to177. All policies fail the complete absolute quality
gates. `adopt_candidate:false`; no default changes follow this experiment.

All repeated and original-graph decoded results, the original4 prefix and
complete coarse consistency results match exactly after declared float32
canonicalization. All168 new image reports are independently reconstructed;
their1920 forwards and the reused720 forwards remain separately counted.
Rust40/Python16 checks,11 valid/9 invalid native fixtures,40 decoded-image
manifests for both grids, all-target Clippy and371 root tests/49 scenario
replays pass. Execution/integrity success is separate from quality acceptance.

The new producer instantiates the frozen416px complete grid and original
supported policy. For1280x720 it measures the unchanged original4 prefix and
eight supplemental squares, totaling12 views in joined waves of at most4
workers. Scores0.3, NMS0.45, match IoU0.5, all original mapped classes and the
entire support policy remain unchanged. Coarse detections, confidence, order
and lineage are preserved. Correlated crops are not independent sensors.

The same pinned official S model and guarded Resize2x lowering are compared
with a current serial original S graph. Triangle/raw-BGR/top-left114
preprocessing remains approximate to upstream OpenCV and unchanged. Each new
detail image runs twice optimized and once with the original graph. There is
no warmup, retiming, threshold sweep or image replacement.

The viewed group is the previous capacity trial's original8 plus16, totaling24.
Its72 frozen S640-grid reports represent720 old forwards that are not counted
again. The new group is the first16 remaining lexicographic BDD validation
JPEG/label pairs in the same pinned complete tree, excluding every prior72
image and cached JPEG stem before acquisition, decode or inference. New
detail40*12*3=1440 forwards and fresh baseline16*10*3=480 total1920. Full saved
outputs are pinned before numeric labels from the new group are parsed.

An independent NumPy checker reconstructs complete merge, support and work
counters. The exact original decoded JPEG pixels come from the frozen
model-free reader; array crops independently hash both416px and640px view
inventories on all40 inputs. Every current repeat and original-graph reference
must preserve decoded float32 class/confidence/box/order and all decisions.
Both geometries must reproduce the exact original4 prefix and coarse result.
No new raw tensor bit comparison, latency, availability, BDD mAP or driving
integration is claimed. Root checks may run concurrently with this quality-only
trial, so observed wall times are not used for performance comparisons.

Adoption requires no supported precision regression versus coarse4 or the
640px supported baseline in either group, precision above unfiltered detail12,
recall above coarse and baseline, and no per-present-class recall regression
versus baseline. Fresh16 must also meet the unchanged absolute70% precision,
60% recall and50% per-present-class recall gates. Any failure rejects adoption.

```sh
source scripts/env.sh
cargo test --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-detail
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-detail --bin rustdriving-camera-capacity --bin rustdriving-camera-linear
python -B -m unittest discover -s integrations/onnx -p test_camera_detail.py -v
python -B assets/camera-detail-v1/verify.py
python -B integrations/onnx/fetch_s.py --output artifacts/camera-model-s
python -B integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
python -B integrations/onnx/camera_capacity_fetch.py
python -B integrations/onnx/camera_detail_fetch.py
python -B integrations/onnx/camera_detail_check.py controls --output artifacts/camera-detail-check/controls
python -B integrations/onnx/camera_detail_check.py run --controls artifacts/camera-detail-check/controls --output artifacts/camera-detail-check/validation
python -B integrations/onnx/camera_detail_check.py audit --controls artifacts/camera-detail-check/controls --output artifacts/camera-detail-check/validation
```

The frozen packet verifies without images, weights, network or inference.
`--fixture` is explicit model-free diagnostics, not model-execution attestation.
Images, pixel exports, weights, executables and wheels are excluded from the
packet. Original annotations retain the full separate BDD data license. The
workflow is added but remote CI is unobserved. The prototype remains simulation
research; this experiment supplies no camera-based driving control or maturity
increase. Maturity remains about20%, below30%.
