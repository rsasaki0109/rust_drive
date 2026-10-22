# YOLOX-S640 capacity and supported-view comparison

This offline experiment compares the cached, independently pinned official
YOLOX-S640 with Nano416 on exactly the same original8 viewed and new16 BDD
images. It also compares S original4, unfiltered10 and supported10 policies
using the same measured S views. Results and the adoption decision are recorded
in `assets/camera-capacity-v1/validation/summary.json.gz`.

The first fresh16 trial contains344 mapped references:

| Policy | TP | FP | FN | Precision | Recall |
| --- | ---: | ---: | ---: | ---: | ---: |
| Nano original4 |142|76|202|65.14%|41.28%|
| S original4 |212|80|132|72.60%|61.63%|
| S unfiltered10 |225|105|119|68.18%|65.41%|
| S supported10 |222|90|122|71.15%|64.53%|

S capacity improves same-image coarse precision by7.47 percentage points and
recall by20.35 points. Supported additions retain all212 coarse matched
references and gain10, while adding10 false positives. Precision regresses
against S coarse in both fresh16 and viewed8. The viewed8 supported result is
80TP/37FP/58FN versus coarse78TP/31FP/60FN. All policies fail the full absolute
quality gates. On fresh16, S supported recall is0/1 for motorcycles,5/13 for
trucks and24/62 for traffic lights, below the unchanged50% per-present-class
minimum. `adopt_candidate:false`; no default changes follow this result.

All current repeated decoded measurements and every S original-graph reference
match bit-for-bit after float32 canonicalization, including complete support
decisions. Rust40 tests, Python16 checks,11 valid/9 invalid native fixtures,
24 independent decoded-pixel crop manifests, all-target Clippy and the root
371 tests/49 scenario replays pass. This verifies execution and reconstruction;
it does not turn failed quality gates into acceptance.

The complete pinned Git tree determines the first16 remaining validation
JPEG/label pairs, excluding every prior56 image and cached JPEG stem, before
acquisition. There is no image replacement, threshold sweep, label-based sample
selection or new class exclusion. The original class map, all occluded and
truncated boxes, score0.3, NMS0.45, match IoU0.5 and absolute quality gates remain.
The added current Nano comparison was fixed before image decoding or inference;
the initial selection protocol and completion receipt are retained separately.

The new CLI runs the guarded S Resize2x lowering or the original S graph.
Both retain Triangle preprocessing, raw BGR float32 and top-left114 padding.
The original1..4 views remain a prefix. Supplemental squares use
`min(width,height,640)` sides and the existing evenly distributed75%-stride
recipe. A1280x720 image has10 views, processed in joined waves of at most4
workers. Complete inventories over16 are rejected. No partial report is success.

The supported policy's production source is byte-identical to the frozen Nano
policy; only test coordinates adapt to640px crops. All original coarse boxes,
scores, order and lineage remain an immutable prefix. Supplemental additions
require two distinct crops, no overlap with a coarse box and no conflicting
vehicle label. Correlated crops are not independent sensors or probabilities.
Independent NumPy code reconstructs every merge, decision and work counter.

Every S image runs twice with the optimized plan and once with the current
serial original graph:720 forwards. The frozen Nano four-view producer runs
twice on the same24 images:192 forwards. Total912 new forwards, no warmup or
retry for timing. Repeats and the S current original graph must preserve every
decoded float32 box/confidence/class/order bit and complete postprocessing.
This comparison does not inspect new raw tensor bits or establish a timing,
availability, BDD mAP, driving integration or physical safety result.

```sh
source scripts/env.sh
cargo test --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-capacity
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml --bin rustdriving-camera-capacity --bin rustdriving-camera-depthwise --bin rustdriving-camera-linear
python -B -m unittest discover -s integrations/onnx -p test_camera_capacity.py -v
python -B assets/camera-capacity-v1/verify.py
python -B integrations/onnx/fetch_s.py --output artifacts/camera-model-s
python -B integrations/onnx/fetch.py --output artifacts/camera-model
python -B integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
python -B integrations/onnx/camera_capacity_fetch.py
python -B integrations/onnx/camera_capacity_check.py controls --output artifacts/camera-capacity-check/controls
python -B integrations/onnx/camera_capacity_check.py run --controls artifacts/camera-capacity-check/controls --output artifacts/camera-capacity-check/validation
python -B integrations/onnx/camera_capacity_check.py audit --controls artifacts/camera-capacity-check/controls --output artifacts/camera-capacity-check/validation
```

`verify.py` audits the frozen packet without images, weights or model execution.
The CLI's `--fixture` is explicit bounded model-free diagnostics, not attestation
of inference or report admission. Images, decoded pixel exports, raw weights,
executables and Python wheels are excluded from committed evidence. Original
annotations retain the separately copied BDD research-data license. All prior
evidence and core scenario outputs remain unchanged. The CI workflow is added
but remote execution is unobserved. Maturity remains about20%, below30%.
