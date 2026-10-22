# Bounded small-object camera grid

The optional native `rustdriving-camera-grid` adds eight overlapping 416-pixel
views to the original four views on 1280×720 road images. It reduces misses but
increases false positives substantially. **It is not adopted as a default.**

| Sample / inventory | TP | FP | FN | Precision | Recall |
|---|---:|---:|---:|---:|---:|
| Viewed 24 / original four | 154 | 99 | 252 | 60.87% | 37.93% |
| Viewed 24 / combined twelve | 191 | 173 | 215 | 52.47% | 47.04% |
| Fresh 16 / original four | 112 | 83 | 222 | 57.44% | 33.53% |
| Fresh 16 / combined twelve | 146 | 143 | 188 | 50.52% | 43.71% |

The fresh sample has 334 mapped original reference objects. Recall improves by
10.18 percentage points, but precision falls by 6.92 points. The candidate gains
37 reference identities and loses three. Person/rider matches rise from 11 to
15, cars from 85 to 105, trucks from four to five and traffic lights from nine
to eighteen. Person/rider false positives rise from five to fourteen and traffic
light false positives from eight to twenty-one. Bicycle and motorcycle recall
remain zero. No present-class recall decreases in aggregate, but this does not
preserve every previous reference match. The viewed sample gains 38 identities
and loses one. All per-image/class failures are retained.

The original precision ≥0.7, recall ≥0.6 and recall ≥0.5 for each present class
all fail. The predeclared adoption rule additionally requires precision not to
decrease, overall recall to increase and every present-class recall not to
decrease on both samples. Both precision comparisons fail. Scores, NMS and
matching thresholds, COCO class mapping, original labels, occluded/truncated
objects and sign exclusions remain unchanged. No image or difficult object is
removed. Successful integrity checks do not mean accuracy gates passed.

Sixteen new images were selected before acquiring/decoding them from the first
lexicographic original BDD `val` image/legacy-label pairs in a complete pinned
Git tree, excluding all cached JPEG stems, the original eight IDs and the
previous sixteen. The old 24 are now explicitly viewed development data.
The new sixteen are now viewed as well. This is a small convenience sample,
not representative coverage, BDD mAP, or proof of non-overlap with upstream
training. The complete selection and thresholds are in `profile-grid.json`.

The original full frame and three max-square views remain the prefix. New grid
tile side is `min(width,height,416)` pixels. For each axis the maximum stride
is `max(1,3*side//4)`; `ceil((dimension-side)/stride)` intervals are spaced with
integer-floor offsets between both image edges. New rectangles append in
row order and duplicates are removed. This gives eight extra tiles on 1280×720
or 720×1280 images. Adjacent tiles overlap by at least one quarter of their side,
apart from integer/single-pixel conventions. Unit checks verify full small-tile
coverage and the unchanged original prefix.

The complete combined inventory is mandatory and limited to sixteen views.
The layout rejects images whose required inventory exceeds that budget; it
does not silently omit tiles. Examples include 1920×1080 and 1280×1280. Source
images retain the existing 16 MB encoded / 16 million pixel limits. At most four
views run concurrently in ordinary scoped-thread waves. All started threads
are joined before a failed result or another wave; a failed wave never starts
later waves. No custom runtime, extra dependency or unsafe code is introduced.

The original Nano weights, Triangle/f32 preprocessing, 30 guarded depthwise
operations and two Resize lowerings remain fixed. Native global merge keeps
the existing one-pixel interior-edge exclusion, classwise IoU >0.45 NMS,
confidence/class/left/view/rank ordering and 300-box output cap. Each view is
limited to 300 boxes. The existing 1,400,000-work consistency rejection remains
unchanged. Inventory validation occurs before loading a model. Pixel-space
detections are hypotheses; every image/inspection report denies driving control.

Two passes alternate candidate/baseline per image and reverse order on repeat.
They run 320 baseline and 960 grid forwards. Forty separate serial, original
ONNX-plan references add 480 forwards: **1,760 total**, 200 image reports and
70 pinned producer-source files. There are no pilot or warmup forwards, retries
to select a result, new latency gate or availability claim. Timing fields remain
actual execution observations. The twelve-view inventory costs three times as
many forwards per image as the four-view baseline; throughput is not established.

All 640 repeat per-view detection pairs, 480 current original-graph pairs and
320 original-prefix pairs agree in every class, f32 box/confidence and order.
Complete merges and consistency decisions also agree with their repeated and
original-plan counterparts. The new experiment compares decoded measurements;
it does **not** add raw model-tensor comparisons. The previously frozen kernel
bit checks remain separate evidence.

Independent NumPy merging and the unchanged vehicle-consistency oracle rebuild
every complete result. A frozen model-free Rust image reader supplies lossless
local exports of the exact decoded JPEG pixels; independent array crops verify
every view's RGB hash for all 40 images. Exports and original JPEGs stay ignored.
Eight valid native fixtures cover square, portrait, landscape, odd dimensions,
edge rejection and class ambiguity. Nine corrupt fixtures fail without output;
an existing output is preserved. Thirty native tests pass, including four new
grid/wave tests and reused/adapted checks; thirteen Python checks cover receipt
corruption, coverage, budget and measurement disagreement. The portrait-order
test expectation was corrected to row order before any model inference.

The required root check passes 371 tests and 49 scenario replays. All 245 core
output pins, 167 previous source files and 79 previous indexes / 6,795 entries
remain unchanged. Fmt, all-target Clippy, workflow lint and saved-result audit
pass locally. The new workflow has no observed remote run. Evidence in
`assets/camera-grid-v1` contains original new annotations under BDD's separate
research/educational/not-for-profit terms; implementation code is Apache-2.0.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-grid --bin rustdriving-camera-depthwise --bin rustdriving-camera-linear
python -m pip install -r scripts/requirements-visual.txt
python -B integrations/onnx/fetch.py --output artifacts/camera-model
python -B integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
python -B integrations/onnx/camera_linear_fetch.py --output artifacts/camera-linear-v1/data
python -B integrations/onnx/camera_grid_fetch.py
python -B integrations/onnx/camera_grid_check.py controls --output artifacts/camera-grid-repro/controls
python -B integrations/onnx/camera_grid_check.py run \
  --controls artifacts/camera-grid-repro/controls --output artifacts/camera-grid-repro/validation
python -B integrations/onnx/camera_grid_check.py audit \
  --controls artifacts/camera-grid-repro/controls --output artifacts/camera-grid-repro/validation
python -B assets/camera-grid-v1/verify.py
```

The last command needs neither images, weights, executables nor OpenCV and runs
no model. The native command accepts `--model FILE --image FILE --output NEW`;
`--reference-image` selects the serial original graph, `--inspect-image` reports
layout without a model, and `--merge-fixture`/`--reconcile-fixture` exercise the
bounded typed view contract. Reports retain complete no-clobber publication.

This adds an executable small-object inventory and measures its recall/precision
tradeoff on newly selected road images. Live transport, object-identity accuracy,
metric depth, driving integration and real-time availability remain absent.
Maturity relative to mature OSS stays **about 20%** (subjective), with **30% unmet**.
