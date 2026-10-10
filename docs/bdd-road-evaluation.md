# Original BDD dashcam camera diagnostic

The unchanged optional Rust-native CPU YOLOX nano detector now has a separately
frozen diagnostic on eight actual dashcam frames, with independent original-format
BDD100K boxes. These are outside COCO, the upstream model's published training
dataset. No BDD image was used to tune thresholds, model parameters, decoder or
preprocessing. This addresses the previous COCO-only evaluation limitation;
it does not establish an automotive-ready learned perception stack.

The first evaluation found **35 true positives, 14 false positives and 103 missed
objects out of 138 references**: precision 0.714, recall 0.254, mean matched IoU
0.740. Rejections, small objects, night scenes and difficult weather were retained
in the report. These low recalls are evidence of an unresolved capability gap.

## Measured outcomes

| Class | References | TP | FP | FN | Precision | Recall |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Person, including original rider labels | 28 | 4 | 0 | 24 | 1.000 | 0.143 |
| Bicycle | 3 | 0 | 0 | 3 | null | 0.000 |
| Car | 72 | 30 | 9 | 42 | 0.769 | 0.417 |
| Motorcycle | 3 | 0 | 0 | 3 | null | 0.000 |
| Bus | 2 | 1 | 1 | 1 | 0.500 | 0.500 |
| Truck | 6 | 0 | 3 | 6 | 0.000 | 0.000 |
| Traffic light | 24 | 0 | 1 | 24 | 0.000 | 0.000 |

There are no train references; train recall is null. Null precision means no
prediction for that class, not perfect accuracy. Generic BDD traffic signs do
not correspond to COCO's stop-sign class: all 38 original sign boxes are preserved
as excluded references, with an explicit reason, and are outside these metrics.
Non-road COCO predictions are preserved separately rather than counted as road
user false positives. No traffic-light color is inferred or evaluated.

| Selected original image | Conditions | References | TP | FP | FN |
| --- | --- | ---: | ---: | ---: | ---: |
| 00a395fe-d60c0b47 | Overcast city, daytime | 36 | 7 | 2 | 29 |
| 026c7465-d54954fa | Overcast highway, daytime | 5 | 1 | 1 | 4 |
| 0798a3a8-c4501e0f | Snowy city, daytime | 26 | 7 | 1 | 19 |
| 0a493f24-352f747e | Clear highway, daytime | 3 | 1 | 1 | 2 |
| 17d21997-6f076249 | Clear city, daytime | 7 | 1 | 1 | 6 |
| 4899be53-dcc6c017 | Clear city, night | 15 | 3 | 3 | 12 |
| 4dab8d2a-be9667d4 | Snowy residential, daytime | 19 | 9 | 5 | 10 |
| ab2360f2-7704a1bf | Rainy city, daytime | 27 | 6 | 0 | 21 |

The [complete first report](../assets/recorded-bdd/first-results.json) retains
each unmatched original annotation ID/box, every unmatched prediction and all
matched IoUs. The first report is never overwritten by a later run. There is no
accuracy acceptance threshold: the check's successful exit means valid real
execution and independent scoring, while `automotive_accuracy_protocol_passed`
is explicitly false.

## Selection, labels and licensing

The eight IDs above were selected using class/weather/time annotation metadata,
sorted and communicated before camera bytes were acquired. This is deliberate
coverage sampling, not a random benchmark sample. Existing viewed COCO images
remain the calibration/regression evidence; all eight BDD frames were new to
RustDriving's image decoding and evaluation at the first frozen run. Every later
run on these BDD frames is a viewed regression. Their original BDD training or
validation split is recorded separately. The upstream model publishes COCO
training; absence from every upstream model-selection workflow cannot be
independently guaranteed.

Original-format JSON and JPEGs are fetched from `ViscaaBarca/MLops-City`, revision
`a87c85607d023e3815ccb58bd80a62ab5a8a8dfd`. Each annotation preserves the original
`frames[0].timestamp = 10000`, object categories, box coordinates, occlusion and
truncation attributes. `person` and `rider` both map to COCO person; `bike` maps
to bicycle and `motor` to motorcycle. No model prediction is used as a label,
and occluded/truncated references are not filtered out.

The image bytes are independently identical by Git object hash and byte count
to the copies in `maye-msft/ai-mini-hack`, revision
`61755749e93a5e9dc2288deb04d16faa0a0984f7`, which preserves the Regents/BAIR dataset
license alongside its samples. **Its processed annotation files are not used**:
discovery comparisons found altered box coordinates and an added bicycle label
relative to canonical originals. The small mirror's original-format annotations
are pinned, but their byte-equivalence to the complete official BDD detection
archive was not independently checked. Legacy label versions can omit distant
objects or differ from Detection 2020 annotations; no versions are mixed here.

The [official BDD data license](https://github.com/bdd100k/bdd100k/blob/9ac17c6c7c51d2fc83065fccd707cd5b1882a293/doc/source/license.rst)
permits educational, research and not-for-profit use; general commercial rights
are limited to BDD/BAIR members and affiliates or a separate license. This
optional input diagnostic is for research. Credit: The Regents of the University
of California and the BDD100K authors, *BDD100K: A Diverse Driving Dataset for
Heterogeneous Multitask Learning* (Yu et al., CVPR 2020).

All raw JPEGs and annotation files remain under ignored `artifacts/`; none are
redistributed. The data-derived boxes and reference metadata in the published
report retain the [accompanying data license](../assets/recorded-bdd/LICENSE-data.txt),
and are **not relicensed under Apache-2.0**. Evaluator code remains Apache-2.0.
The [source record](../assets/recorded-bdd/SOURCE.json) gives individual URLs,
license provenance, SHA-256 and independently matching original/licensed image
Git hashes. Images plus annotations total 739,054 bytes; the input cap is 10 MiB.

## Reproduction and frozen evidence

The [protocol](../integrations/onnx/bdd_protocol.json) pins eight 1280×720 images,
canonical labels, class mapping, confidence 0.3, NMS IoU 0.45 and matching IoU 0.5.
The [first freeze](../assets/recorded-bdd/first-freeze.json) records source, locked
dependency, model and actual executable hashes. The
[preregistration](../assets/recorded-bdd/preregistration.json) was recorded before
first decoding or inference. The existing nano model, Rust decoder and lockfiles
were unchanged from the preceding checked camera execution.

```sh
python3 integrations/onnx/fetch.py --output artifacts/camera-model
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml
mkdir -p artifacts/bdd-camera/check
python3 integrations/onnx/bdd_check.py \
  --binary integrations/onnx/target/release/rustdriving-camera-detect \
  --model artifacts/camera-model/yolox_nano.onnx \
  --prepare-freeze artifacts/bdd-camera/check/reproduction-freeze.json
python3 integrations/onnx/bdd_check.py \
  --binary integrations/onnx/target/release/rustdriving-camera-detect \
  --model artifacts/camera-model/yolox_nano.onnx \
  --freeze artifacts/bdd-camera/check/reproduction-freeze.json \
  --data artifacts/bdd-camera --output artifacts/bdd-camera/check
```

Each reproduction needs its own immutable freeze for the locally built binary;
the published first freeze remains the original host's evidence. Preparation
refuses to overwrite an existing freeze and performs no image decoding or model
inference. Actual execution refuses a changed freeze before camera input access,
then runs every image twice: 16 real Rust CPU inference runs, identical repeated
boxes on the measured host. Source, binary, model and external freeze identity
are checked again after completion. Host timings are recorded without a latency
acceptance gate or real-time claim.

Python independently performs descending-confidence, class-aware, one-to-one
pixel-box matching. Eight corrupted model/image/dimension/threshold/geometry/class/
label probes were rejected. Five independent hand-labelled cases verify exact
matches, duplicate false positives, wrong-class rejection, exactly IoU 0.5 and
just below IoU 0.5. A correct prediction cannot count twice against a reference.

This diagnostic has no camera calibration, metric depth, temporal association,
camera/LiDAR fusion, driving control input or safety qualification. Improving
recall on genuinely new dashcam scenes requires further model evaluation and
data coverage; low-recall baselines are preserved as evidence for that work.
