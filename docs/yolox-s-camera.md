# Optional YOLOX-S automotive camera comparison

RustDriving executed the fixed offline CPU comparison of official **YOLOX-S**
against the preserved YOLOX nano BDD diagnostic. On eight already viewed images,
S matches **55/138 references**, compared with nano’s **35/138**, but still fails
the fixed quality gates. Pixel boxes remain separate from driving controls.

## Model identity and preprocessing provenance

[The profile](../integrations/onnx/profile-s.json) pins the official
`Megvii-BaseDetection/YOLOX` release `0.1.1rc0`, asset **42724906**:
`yolox_s.onnx`, **35,858,002 bytes**, SHA-256
`c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063`.
The graph has float32 input **[1, 3, 640, 640]** and **8,400 × 85** output
values: undecoded grid-relative boxes with sigmoid objectness/classes. Size and
SHA verification precede graph parsing; a local computed hash is not a
publisher signature or separately published checksum.

The release's old Git tag points to legacy normalization code. The frozen
profile instead records contemporary source revision
`c9fe0aae2db90adccc90f7e5a16f044bf110c816`, documenting the actual nonlegacy
release path: **raw BGR [0,255]**, top-left letterbox with 114 padding and CHW
float32, without division by 255 or mean/std normalization. This distinction
is explicit rather than assuming the old tag's preprocessing.

Rust resizes to uint8 with `image::Triangle`, an approximation of upstream
OpenCV `INTER_LINEAR`. Grid decoding uses strides **8/16/32**,
`xy = (raw_xy + grid) * stride`, `wh = exp(raw_wh) * stride`, then divides by
letterbox ratio. Score is objectness times the highest class probability, with
lower class ID breaking ties; one argmax class per row then classwise NMS is
the original Rust policy, differing from upstream full per-class emission.
The comparison does not claim exact upstream preprocessing/postprocessing parity.

Inputs are bounded to 16 MiB encoded bytes and 16 million pixels, with at most
300 retained detections. Invalid model identity, dimensions, probabilities or
box geometry fail closed. These are resource limits, not real-time guarantees.
YOLOX repository code is Apache-2.0; no separate weight-license declaration or
publisher checksum was found in the reviewed release/docs. We retain the
official publication/license provenance and do not redistribute raw weights.

## Fixed viewed BDD protocol

[The design](../assets/yolox-s-bdd-v1/design.json) fixes the same eight previously
viewed BDD dashcam images and all **138** scoped references, confidence **0.3**,
classwise NMS IoU **0.45** and match IoU **0.5** before first S inference. Model
selection used official architecture capacity/resolution rather than a search
over BDD scores. No competing-model trial or post-result threshold tuning is
part of this protocol.

Historical `heldout` tags are retained from the original acquisition, but these
images and labels are already viewed. This is **not fresh held-out evidence**,
BDD mAP or proof of unseen automotive generalization. The COCO-pretrained
model's upstream exposure also prevents such a claim.

All scoped misses and false positives remain scored. Person and rider map to
COCO person; bike, car, motor, bus, truck, train and traffic light retain the
declared mappings. Generic traffic signs have no matching COCO generic-sign
class and are separately counted, not relabelled as stop signs. A fixed small
diagnostic gate requires overall precision ≥ **0.7**, recall ≥ **0.6**, and
recall ≥ **0.5** for each class with references. Passing would still be only a
viewed diagnostic success.

The [original nano result](../assets/recorded-bdd/first-results.json) remains
**35 TP, 14 FP, 103 FN / 138 references**, recall **25.4%**. Its sources, model,
freeze and report remain immutable. The S comparison must freeze source/model
and binary identities before decoding, execute each image twice, verify repeated
boxes, and score numerical reference boxes only after all actual inference.
The [first results](../assets/yolox-s-bdd-v1/first-results.json) preserve all
**55 TP, 14 FP and 83 FN**, precision **79.71%** and recall **39.86%**.
All 16 actual CPU inference runs (first plus repeat for each image) exit 0 and
repeat detection identities match exactly. Numerical reference scoring follows
all inference. The [checker status](../assets/yolox-s-bdd-v1/first-check-status.json)
records integrity success but **quality exit 1**: overall recall misses 60%,
and person, bicycle, motorcycle and traffic-light recall miss 50%.

| Scoped class | Matched / references | Recall |
| --- | ---: | ---: |
| Person, including rider | 11 / 28 | 39.29% |
| Bicycle | 1 / 3 | 33.33% |
| Car | 39 / 72 | 54.17% |
| Motorcycle | 0 / 3 | 0% |
| Bus | 1 / 2 | 50% |
| Truck | 3 / 6 | 50% |
| Traffic light | 0 / 24 | 0% |
| Train | 0 / 0 | Not scored |

The [reference-ID comparison](../assets/yolox-s-bdd-v1/nano-comparison.json)
retains 34 nano matches, adds 21 and **loses one** previous match. Both model
and input resolution differ, so this is not a causal model-size ablation.
First-run inference averages **0.606 s/image**, ranging **0.574–0.678 s** on
this CPU host; model loading, image decoding and scoring are outside that
measurement. The earlier nano timing used a different run without an established
comparable protocol; no speed ratio or real-time claim is made.

The [pre-inference source/model freeze](../assets/yolox-s-bdd-v1/first-freeze.json),
[stage journal](../assets/yolox-s-bdd-v1/first-stage-journal.json),
[graph inspection](../assets/yolox-s-bdd-v1/graph-inspection.json) and
[pretrial negative controls](../assets/yolox-s-bdd-v1/pretrial-checker-controls.json)
separate provenance/integrity from failed accuracy acceptance. The further
[complete regression replay](../assets/yolox-s-bdd-v1/regression-suite.json)
executes all 16 CPU runs again: all eight first raw detection sets agree within
absolute tolerance **1e-5**, and match IDs, counts and quality gates are unchanged.
Its integrity/reproduction flag is true, while the
[wrapper exit remains 1](../assets/yolox-s-bdd-v1/reproduction-exit.json)
because quality still fails. This tolerance comparison does not claim byte
identity of timings or every floating-point output.

BDD data/labels retain their [documented research and educational terms](bdd-road-evaluation.md),
including commercial limitations. Raw images, labels and weights stay in ignored
storage; no modified previews or raw inputs are published.

## Reproduction

The new standalone binary is `rustdriving-camera-detect-s`; the original
`rustdriving-camera-detect` remains the nano path. Explicit `--bin` selection is
required now that Cargo discovers both binaries. The first complete S check
executed successfully and retained its failed-quality result.
Use a new checker output directory and the existing optional CPU environment:

```sh
python3 integrations/onnx/fetch_s.py --output artifacts/camera-model-s
python3 integrations/onnx/bdd_fetch.py --output artifacts/bdd-camera
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-detect-s
python3 integrations/onnx/bdd_s_check.py \
  --binary integrations/onnx/target/release/rustdriving-camera-detect-s \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --data artifacts/bdd-camera --output artifacts/yolox-s-bdd/local-comparison
```

The checker creates its own fresh output directory. Use the actual binary path
if `CARGO_TARGET_DIR` is configured. Exit 0 requires the fixed
viewed quality gates; exit 1 preserves valid inference/scoring that fails them.
The tested wrapper additionally verifies reproduction of the immutable first
evidence:

```sh
python3 integrations/onnx/bdd_s_replay.py \
  --binary integrations/onnx/target/release/rustdriving-camera-detect-s \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --data artifacts/bdd-camera --output artifacts/yolox-s-bdd/local-replay
```

Inference, independent scorer integrity and quality acceptance are separate
outcomes. Both checker and replay wrapper retain **exit 1** for this failed
quality result, even when integrity/reproduction passes; preserve their outputs.
It is not an inference crash. Timings describe the measured CPU
host, without a real-time or cross-platform guarantee.

This path supplies no metric depth, camera extrinsics, temporal association,
signal-color recognition or camera-derived vehicle commands. Maturity remains
**about 20% by subjective comparison with Autoware, Apollo and openpilot**;
the **30% and 50%** waypoints remain unmet.
