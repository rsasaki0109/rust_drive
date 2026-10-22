# Optional Rust-native CPU camera detection

This standalone workspace runs the official YOLOX nano ONNX network through
`tract-onnx`, then decodes actual network outputs into 80-class pixel boxes and
per-class nonmaximum suppression. It is offline perception research, separate
from the vehicle pipeline. Boxes have image coordinates and no inferred metric
depth. The default RustDriving build has no model, camera, or runtime dependency.

```sh
python3 integrations/onnx/fetch.py --output artifacts/camera-model
cargo run --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-detect -- \
  --model artifacts/camera-model/yolox_nano.onnx \
  --image artifacts/camera-model/astronaut.jpg \
  --output artifacts/camera-model/detections.json
python3 integrations/onnx/score.py --data artifacts/camera-model \
  --detections artifacts/camera-model/detections.json \
  --output artifacts/camera-model/score.json
cargo test --release --locked --manifest-path integrations/onnx/Cargo.toml
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml
python3 integrations/onnx/check.py \
  --binary integrations/onnx/target/release/rustdriving-camera-detect
```

Python's standard library performs the explicit opt-in download and independent
pixel-box check. Rust loads JPEG/PNG, uses a 416×416 BGR top-left letterbox padded
with 114, and runs CPU inference. Resizing uses the image crate's triangle filter;
this differs slightly from upstream OpenCV bilinear interpolation. Score 0.3 and
classwise NMS IoU 0.45 are fixed before evaluating the fixture. Reports include
separate model-load and inference durations measured on the executing machine;
these are not real-time guarantees. The model is size- and SHA-verified before
ONNX parsing. Inputs are bounded to 16 MiB encoded data and 16 million pixels,
output to exactly 3,549×85 finite values and 300 retained boxes. Sigmoid
roundoff within 1e−6 of [0,1] is clamped; larger violations are rejected. Unknown models,
malformed images, invalid probabilities, geometry and output shapes fail closed.

## Provenance and licensing

- Official [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) by Megvii Inc. is
  Apache-2.0. Repository revision `6ddff4824372906469a7fae2dc3206c7aa4bbaee`
  documents the pretrained nano artifact from release `0.1.1rc0`.
  `yolox_nano.onnx`: 3,659,407 bytes, SHA-256
  `c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d`.
  We do not redistribute the model; the opt-in fetch uses its official release.
  The official repository does not publish a separate model-weight license;
  its Apache license and release publication are the provenance evidence.
- Measured portrait of astronaut Eileen Collins, NASA Great Images database
  ([NASA source](https://flic.kr/p/r9qvLn)).
  [scikit-image documentation](https://github.com/scikit-image/scikit-image/blob/b700edc8005e4ee9e74fc83259f09e8df3213f15/src/_skimage2/data/_fetchers.py)
  states: “No known copyright restrictions, released into the public domain.”
  The fetched JPEG is the torchvision gallery copy at revision
  `9a8d5453bdbcd882f7c7f401064b7514581636c3`. Its SHA-256 is
  `874dba0332a5a9a6a9268e732745f57c7ba21bc733867463daaae4a766f0a03f`.
- The independently supplied person box `[17,16,344,495]` (XYWH) comes from
  torchvision's `gallery/assets/coco/instances.json` at that same revision.
  Torchvision is BSD-3-Clause; the file is a two-image **COCO-format example**,
  not an official COCO benchmark. Only its astronaut/image-ID 1 annotation is
  scored. Annotation SHA-256:
  `11e721e049f44f43cba66f240c249869116380a7fb17a5880dd3102512efdba9`.

All downloaded assets and generated evidence stay under ignored `artifacts/`.
RustDriving code remains Apache-2.0. The isolated lockfile records actual inference
and decoder dependency versions; existing root and RNE dependency pins stay
unchanged. No Python inference package, GPU, ONNX Runtime shared library, or
custom runtime is required.

One measured portrait with one independently supplied reference box can verify
execution and box geometry. It cannot establish driving-camera accuracy or
COCO AP. A separate six-image urban-camera diagnostic below now measures road
users against independent labels. Camera calibration, camera/LiDAR association,
synchronization, general driving-camera accuracy, and control integration remain
unimplemented or unvalidated.

## Independently labelled urban-image diagnostic

The optional evaluator runs this same Rust CPU detector on four real urban
street photographs and two urban-sign hard negatives, with independently supplied
COCO boxes. `road-protocol.json` freezes image hashes, references, class mapping,
two calibration/four evaluator-held-out images, confidence 0.3, NMS IoU 0.45 and
match IoU 0.5 before inference. No tuning was performed on either partition.
The held-out partition is only held out from this evaluator: these photographs
originate in COCO val2017, which the upstream model could have used for model
selection. This is not evidence of unseen-scene generalization.

```sh
python3 integrations/onnx/fetch.py --output artifacts/camera-model
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml
python3 integrations/onnx/road_check.py \
  --binary integrations/onnx/target/release/rustdriving-camera-detect \
  --model artifacts/camera-model/yolox_nano.onnx \
  --data artifacts/road-camera --output artifacts/road-camera/check \
  --baseline integrations/onnx/road-results.json
```

The check fetches at most the pinned bytes for each input, performs twelve actual
CPU inference runs (each image twice), checks repeat identities, independently
matches predictions one-to-one within each class, and preserves every false
positive and missed object. Seven changed identity/threshold/geometry/label probes
must fail; a duplicated correct prediction must add an FP, never another TP.
The optional baseline checks recorded counts and matching identities, with
matched IoU tolerance 1e-5; timings are never an acceptance gate. It records a
valid execution, **not an automotive accuracy pass**.

The measured evaluator-held-out diagnostic contains 25 road-user/signal boxes:
14 TP, 4 FP and 11 FN, precision 0.778, recall 0.560, mean matched IoU 0.802.
Both hard negatives returned no road-user/signal predictions. Small distant
pedestrians, occluded cars and traffic lights were missed. `road-results.json`
contains class and image details, calibration results, source/binary fingerprints
and actual host inference times. This is a fixed-threshold diagnostic, not COCO AP.

### Urban input provenance and separate licenses

Inputs come from [Deci-AI/data-gradients](https://github.com/Deci-AI/data-gradients/tree/58a9c4493aafe335d8e7656568d59de7d08bf695/example_dataset/tinycoco),
revision `58a9c4493aafe335d8e7656568d59de7d08bf695`. Its directory says `train2017`,
but the original `coco_url` in all six image records says **val2017**. The
177,526-byte annotation file has SHA-256
`760f6d57e617b32f52d687c94806c7e36220726f478564b2202c91c67559e06a`.
Every JPEG hash, original Flickr URL, original COCO URL, dimension and license ID
is recorded in `road-protocol.json` and checked against that file before scoring.

The [official COCO terms](https://github.com/cocodataset/cocodataset.github.io/blob/5e1c4da72464b1c6f068df0c02c91e3000ea62c4/dataset/termsofuse.htm)
state: “The annotations in this dataset along with this website belong to the
COCO Consortium and are licensed under a Creative Commons Attribution 4.0
License.” Credit: COCO Consortium, *Microsoft COCO: Common Objects in Context*
(Lin et al., 2014). The reproduced reference-box metadata in the result report
is an adaptation under CC BY 4.0; RustDriving evaluator code is Apache-2.0.

Original image metadata records CC BY 2.0 for IDs 58636, 226111, 252219, 303818,
and CC BY-ND 2.0 for IDs 174482, 322864. These licenses apply to images, separately
from annotations and code. Only unmodified original JPEGs are opt-in downloads
under ignored `artifacts/`; **no images or altered previews are redistributed**.
The download is approximately 1.2 MB including annotations. The COCO128 archive
examined during discovery was not used: its transformed metadata omitted the
original per-image licensing information.

## Recorded check (2026-10-10 JST)

Actual CPU inference on the pinned portrait returned one person box, confidence
approximately 0.93, with IoU 0.94865 against the independent reference. The single
reference was matched at IoU ≥0.5, with no unmatched prediction. This is a tiny
execution check, not an accuracy benchmark. Detailed measured timings and
fingerprints are in `results.json`; expect timings to vary by host.

## Separate YOLOX-S comparison

An additive `rustdriving-camera-detect-s` binary uses the pinned official
35,858,002-byte YOLOX-S artifact, 640 × 640 input and 8,400-row grid decoding.
Its [explicit source profile](profile-s.json) documents contemporary nonlegacy
raw-BGR preprocessing, Rust Triangle resizing and argmax/classwise NMS differences
from upstream. The nano algorithm, first reports and model identity remain
unchanged. Select `--bin rustdriving-camera-detect` explicitly for the existing
nano `cargo run` command; Cargo now discovers a second binary.

[The fixed S protocol and reproduction interfaces](../../docs/yolox-s-camera.md)
use the same eight already viewed BDD images with unchanged thresholds. The
first complete check runs all images twice on CPU with identical detections:
55 TP, 14 FP and 83 FN / 138 references, precision 79.71% and recall 39.86%.
This improves viewed matches over nano’s 35 while losing one earlier match,
but quality acceptance exits 1 because overall and several class recall gates
fail. Complete replay verifies detection sets within absolute tolerance 1e-5
and exactly reproduces match IDs/counts/gates; its integrity flag is true but
quality exit remains 1. No fresh held-out generalization, real-time performance
or driving integration is claimed.

The separate [bounded multi-view S adapter](../../docs/yolox-tiles-camera.md)
adds up to three overlapping square crops and an independently checked global
merge. It retains all 55 single-view matches and reaches 78/138 viewed
references, with 33 false positives. Recall improves to 56.52%, but the
unchanged overall and per-class quality gates still fail. Select
`--bin rustdriving-camera-detect-tiled-s` explicitly; model-free crop/merge
controls and full repeated native inference are available through
`bdd_tiles_check.py`. No fresh holdout or fitted calibration is claimed.

The optional [vehicle consistency postprocessor](../../docs/yolox-vehicle-context.md)
reads original tiled detector JSON with `rustdriving-camera-reconcile --input
RAW.json --output NEW.json`. It reconciles car/bus/truck conflicts only when
all eligible full-image proposals support one class, and preserves ambiguous
hypotheses. The viewed result retains all 78 reference matches and reduces
false positives from 33 to 31. Earlier majority/full-context failures remain
reproducible. Original overall/per-class recall gates still fail; this is no
fresh holdout, confidence calibration or camera-based driving capability.

For a single image, [one-command analysis](../../docs/camera-analysis.md) runs
the unchanged tiled detector and consistency rule with
`rustdriving-camera-analyze --model FILE --image FILE --output NEW.json`.
It saves the original proposals and final hypotheses together, and publishes
only a complete bounded report without replacing an existing destination.
The same eight-image repeated check retains 78 TP / 31 FP / 60 FN; this is an
execution and failure-handling improvement, with the original quality failure
still explicit.

The optional [recorded-image sequence CLI](../../docs/camera-sequence.md)
reuses one model plan across consecutive images and runs the unchanged
image-space tracker in the same process. `prepare_sequence.py` derives input
paths, hashes and original clock text from an existing cached RGB inventory.
An eight-frame indoor recording reproduces both the single-image detector and
standalone tracker, with no measured object-identity accuracy or driving
integration claim. Select `--bin rustdriving-camera-sequence` explicitly.

The [complete offline camera pipeline](../../docs/camera-pipeline.md) adds the
unchanged two-clock freshness guard to actual image analysis/tracking in one
command. It shares one CPU model plan, skips inference for rejected image
deliveries and retains complete detector and freshness lineage. Its clocks are
caller-supplied replay metadata; live-camera ingestion and measured end-to-end
latency are unimplemented. `prepare_camera_pipeline.py` adds an explicit
authored delivery lag to the existing image sequence without reading images.
Select `--bin rustdriving-camera-pipeline` explicitly.

The [measured serial replay](../../docs/camera-latency.md) uses actual host
monotonic time for analysis and tracking, preserving the original acquisition
schedule. It skips expired work and withholds results that become late during
processing. The first CPU trials take 2.27–2.31 s for one four-view image and
fail the prior 0.2 s limit. Startup is reported separately; the final poll and
audit publication are excluded. This is offline file replay without live sensor
transport. Select `--bin rustdriving-camera-latency` explicitly.

The optional [parallel fixed-view analyzer](../../docs/camera-parallel.md) shares
one immutable CPU plan across at most four scoped view workers, then restores
the original merge order. It offers image analysis and the unchanged measured
replay controller. Sixteen images twice retain all serial detections; this
host's paired processing improves from 2.26–2.39 s to 0.58–0.63 s (~3.9×).
The 200 ms deadline and original quality gates still fail. Its producer receipt
is distinct from the old pinned serial JSON profile. Select
`--bin rustdriving-camera-parallel` explicitly.

The optional [CPU-specific build recipe](../../docs/camera-cpu-build.md) uses
O3, ThinLTO and native CPU features in a separate target directory, preserving
normal releases and all detector code/weights. It records compiler settings,
binary/source hashes and CPU flags, with complete numerical comparison on
16 images twice. Its two-pair mean is 5.94% shorter but one pair regresses;
consistent speedup is unproven and the 200 ms gates still fail. Use
`python3 integrations/onnx/build_cpu.py build --output NEW` from the repository
root after normal dependencies have been cached; this recipe is Linux x86-64 only.

The optional [guarded Resize lowering](../../docs/camera-operators.md) provides
`rustdriving-camera-profile` for operator clocks and full raw-tensor bit
comparison, and `rustdriving-camera-repeat2x` for actual parallel image/sequence
execution. Only the pinned model's two nearest/asymmetric/floor 2× expansions
are lowered, using standard tract operators. Sixteen images twice preserve all
output bits; the latest two measured parallel pairs average 7.14% less processing time.
All original 200 ms and accuracy gates still fail. Build these binaries
explicitly and follow the linked controls/reproduction instructions; no model
weights or detections are changed.
