# Optional Rust-native CPU camera detection

This standalone workspace runs the official YOLOX nano ONNX network through
`tract-onnx`, then decodes actual network outputs into 80-class pixel boxes and
per-class nonmaximum suppression. It is offline perception research, separate
from the vehicle pipeline. Boxes have image coordinates and no inferred metric
depth. The default RustDriving build has no model, camera, or runtime dependency.

```sh
python3 integrations/onnx/fetch.py --output artifacts/camera-model
cargo run --release --locked --manifest-path integrations/onnx/Cargo.toml -- \
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
COCO AP. Camera calibration, camera/LiDAR association, synchronization, road
camera evaluation, and control integration are not implemented.

## Recorded check (2026-10-10 JST)

Actual CPU inference on the pinned portrait returned one person box, confidence
approximately 0.93, with IoU 0.94865 against the independent reference. The single
reference was matched at IoU ≥0.5, with no unmatched prediction. This is a tiny
execution check, not an accuracy benchmark. Detailed measured timings and
fingerprints are in `results.json`; expect timings to vary by host.
