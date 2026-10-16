# One-command offline camera analysis

`rustdriving-camera-analyze` runs the pinned four-view YOLOX-S detector and the
ambiguity-preserving vehicle consistency rule in one process. One image read
and one reusable CPU model plan produce both the original detections and the
final hypotheses in a single JSON file. No reference annotations enter runtime.
This optional image tool is separate from the driving pipeline.

## Run

Use an existing pinned YOLOX-S download, or follow the model acquisition and
licensing instructions in the [ONNX integration](../integrations/onnx/README.md).
From the repository root:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-analyze
mkdir -p artifacts/camera-analysis
integrations/onnx/target/release/rustdriving-camera-analyze \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --image artifacts/bdd-camera/00a395fe-d60c0b47.jpg \
  --output artifacts/camera-analysis/image-001.json
```

Use a fresh output filename for each invocation. Exit 0 means a complete
report was published; it does not mean detection accuracy passed a gate.
Errors print their stage to stderr and exit 2. The executable receipt path
`--executable-receipt --output NEW.json` records compiled source/profile hashes
and the executable SHA-256 without opening images or weights.

## Output and failure handling

The version-1 `yolox_s_image_analysis_v1` report contains:

- `base_prediction`: the unchanged tiled detector report, including every
  source view, raw RGB hash, model proposal, global merge and source rank.
- `result`: the same vehicle consistency result as the separate
  [report postprocessor](yolox-vehicle-context.md), with retained original
  indices, decisions and unresolved ambiguity.
- `source_image_sha256`, `compiled_sources` and `execution`: input identity,
  source/profile identities, one image read and the actual number of model
  forward passes. The full image plus unique square crops requires 1–4 passes.

The image must be a regular non-symlink JPEG/PNG, at most 16 MiB encoded and
16 million pixels. The model retains its exact size/SHA checks. Confidence
0.3, classwise NMS IoU 0.45 and the reconciliation rule are unchanged.

Before any input read, the command rejects existing output files, directories
and dangling links and reserves a hidden sibling staging file. Serialization
is limited to 4 MiB. It syncs the complete file, then creates the destination
with a hard link that cannot replace an existing path. A concurrent writer's
output is preserved. Handled input, serialization and publication failures
remove staging and do not publish a partial report. The destination's parent
directory must already exist and its filesystem must support hard links.

Validation covers the local Linux filesystem. An abruptly killed process can
leave a `.rustdriving-camera-*.partial` file. The destination is either absent
or contains finished bytes; there is no parent-directory fsync or power-loss
durability guarantee. No cross-platform or real-time claim is made.

## Reproduce the checks

```sh
python integrations/onnx/camera_analysis_check.py controls \
  --binary integrations/onnx/target/release/rustdriving-camera-analyze \
  --output artifacts/camera-analysis-controls-new
python integrations/onnx/camera_analysis_check.py run \
  --binary integrations/onnx/target/release/rustdriving-camera-analyze \
  --controls artifacts/camera-analysis-controls-new \
  --output artifacts/camera-analysis-regression-new
```

The checker uses NumPy and Pillow, checks 15 input/admission failures, and
executes all eight already viewed BDD images twice: 16 native processes and
64 actual model forward passes. It compares raw detector reports and retained
results with the historical two-command path, reconstructs reconciliation
independently, and uses the original matcher for all 138 reference objects.
Local repeats compare f32 identities; historical comparisons retain the
existing absolute 1e-5 tolerance after f32 canonicalization and exact identities.
Three new Rust tests cover a competing publisher, a serialization budget failure
and abandoned staging. An additional native CLI check publishes a receipt when
the requested filename matches the first internal staging name. The publisher
skips that name. Other module tests run again through Cargo's binary
targets and are not counted as newly implemented tests.

Engineering equivalence returns 0 only when these checks pass. The result
explicitly retains the failed original accuracy gates: **78 TP, 31 FP, 60 FN**,
precision **71.56%**, recall **56.52%**. This change improves operation and failure
handling; it does not improve accuracy, provide a fresh holdout, calibrate
confidence or establish automotive operation. The subjective OSS-relative
maturity estimate remains **about 20%**, with the 30% waypoint unmet.

The [evidence index](../assets/camera-analysis-v1/files.json) preserves source
pins, repeated native reports, failure receipts and scores. The first run and
its source archive remain preserved; `publication-repair/` records the reproduced
staging-name failure, its repair and the complete repeated inference check.
Raw BDD images,
original annotations and weights are not redistributed. BDD derivatives carry
the Regents/BAIR research/education/not-for-profit
[data terms](../assets/camera-analysis-v1/LICENSE-data.txt), separately from
RustDriving's Apache-2.0 code and the documented model provenance.
