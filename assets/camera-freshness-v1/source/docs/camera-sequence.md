# Recorded images through detection and tracking

`rustdriving-camera-sequence` runs the unchanged YOLOX-S image analyzer and
image-space tracker together. It loads **one CPU model plan per image sequence**,
keeps the original acquisition times and publishes every detector report and
tracking snapshot in one completed JSON file. This is an optional offline
camera tool, separate from vehicle controls.

## Run an existing recording

The example uses the locally cached TUM Freiburg 3 `sitting_xyz` recording.
If the cache is absent, its existing opt-in fetch command is
`python3 scripts/fetch-visual-datasets.py tum-fr3-sitting-visual`.
Use the pinned YOLOX-S model described in the
[ONNX integration](../integrations/onnx/README.md).

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-sequence
mkdir -p artifacts/recorded-camera
python3 integrations/onnx/prepare_sequence.py \
  --dataset data/tum-fr3-sitting-visual --first 102 --count 8 \
  --camera-id tum-fr3-sitting-rgb \
  --output artifacts/recorded-camera/sequence.json
integrations/onnx/target/release/rustdriving-camera-sequence \
  --inspect-sequence artifacts/recorded-camera/sequence.json \
  --output artifacts/recorded-camera/inspection-new.json
integrations/onnx/target/release/rustdriving-camera-sequence \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --sequence artifacts/recorded-camera/sequence.json \
  --output artifacts/recorded-camera/result-new.json
```

The preparation helper selects consecutive original RGB indices, verifies
cached bytes against the existing inventory and copies timestamp text from
`rgb.txt`. It neither decodes images nor reads depth or motion-capture poses.
It never downloads missing files or chooses frames using detections. Recreating
the example in the same directory layout reproduces the frozen sequence bytes.
The inspection command is optional and reads images without loading a model.

## Input, validation and output

The version-1 sequence declares `camera_id`, fixed `width`/`height`, optional
tracker `config`, and 1–32 `events`. Each event supplies `timestamp_s` as a
**decimal string**, plus one of these `input` objects:

| `kind` | Additional fields | Behavior |
|---|---|---|
| `image` | `path`, `sha256` | Detect the original RGB image and update tracking. Paths are relative to the sequence file unless absolute. |
| `missing` | none | Advance the clock using the existing coasting/expiry policy. |
| `failed` | none | Retire active tracks immediately using the existing failure policy. |

Timestamp strings preserve the original source text; Rust parses them directly
to f64 seconds before checking the monotonic clock. Output keeps both text and
numeric time. All existing [tracker limits and assumptions](image-tracking.md)
remain active, including the bounded repeated-image check. Camera identity
and capture times are caller declarations, not authenticated sensor evidence.

Before model loading, preflight decodes and verifies every image's SHA and
dimensions and validates clocks, configuration and repeated identities. It
keeps one decoded source image at a time. For inference it reads each image
again and verifies the same SHA/dimensions, avoiding retention of the entire
decoded recording. Eight image events therefore perform eight preflight reads
and eight inference reads. They share one plan and perform 32 model forwards
under the unchanged four-view profile. Missing/failed-only sequences open no
model. No depth, reference pose or object-identity annotations enter runtime.

`yolox_s_sequence_tracking_v1` output records input/source hashes, effective
configuration and execution counts. Every event preserves its original
single-image `analysis` report and a `tracking` snapshot. It compiles the exact
independent tracking sources alongside the existing detector sources, with the
unchanged ONNX workspace lockfile. The standalone tracker and old executables
remain independently usable; no new dependency or custom runtime is required.

Sequence JSON is limited to 512 KiB, images to the existing 16 MiB/16-million-
pixel budgets and the final report to **4 MiB**. A long valid sequence can still
exceed the aggregate report budget. Fresh output is reserved before source
reads; only a completed bounded report is published without replacement.
Handled errors exit 2 and remove staging. Exit 0 is execution success, not an
accuracy gate. Abrupt termination can leave hidden staging; filesystem behavior
is locally verified on Linux without a power-loss durability guarantee.

## Recorded verification

The fixed development slice is the first **eight consecutive original RGB
indices 102–109** associated with the already viewed TUM sitting recording:
1341845824.165751–1341845824.398039 s, a **0.232288-second** indoor interval.
It was selected from timestamps/inventory before new detector inference.
No image or detection-based window search was performed.

The complete slice runs twice with identical detections and tracking snapshots
after excluding only measured model-load/inference times. Every image also runs
through the existing single-image CLI, reproducing its complete detector output.
Exported reports run through the standalone tracker, reproducing every snapshot.
An independent residual min-cost-flow solver verifies association optimality;
its results are checked against exhaustive enumeration on 96 small graphs.
The existing independent reconstruction checks motion, phases and lifetimes.
This performs **96 actual model forwards**: 64 from the two sequence runs and
32 from the eight separate image-parity runs.

Nineteen bad-input cases are rejected, including late image corruption before
model access, wrong dimensions/hashes, clock reversal, nonfinite timestamps,
repeated images, special files, manifest budgets and invalid model size. Two
new Rust tests cover original epoch timestamp precision and invalid clocks;
existing shared-module tests execute again and are not new tests.

The [evidence index](../assets/camera-sequence-v1/files.json) preserves raw
detector proposals, all tracking states, source pins, failures and the exact
selection. Original RGB/depth images, poses and weights are not redistributed.
Derived data carry [TUM attribution and CC BY 4.0 terms](../assets/camera-sequence-v1/LICENSE-data.txt),
separately from RustDriving's Apache-2.0 code. Original image bytes come from
the pinned mirror recorded in the existing dataset inventory; official-archive
byte identity is not independently established.

There are no independent object-identity labels for this interval. Confirmed
IDs and reproducibility are not accuracy measurements. Automotive tracking,
long-duration availability, calibrated uncertainty, metric projection and
vehicle integration remain unverified. The subjective OSS-relative maturity
estimate stays **about 20%**, with the 30% waypoint unmet.
