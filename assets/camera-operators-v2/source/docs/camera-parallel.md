# Parallel fixed-view camera inference

`rustdriving-camera-parallel` runs the existing fixed image views concurrently
through **one shared immutable CPU model plan**. It uses at most four scoped
standard-library worker threads, with independent tract execution state per
view. Results return in original view order before the unchanged merge and
vehicle-consistency rules run. Model bytes, image geometry, f32 arithmetic,
score/NMS thresholds and previous executables remain unchanged.

On this four-CPU-quota Linux environment, paired first-image processing gives:

| Order | Serial processing | Parallel processing | Age limit |
|---|---:|---:|---:|
| First pair | 2.260491 s | 0.625560 s | 0.200 s |
| Repeat pair | 2.394728 s | 0.578763 s | 0.200 s |

The full measurement order is serial, parallel, parallel, serial (ABBA), fixed
before inference. The ratio of means is **3.865×**, a **74.13% reduction** in
processing time. All four deadline gates still fail. Every run infers one
image, skips seven expired images and exposes zero available results. Startup
and model loading precede the measured acquisition schedule; this comparison
uses the same [measurement boundary](camera-latency.md) in both implementations.
The final poll and audit JSON publication are excluded. These local measurements
are not a real-time guarantee or a live-camera benchmark.

## Run image analysis or measured replay

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-parallel
mkdir -p artifacts/camera-parallel-example
integrations/onnx/target/release/rustdriving-camera-parallel \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --image data/tum-fr3-sitting-visual/raw/rgb-1341845824.165751.png \
  --output artifacts/camera-parallel-example/image.json
python3 integrations/onnx/prepare_sequence.py \
  --dataset data/tum-fr3-sitting-visual --first 102 --count 8 \
  --camera-id tum-fr3-sitting-rgb \
  --output artifacts/camera-parallel-example/sequence.json
integrations/onnx/target/release/rustdriving-camera-parallel \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --sequence artifacts/camera-parallel-example/sequence.json \
  --output artifacts/camera-parallel-example/measured.json
```

Use fresh outputs and the existing [model/image preparation](camera-latency.md).
`--inspect-sequence FILE --output NEW` verifies every image and original release
offset without loading a model. `--executable-receipt --output NEW` records
executable/source hashes without model/image reads. The measured input is the
original image-sequence schema, with decimal acquisition strings, at most
32 events and a 2-second source interval. The existing 0.2-second freshness
policy and two-sample recovery remain fixed.

## Determinism, bounds and compatibility

Each worker uses the unchanged crop, pixel transform, model inference and
decoder. One read-only detector/plan is borrowed across scoped threads; tract
creates separate per-invocation state. A square image with one unique view runs
on the calling thread. Other shapes start at most four workers. Concurrency
increases simultaneous image/scratch memory; no peak-memory benchmark is claimed.
There is no new dependency, lockfile change, thread-pool runtime or ROS requirement.

Every started worker is joined on success, inference error, spawn failure or
panic. Failed work publishes no partial report. Results are assembled by their
original rectangle indices, preserving ranks, deterministic tie-breaking,
pixel hashes, boxes/confidences and downstream decisions. Internal typed view
measurements feed the existing merge/consistency functions directly.

The image producer has its own algorithm and source receipts:
`yolox_s_parallel_image_analysis_v1`, with
`pinned_yolox_s_parallel_fixed_views_v1` base predictions. Per-view detector
receipts remain unchanged. Execution metadata records worker count, actual
forwards, one shared plan and analysis wall duration. The previous pinned
serial JSON admission deliberately rejects this new producer identity; no
serial provenance is rewritten. Use this binary's sequence mode for its direct
typed connection to the unchanged tracker/freshness guard.

Sequence mode reuses the **exact existing measured replay controller** under
the new analyzer binding. Its top-level `yolox_s_measured_serial_replay_v1`
format still describes serial frame scheduling; view execution is concurrent
and explicitly identified in nested analyzer metadata/source hashes. All
original clock, recovery and deadline behavior remains in that shared source.

Existing 16 MiB encoded-image / 16-million-source-pixel limits and the 4 MiB
`AtomicReport` bound apply. Outputs are reserved before source reads and only
complete synced bytes are published without replacement. Handled errors exit
2 and remove staging; abrupt termination can leave hidden staging. No
power-loss durability or cross-platform result has been established.

## Verification and quality

```sh
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python3 integrations/onnx/camera_parallel_check.py controls \
  --output artifacts/camera-parallel-controls-new
python3 integrations/onnx/camera_parallel_check.py run \
  --controls artifacts/camera-parallel-controls-new \
  --output artifacts/camera-parallel-verification-new
```

The checker needs only Python's standard library, the pinned model and the
already cached original images. It freezes the ABBA comparison and all 16
previously viewed parity images: TUM RGB indices 102–109 and the same eight
BDD road frames. Every image runs twice. All per-view pixels/detections,
ordering, merge results, final hypotheses, ranks, vehicle choices and work
counts match immutable serial output exactly, excluding measured durations.
There are **144 actual model forwards** in this packet: 16 from four timed
replays and 128 from 32 image-parity executions.

Each actual timing trace is independently audited from measured checkpoints
and replayed through the separate unchanged guard. Parallel frame wall time
covers the longest individual view; view timing intervals overlap. Five new
Rust tests cover concurrent completion ordering, complete cleanup after worker
errors/multiple panics, square-image execution and invalid worker budgets.
The binary's 39 test executions include 34 existing shared tests. Twenty-one
existing source/model rejection controls, four image-entry failures and two
existing-output protections pass. A further control confirms that the new
producer cannot enter the old serial-report profile.

All BDD results match the existing scored results exactly. The inherited
**78 TP / 31 FP / 60 FN** and failed original accuracy gates remain unchanged;
these counts are preserved through exact output parity, without parsing new
annotations or measuring improved accuracy. These are viewed development
images, with no new holdout or real-sequence identity score.

The [evidence packet](../assets/camera-parallel-v1/files.json) preserves raw
timings, paired plans, full image reports, replays, controls and source pins.
The CPU research CI job repeats correctness and timing observations; speedup
and deadline outcomes are preserved as hardware-dependent observations. No
remote run has been observed for this change. Derived data retain separate
[TUM CC BY 4.0 terms](../assets/camera-parallel-v1/LICENSE-TUM-data.txt) and
[BDD Regents terms](../assets/camera-parallel-v1/LICENSE-BDD-data.txt);
images, poses, depth and weights are not redistributed. Detection accuracy,
sensor transport and vehicle integration are unchanged. OSS-relative maturity
remains **about 20%**, with the 30% waypoint unmet.
