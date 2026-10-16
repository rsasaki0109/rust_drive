# CPU-specific camera build comparison

`integrations/onnx/build_cpu.py` builds the unchanged
[parallel fixed-view analyzer](camera-parallel.md) in a separate directory,
with Rust 1.90.0, optimization level 3, ThinLTO, one codegen unit and
`-C target-cpu=native`. Normal release binaries and all model/algorithm sources,
weights, geometry and thresholds remain unchanged. This is an **optional Linux
x86-64 experiment**, without a demonstrated consistent speed improvement.

## Measured outcome

The complete ABBA order was frozen before inference. Both executables share
the exact same 35 compiled source files. These are fresh measurements against
the previous **parallel** implementation, not against the earlier serial one.

| Order | Normal parallel build | Native CPU build |
|---|---:|---:|
| First pair | 0.722091 s | 0.606686 s |
| Repeat pair | 0.643847 s | 0.678183 s |

The ratio of means is 1.063×, or **5.94% less processing time on average**.
The second pair is slower after optimization; the ranges overlap and there
are only two samples per executable. This does **not** establish a reliable
speedup. All four original **200 ms deadline gates still fail**. Each trace
infers one image, skips seven expired images and makes zero results available.
The native build is optional; the normal release recipe remains the default.

Timing uses the exact existing [measured replay controller](camera-latency.md).
Preflight and model loading precede acquisition-clock origin, without inference
warmup. Measured processing covers image read/decode, all four views, merge,
consistency and tracking through the post-tracking checkpoint. Final guard
poll execution and JSON publication are excluded. This is file replay on one
four-CPU-quota host; load, CPU reservation and peak memory were not measured.
These data do not establish live-camera throughput or real-time behavior.

## Build and verify

Prepare the normal release executables, pinned model and original images using
the [parallel-mode instructions](camera-parallel.md). The native recipe uses
the same locked, already cached dependencies offline:

```sh
source scripts/env.sh
python3 integrations/onnx/build_cpu.py build \
  --output artifacts/camera-cpu-example/build
python3 integrations/onnx/build_cpu.py verify \
  --output artifacts/camera-cpu-example/build
```

The output must be fresh. `--target-dir PATH` optionally reuses a native Cargo
build cache; it does not replace the normal target directory. Compilation
records the exact command, compiler version, manifest/lock hashes and overrides
before building. The published receipt binds the executable SHA-256, all 35
compiled source hashes, builder hash and common Linux CPU flags. `verify`
checks these bindings and requires the current CPU to provide all recorded
flags. This check is conservative and is not a portable-binary qualification or
an authenticity signature. A successful build alone does not assert numerical
parity, accuracy or timing success.

Run on the qualified build host after verification:

```sh
artifacts/camera-cpu-example/build/bin/rustdriving-camera-parallel \
  --model artifacts/camera-model-s/yolox_s.onnx \
  --image data/tum-fr3-sitting-visual/raw/rgb-1341845824.165751.png \
  --output artifacts/camera-cpu-example/image.json
```

Image and sequence modes retain the existing bounds, atomic fresh-output
publication, distinct parallel producer identity and pinned legacy-profile
rejection. Build settings are recorded separately from the unchanged detector
source receipts, and execution checks bind each report to its actual binary.

## Exact comparison and retained limits

```sh
python3 integrations/onnx/camera_cpu_check.py controls \
  --build artifacts/camera-cpu-example/build \
  --output artifacts/camera-cpu-example/controls
python3 integrations/onnx/camera_cpu_check.py run \
  --build artifacts/camera-cpu-example/build \
  --controls artifacts/camera-cpu-example/controls \
  --output artifacts/camera-cpu-example/verification
```

The standard-library checker freezes both executable receipts, all source
pins, ABBA order, original eight-image acquisition schedule, 0.2 s policy and
all 16 previously viewed parity images before inference. Every TUM and BDD
image runs twice. All view pixels, raw boxes/confidences, ordering, merged
proposals and final vehicle-consistency results match original saved serial
results **exactly**, excluding timings. Both timed builds also retain complete
source receipts and replay through the separate unchanged freshness guard.
There are 144 actual model forwards: 16 in four measured traces and 128 in
32 image-parity runs. This is repeated development data, without a new holdout.

Twenty-five existing invalid-input cases and two existing-output protections
pass on the native binary. Six new checks reject mismatched executable hashes,
unsupported/empty CPU features, incomplete source maps, changed compiler
recipes and changed builder hashes. No new Rust algorithm tests are claimed.
The required root check passes 371 tests and 49 scenario replays.

Inherited BDD quality remains **78 TP / 31 FP / 60 FN** through exact output
parity. Existing recall/per-class gates fail; no new annotation scoring,
accuracy improvement or real-sequence object-identity score is claimed.
Acquisition transport, automotive validation and vehicle-control integration
are unchanged. Maturity remains **about 20% subjectively**, with 30% unmet.

The [evidence packet](../assets/camera-cpu-v1/files.json) keeps all four first
timings, full parity reports, controls, build metadata and source snapshots.
The CPU CI job repeats build/correctness checks and retains timing observations,
without a hardware-specific speedup requirement; no remote run is claimed.
Separate [TUM CC BY 4.0 terms](../assets/camera-cpu-v1/LICENSE-TUM-data.txt) and
[BDD Regents terms](../assets/camera-cpu-v1/LICENSE-BDD-data.txt) apply to derived
data. Original images, depth, poses, model weights and executable binaries are
not redistributed in the packet.
