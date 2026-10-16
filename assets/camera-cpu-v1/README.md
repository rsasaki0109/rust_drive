# CPU-specific camera build evidence

An isolated optional Linux x86-64 build compiles the unchanged parallel
fixed-view analyzer with O3, ThinLTO, one codegen unit and native CPU features.
The normal release executable remains the baseline and default. The exact
35 compiled source files, model weights, image geometry, thresholds and final
rules remain unchanged. This packet does not demonstrate a consistent speedup.

The first frozen ABBA comparison, retained in full, gives:

| Trial | First-image processing | 200 ms gate |
|---|---:|---|
| Normal parallel, first | 0.722091085 s | Fail |
| Native CPU, first | 0.606685816 s | Fail |
| Native CPU, repeat | 0.678182959 s | Fail |
| Normal parallel, repeat | 0.643847479 s | Fail |

The ratio of means is 1.0631×, or 5.94% less time on average. One pair improves,
the other regresses; ranges overlap and two samples do not establish reliable
improvement. Every trace infers 1/8 images, skips seven expired acquisitions and
exposes zero available results. The pre-existing 0.2 s age limit is unchanged.
Startup/model loading precede acquisition origin; the final guard poll and JSON
publication are excluded from processing timing. No measured host load,
CPU reservation, peak memory or real-time/live-camera guarantee is provided.

Every original TUM RGB image at indices 102–109 and each of the same eight
BDD development frames runs twice. All view pixels, decoded detections,
ordering, merge proposals and final consistency results exactly match original
saved serial output, excluding durations. The unchanged independent freshness
guard reproduces all four actual timing traces. The packet records 144 model
forwards: 16 for timed traces and 128 for image parity. This is viewed data,
without a fresh holdout or real object-identity score.

Inherited BDD quality stays 78 TP / 31 FP / 60 FN; the original quality gates
still fail. These are prior scored metrics preserved through exact output
parity, without a new label-scoring pass. Sensor transport and vehicle-control
integration are unimplemented. OSS-relative maturity stays about 20%
subjectively; the 30% waypoint remains unmet.

- `before.json`: pre-change source and previous evidence-index hashes.
- `build/`: actual compiler recipe, pre-build declaration, binary/source/CPU
  receipts and build log. Executable binaries and the Cargo target cache are
  intentionally excluded. Successful compilation alone asserts no parity.
- `controls/`: 25 native-binary invalid-input controls, two existing-output
  protections and six build-admission rejection receipts/diagnostics.
- `verification/`: frozen image selection/order/source pins, both binary
  receipts, all first timing results, exact guard replays and 32 full parity
  reports. Build verification binds native executable SHA-256 to the recipe.
- `sequence.json`: unchanged original acquisition strings and relative paths.
- `source/`, `source-sha256.json`: complete current source/document snapshots.
- `checks/`: required 371-test/49-scenario root check and execution logs.
- `environment.json`: non-secret Linux CPU/quota observations after testing.
- `protocol-verification.json`: publication-path replay and previous-evidence
  integrity counts, including unchanged 245 core replay-output hashes.
- `summary.json`, `files.json`: findings and complete packet byte/hash index.

From the repository root, prepare the original images, model and normal
release binaries as described in the [parallel guide](../../docs/camera-parallel.md).
Reproduce with fresh output directories using only standard-library Python:

```sh
source scripts/env.sh
python3 integrations/onnx/build_cpu.py build --output artifacts/camera-cpu-replay/build
python3 integrations/onnx/build_cpu.py verify --output artifacts/camera-cpu-replay/build
python3 integrations/onnx/camera_cpu_check.py controls \
  --build artifacts/camera-cpu-replay/build --output artifacts/camera-cpu-replay/controls
python3 integrations/onnx/camera_cpu_check.py run \
  --build artifacts/camera-cpu-replay/build \
  --controls artifacts/camera-cpu-replay/controls --output artifacts/camera-cpu-replay/verification
```

Dependencies must already be cached for the locked offline native build. Use a
dedicated native `--target-dir` if reusing a cache. Native execution requires
all recorded CPU flags; this is conservative compatibility checking, without
an authenticity signature or portable-binary qualification. Hardware timings
may differ. CI repeats correctness and preserves timing observations without
a speedup requirement; no remote CI run is claimed for this change.

Code is Apache-2.0 under `LICENSE-code.txt`. Derived TUM data retain separate
CC BY 4.0 attribution/source limits in `LICENSE-TUM-data.txt`; BDD derived data
retain Regents research/noncommercial and other terms in `LICENSE-BDD-data.txt`.
No original RGB images, depth, poses or model weights are redistributed here.
See the [CPU build guide](../../docs/camera-cpu-build.md) for the full boundary.
