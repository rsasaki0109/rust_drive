# Safe row-wise Nano depthwise evaluation

The optional `rustdriving-camera-depthwise` command retains the original Nano
weights, four image views, Triangle/BGR preprocessing, score/NMS thresholds,
decoder and vehicle-consistency rules. It adds safe Rust row-wise evaluation for
exactly thirty depthwise convolutions in the pinned, optimized tract graph,
alongside the previously checked two Resize2x lowerings. The original executables
and every earlier source/measurement remain reproducible.

The loader verifies the original ONNX byte/hash pin, then binds the thirty
original Conv names and explicit attributes: 3×3 kernel, channel groups, unit
dilation, pad one and strides one or two. Static single-image NCHW/CHW facts,
finite constant kernel/bias data and contiguous layouts must match the pinned
schema. Unsupported graphs fail before inference. Runtime operands must match
the bound shapes. This path uses the existing tract executor's public evaluation
hook; other nodes use its original evaluator. There is no new dependency,
unsafe code, custom runtime, model quantization or fused multiply-add.

Channel/row traversal makes adjacent pixels contiguous. Each output still starts
with the original bias and adds only valid kernel taps in the original row-major
order. Padding taps are skipped rather than multiplied by zero, preserving
signed-zero behavior. The scalar coordinate oracle checks both strides, edge
positions, random finite operands and signed-zero/subnormal controls. Every
production forward requires all thirty replacement calls.

## Recorded comparison

An initial 16-forward pilot checks four raw output pairs, an original serial
reference and the actual parallel producer. It passes on its first observation
and shows a 12.68% aggregate serial reduction. The pilot is diagnostic; the
subsequent main order is fixed before its timed trials. Every first/repeat result
is retained, with no timing-based retries.

FIFO compares against the existing Nano Resize2x executable, isolating the new
depthwise work. Supervised replay compares against the original unoptimized Nano
supervisor, so its difference includes both Resize and depthwise optimization.
Each schedule uses baseline/optimized/optimized/baseline; all eight timed trials
precede the main serial references and operator profiles. Source acquisitions,
200 ms age, two-sample recovery and fault epochs are unchanged. All eight original
source images stay in availability denominators.

| FIFO trial | Inferred / 8 | Available / 8 | First processing |
|---|---:|---:|---:|
| Resize baseline first | 4 | 1 | 95.46 ms |
| Depthwise first | 4 | 2 | 69.66 ms |
| Depthwise repeat | 4 | 2 | 71.43 ms |
| Resize baseline repeat | 4 | 1 | 92.76 ms |

| Supervised trial | Inferred / 8 | Available / 8 | First worker processing |
|---|---:|---:|---:|
| Original Nano first | 4 | 3 | 83.00 ms |
| Depthwise first | 5 | 4 | 70.11 ms |
| Depthwise repeat | 5 | 4 | 84.37 ms |
| Original Nano repeat | 4 | 1 | 117.85 ms |

Both optimized trials improve availability in this fixed local comparison, but
neither provides all source results. Initial recovery, skipped/superseded images
and expiry remain visible. No inferred image misses its own deadline in these
eight trials; that alone does not establish availability. The optimized
supervisor's maximum periodic poll gaps are 13.63/13.08 ms, exceeding the 10 ms
target. It still drains active work rather than cancelling a hung native forward.
This is an optional offline path without a hard real-time or vehicle-control
claim. These two pairs are not portable hardware performance evidence.

The main operator comparison uses the same sixteen previously viewed TUM/BDD
images, alternating profile/reference order by image. Its original reference
plan already has Resize2x lowering and executes the original depthwise operator.
Fifteen of sixteen paired serial observations are faster; the aggregate forward
time is **9.79% lower**. The `tum-3` pair regresses 4.29% and is retained. These
instrumented serial durations are separate from parallel replay latency.

All 64 main raw-output pairs match **19,306,560 f32 values bit for bit**. Current
original Nano serial detections, every parallel per-view result, global merge
and final decision also match. Independent NumPy postprocessing reconstructs
them, and eight standalone guard replays exactly match every recorded output,
including intermediate monitored states and final expiry.

There are **408 actual model forwards**: 16 pilot + 392 main (136 timed, 128
profile/reference, 64 parallel image and 64 original serial reference). Two new
kernel tests supplement 62 reused native tests; twelve Python regressions reject
corrupted raw hashes, replacement inventories, clocks, f32 measurements,
provenance, readiness and coverage. Twenty-seven fatal-input controls, two output
protections and one model-free supervised guard replay execute no valid model.
The required 371 root tests and 49 scenario replays pass; all 245 core outputs,
137 earlier frozen sources and 6,033 prior indexed evidence entries are unchanged.

The detector's inherited BDD accuracy gates still fail. This adds no new quality
score, unseen recording, identity score or driving integration. Subjective
maturity versus mature driving OSS stays about 20%, below the 30% waypoint. The
new variant has local validation and no remote CI observation.

## Reproduce

Use the unchanged model/data cache in [the Nano instructions](camera-nano.md).

```sh
source scripts/env.sh
python -m pip install -r scripts/requirements-visual.txt
cargo build --release --locked --manifest-path integrations/onnx/Cargo.toml \
  --bin rustdriving-camera-depthwise --bin rustdriving-camera-depthwise-inspect \
  --bin rustdriving-camera-nano-repeat2x --bin rustdriving-camera-supervised \
  --bin rustdriving-camera-nano
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
python -B integrations/onnx/camera_depthwise_reproduce.py \
  --output artifacts/depthwise-example
```

The reproduction command explicitly runs its own 16-forward pilot before the
main comparison. The lower-level `camera_depthwise_check.py run` executes only
the main phase; its prior-pilot counter refers to the preceding session pilot
and must not be treated as additional forwards executed by that main call.
`camera_depthwise_reproduce.py --audit-pilot DIR` audits an existing pilot without
inference. The orchestrator is audited against the saved pilot; a second full
reproduction has not been run locally.
New outputs are required. The native command accepts `--image`, `--profile-image`
(with optional `--reference-first`), `--sequence`, `--latest-sequence` and
`--supervised-sequence` after `--model FILE`, followed by `--output NEW`.
`--inspect-sequence FILE` preflights without a model; `--executable-receipt` binds
the 65-file native source closure. The separate read-only inspector takes the
pinned model path and emits graph metadata without a forward.

```sh
python -B assets/camera-depthwise-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_depthwise.py -v
```

These final checks execute no model and require no original camera images.
