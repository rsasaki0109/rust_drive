# Acquisition-time motion prediction

The optional Rust tracker now supports `motion_model: constant_velocity` as an
initializer. It predicts a camera pose from the last two **accepted** depth
poses and their actual acquisition intervals. It does not compensate RGB/depth
time offsets; a separate [temporal image option](projective-timing.md) now adds
that compensation with its own controls and recorded comparison. The default
remains `stationary`: complete cross-record accuracy
still fails adoption.

## Recorded outcomes

All runs retain 180 original acquisitions per recording, zero resets, the first
optical-camera root, K=525 and 0.1 m / 0.1 rad gates. All 179 updates are accepted
without expiry in every case. Numeric reference poses are parsed only after
operational outcomes are saved.

| Tracker / initialization | Accurate, room / desk2 | Maximum position, room / desk2 (m) | Maximum rotation, room / desk2 (rad) |
|---|---|---|---|
| Depth / stationary | 64 / 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| Depth / constant velocity | 67 / 159 | 0.298279 / 0.123121 | 0.141606 / 0.090772 |
| Exposure-aware RGB-D / stationary | 84 / 124 | 0.267082 / 0.121676 | 0.102433 / 0.149427 |
| Exposure-aware RGB-D / constant velocity | 75 / 169 | 0.270406 / 0.098134 | 0.117406 / 0.110973 |

The depth candidate fails both recordings' original adoption rule. The RGB-D
candidate improves desk2's count and maximum errors relative to ordinary RGB-D,
but worsens room. Relative to the original depth baseline it passes room's
relative gate, while desk2 still exceeds the unchanged maximum-angle bound.
Neither candidate is adopted. These are already-viewed indoor development
intervals, without held-out or automotive qualification.

Both stationary defaults reproduce all 360 original outputs each, including
equations, model fusion and clocks. The [source-bound packet](../assets/projective-motion-v1/summary.json)
preserves first and corrected source versions, initial numerical-audit failures,
complete corrected ledgers, raw audits, controls and failed adoption decisions.

## Causal model and numerical correction

For accepted optical poses `G0` at `t0` and `G1` at `t1`, the prediction is
`G1 * Exp(Log(G0^-1 * G1) * (t-t1)/(t1-t0))`. This uses a constant **body twist**
on SE(3), not separate world translation scaling and Euler angles. Rotation and
translation are coupled through the exponential's left Jacobian. Small-angle
series avoid cancellation. A near-pi logarithm does not invent a rotation axis.

Only accepted measurement fits advance the two-entry history. A prediction never
becomes history by itself. Duplicate, regressing, malformed or rejected frames
cannot change the accepted poses or renew their clock. The existing 0.2 s expiry
check runs before prediction; expired tracking stays lost. Bootstrap, ambiguous
rotation and a predicted increment exceeding 0.1 m or 0.15 rad fall back to the
previous accepted initializer. All original fitting, support, condition,
iteration, final residual, keyframe and fusion policies remain.

Every prediction reports its two source timestamps, history interval, horizon,
predicted first-camera pose and increment from the latest accepted camera. The
initializer composes the latter directly in the maintained reference frame.
The first implementation instead round-tripped a global prediction through the
reference inverse. Repeated frame conversions amplified floating-point rotation
nonorthogonality; independent checks rejected its initial complete room runs.
The correction removes that round trip and restores the original 1e-11
initializer audit tolerance, rather than weakening it. Maximum rotation
orthogonality error in the corrected four trajectories stays below 1.4e-14.
First sources and unscored operational ledgers are retained.

## Timing diagnostic and its limits

The separate timing diagnostic projects a sparse measured-depth grid using the
previous two accepted exposure-aware RGB-D estimates and the recorded RGB/depth
timestamp difference. It verifies the original manifest, immutable pose ledger
and depth bytes, without decoding numeric reference truth. It runs causally:
current or future fitted poses do not enter a frame's motion estimate.

The maximum absolute association difference is 17.31 ms in room and 17.03 ms in
desk2. Across fresh-image frames with two accepted predecessors, the median
frame's 95th-percentile predicted displacement is 5.93 px / 6.54 px; the largest
frame's corresponding statistic is 15.19 px / 23.35 px. There are 171 / 160 probe
frames. These are model-predicted coordinates under constant body motion and a
static-scene assumption, **not measured timing error or photometric residual**.
The source estimated poses already drift. Equal image/depth phase and motion
can also cancel parts of a relative-image error; absolute displacement alone
does not identify a cause of tracker drift.

The initializer supplies a tested motion-history foundation. It does not warp
source/reference images to their respective exposure times, correct rolling
shutter or supply calibrated velocity uncertainty. No timing compensation or
localization accuracy gain is claimed from this diagnostic.

## Independent controls and reproduction

NumPy reconstructs body motion with a quaternion eigensolve, integrates its
Jacobian by Gauss-Legendre quadrature, and evaluates a four-dimensional matrix
exponential by Taylor scaling/squaring. Rust uses a skew/trace logarithm and
closed-form Jacobian expressions. The auditor checks predicted roots,
increments, accepted-history provenance and the actual initial ICP poses.

A ray-cast helix control uses irregular 10–60 ms intervals and checks 4 mm /
0.004 rad motion gates. It retains valid-size rejected depth, malformed size,
duplicate/regressing clocks, stationary fallback after a large predicted step,
and latched expiry. Four corrupted prediction/history reports are rejected.
Synthetic poses only generate depth and score results; runtime receives depths
and acquisition timestamps. CI runs these and raw analytic checks without
recording downloads. The optional crate passes 41 Rust tests.

The corrected recorded audit reconstructs all 14,908 geometric systems, 7,408
image systems, 716 prediction reports and 716 prescribed model generations,
without decoding numeric pose truth. Together with 371 core Rust tests, 412
Rust tests pass; all 49 scenario/replay checks and 245 deterministic output
hashes remain. These checks validate the implementation, while the recorded
physical accuracy decision remains a failure.

Use the [original input/environment setup](projective-depth-rust.md#reproduction)
and fresh output paths:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-motion.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/motion-controls
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --motion-model constant_velocity --output artifacts/motion-depth
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --photometric keyframe --motion-model constant_velocity --output artifacts/motion-rgbd
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-projective-motion-timing.py \
  --output artifacts/motion-timing
```

Both physical evaluators and adoption decisions save complete failures and exit
1. Use motion-enabled `check-projective-depth.py` reports with the existing raw
auditor, adding `--photometric keyframe` for RGB-D. This is offline research;
vehicle localization, temporal image compensation, loop closure and long-run
accuracy remain unqualified. Subjective maturity stays about 20% versus mature
driving OSS.
