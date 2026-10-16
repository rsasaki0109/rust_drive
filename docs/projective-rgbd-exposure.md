# Exposure-aware fused RGB-D tracking

The maintained optional RGB-D tracker now estimates brightness gain and offset
for each image equation and removes both nuisance directions from its pose
system. Exposure changes no longer need to become camera motion in the
synthetic controls. This does not establish accurate continuous localization.
The depth-only default and driving pipeline remain unchanged.

The separate [fixed-support step-control experiment](projective-step-control.md)
regresses recorded accuracy and defaults to disabled. Ordinary `keyframe` mode
retains the exposure-aware results on this page.

## Complete recorded outcomes

| Tracker | Accurate, room / desk2 | Maximum position, room / desk2 (m) | Maximum rotation, room / desk2 (rad) |
|---|---|---|---|
| Depth-only default | 64 / 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| Preceding RGB-D | 80 / 75 | 0.275569 / 0.306636 | 0.109385 / 0.201632 |
| Exposure-aware RGB-D | 84 / 124 | 0.267082 / 0.121676 | 0.102433 / 0.149427 |

Every run accepts all 179 updates in each recording, without expiry or reset.
Exposure correction improves both already-viewed RGB-D results and all their
maximum errors. It still fails the original adoption rule: desk2 remains below
161 accurate, and its maximum errors remain worse than depth-only tracking.
The mode stays optional. No complete physical accuracy pass is claimed.
The current default reproduces all 360 original native operational outcomes
exactly. These indoor development intervals are not held-out validation.

The [source-bound packet](../assets/projective-affine-v1/summary.json) retains
the original diagnostic, first implementation and symmetry failure, corrected
ledgers, independent audit, controls, source versions and adoption decision.

## Diagnosis and implementation

The preceding [uncorrected RGB-D result](projective-rgbd-fusion.md) scores
80/179 room and 75/179 desk2 accurate updates. Reconstructing its measured image
pairs finds mean residual magnitudes up to about 32 gray levels. Median fine
image/geometry Hessian diagonal ratios exceed one in five of six pose axes in
both recordings. These are diagnostics at the estimated poses, with changing
support: motion misalignment, occlusion and exposure are confounded. They do
not prove exposure is the unique cause of the desk2 regression.

The correction models reference intensity as `gain * source + offset`.
Geometrically admitted samples still require image gradient at least one gray
level/pixel. Initial ordinary least squares is followed by three Tukey-weighted
least-squares fits, with the existing 30-gray cutoff applied to the corrected
residual. Gain must lie in [0.5, 2], absolute offset must not exceed 80 gray
levels, weighted source variance must be at least 25 gray², and at least 200
corrected samples must survive. An unavailable or out-of-bounds exposure fit
contributes no image equation; original geometric admission still applies.

The pose Hessian and gradient use the Schur complement of both brightness
parameters. Merely fixing a fitted exposure while differentiating pose would
invent information in image directions explained by brightness. Centered source
intensities keep the nuisance normal matrix diagonal. Gain, offset, candidate
count and weighted variance are reported with each image system. Exposure is
re-estimated at each pose and pyramid level; it is not persisted or averaged
into the reference image. All original phase, freshness, depth fusion, solver,
iteration, displacement, residual and expiry policies remain, including the
same image/geometry weight. No recording-specific switches or parameter sweep
are used.

The first implementation's complete room operational ledger was retained when
the independent symmetry check found floating-point asymmetry in a large image
Hessian. Computing `weight * (Ja * Jb)` makes mirrored products identical before
the Schur subtraction. The correction keeps the existing audit tolerance;
the first source and binary remain archived.

## Independent controls

The Rust crate passes 37 tests. The exposure checker additionally runs four
positive plane controls: stationary darkening, moving darkening, moving
brightening, and moving exposure with deterministic image outliers. Every
accepted motion must remain within 4 mm / 0.004 rad and recover the known
gain/offset. Four negative controls cover out-of-bounds gain, out-of-bounds
offset, uniform intensity, and a gradient whose apparent motion can be explained
by brightness. Rejections cannot update the model or accepted-pose clock.
The archived preceding binary fails all four positive controls at the same
4 mm gate: translation errors range from 6.96 to 89.03 mm, including 7.60 mm
of false motion for a stationary camera. The corrected binary passes all four.

NumPy independently projects the weighted pose Jacobian out of the brightness
column space using SVD least squares, rather than copying the Rust centered
Schur arithmetic. It reconstructs the controls' reported raw systems and checks
gain, offset and fitting support. The usual four ray-cast motion controls and
ten-frame asynchronous reference lifecycle still pass. Raw analytic mutation
controls reject altered exposure gain/support as well as image-gradient,
freshness, geometry and fusion tampering. CI runs these synthetic checks without
recording downloads.

Complete recorded reconstruction passes for **7,963 geometry systems and 7,757
image systems**, including unused image equations, and all 358 prescribed depth
model generations. This audit never decodes numeric pose truth. Root checks
retain 371 tests, 49 scenario/replay checks and all 245 deterministic output
hashes. With 37 optional-crate tests, 408 Rust tests pass. No historical source
or evidence hashes are changed.

## Reproduction and scope

Use the existing [input/environment setup](projective-depth-rust.md#reproduction)
and fresh output paths:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-rgbd-exposure.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/exposure-controls
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --photometric keyframe --output artifacts/exposure-recorded
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --photometric keyframe --output artifacts/exposure-analytic
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --photometric keyframe --analytic-reports artifacts/exposure-analytic \
  --recorded-reports artifacts/exposure-recorded --output artifacts/exposure-raw
```

Physical scoring retains the same 180 acquisitions per recording, K=525 camera,
first optical-camera root, 0.1 m / 0.1 rad gates, all 179 updates and zero resets.
Numeric pose truth is decoded only after operational ledgers are saved.
The adoption rule still requires room >64 and desk2 ≥161 accurate, all updates
accepted without expiry, and neither recording's maximum error worsening.
The evaluator saves complete failures and exits 1 when full accuracy fails.

Global affine intensity is only an approximation. It does not compensate
RGB/depth time offsets, rolling shutter, clipping, spatial lighting changes,
occlusion or moving objects. A correct local equation and synthetic invariance
do not establish root-pose accuracy, vehicle integration or automotive validity.
Subjective maturity stays about 20% versus mature driving OSS.
