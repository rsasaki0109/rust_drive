# Relative axial-depth weighting

The optional native tracker now implements `depth_variance_plane` and
`depth_whitened_plane`. They test a depth-dependent measurement-weight hypothesis
and its consistent standardized Huber loss. Both improve viewed room and RPY
counts but substantially regress desk2. Neither meets adoption: the original
target-plane default remains, and maturity stays **about 20%**, with 30% unmet.

## Distance-stratified diagnosis

Fifteen fixed consecutive raw-depth pairs per recording, at selected offsets
1,13,…,169, are probed in both directions at native resolution. Mocap supplies
only the offline probe motion. Original pixel projection, border, normal and
distance gates remain; the independent sampler agrees with every original
point-to-plane equation. No reference motion enters the native tracker.

The table reports `1.4826 * median(abs(r-median(r)))` in millimetres. Bins use the
mean of the source and target axial depths, not radial distance from the camera.

| Viewed recording | Z 0.3–1 m | Z 1–2 m | Z 2–3 m | Z 3–5 m |
|---|---:|---:|---:|---:|
| Room | 2.176 | 4.341 | 13.903 | 15.949 |
| Desk2 | 2.869 | 6.361 | 10.610 | 25.176 |
| RPY | 2.972 | 5.772 | 10.155 | 12.485 |

All four bins increase monotonically in these three viewed recordings. The
far-to-near dispersion ratios are 7.33, 8.78 and 4.20. Desk2's far bin has only
1,335 matches, versus over 45,000 in room and 82,000 in RPY. These correlated
pixels/pairs and differing surfaces are not independent noise observations.
The residual also includes association, calibration, reference-motion and
surface errors. A separately disclosed channel divides the residual by source
axial incidence only where that factor exceeds 0.5; the original channel is
retained. This diagnosis motivates an experiment, not a fitted or calibrated
physical noise law.

## Fixed relative variance and robust loss

For axial depth `z=fb/disparity`, constant disparity variance implies an axial
variance proportional to `z^4`. The frozen engineering hypothesis uses

```
v = max(1, (zs^4 + zt^4) / 2)
relative = 1/v
```

Here `zs` is the original source depth and `zt` the matched maintained-model
depth, each in its own optical frame. The 1 metre floor and original 10 mm Huber
threshold were fixed before the candidate outcomes. Valid depths keep relative
weights between 1/625 and 1. The depth values remain fixed for each normal
equation; they are not replaced with the fitted point's transformed Z.

`depth_variance_plane` multiplies the original Huber weight by `1/v`.
Its initial trial scores are preserved. The separately frozen revision
`depth_whitened_plane` applies Huber to `r/sqrt(v)`, giving

```
w = (1/v) * min(1, 0.01*sqrt(v)/abs(r))
J = [nt, q cross nt]
H = sum(w * J^T J), g = sum(w * J^T r)
```

The native revision supplies a ratio of robust multipliers to the existing
accumulator; NumPy independently computes this direct weight. No parameter is
fitted to pose-error scores, and the first trial is not overwritten.

Counts and **raw, unweighted geometric RMS** retain their original definition.
All support, distance, normal, physical-information, solver, increment, final
RMS, fusion, keyframe and accepted-clock gates remain. Lower matrix weights
cannot make the reported physical residual smaller. A weak plane still rejects
and cannot renew the accepted pose or bypass latched expiry.

This scalar proxy omits normal/ray incidence in actual point-to-plane variance,
fused-model and history uncertainty, spatial bias and correlation. The target
model and source observations are not independent samples with known variances.
The modes supply neither calibrated covariance nor a probabilistic error bound.

## Complete viewed outcomes

All runs use the same 180 original depth acquisitions, indices 100–279, one first
optical-camera origin, K525, depth/5000 and all 179 updates. Root gates remain
0.1 m / 0.1 rad; numeric reference poses are scored only after the complete
recording's sensor-only ledger is saved. RPY is now viewed regression data.
The [registered-grid camera contract](registered-grid-camera.md) and
[original TUM source attribution](tum-source-provenance.md) remain in force.

| Mode | Room / desk2 / RPY accurate updates, each out of 179 |
|---|---|
| Original target plane | 64 / 161 / 74 |
| Relative variance | 79 / 74 / 115 |
| Standardized Huber | 77 / 75 / 107 |

| Mode and recording | Maximum position error (m) | Maximum rotation error (rad) |
|---|---:|---:|
| Original, room | 0.296290 | 0.141286 |
| Relative variance, room | 0.275240 | 0.130593 |
| Standardized, room | 0.280087 | 0.133846 |
| Original, desk2 | 0.114800 | 0.090386 |
| Relative variance, desk2 | 0.311419 | 0.305761 |
| Standardized, desk2 | 0.285187 | 0.274443 |
| Original, RPY | 0.087348 | 0.173375 |
| Relative variance, RPY | 0.094389 | 0.128127 |
| Standardized, RPY | 0.093976 | 0.133401 |

Every candidate accepts all 179 updates without expiry or reset, but every
complete accuracy protocol fails and exits 1 after saving its results. The
predeclared adoption rule requires room above 64, desk2 at least 161, RPY at
least 74, complete tracking and no worse maximum position/rotation error in all
three. Both fail. Distance-dependent weighting alone does not consistently
repair this tracker. No new recording, easier selection or relaxed accuracy
gate replaces these development failures.

## Independent checks and scope

Native tests check the fourth-power law, symmetric endpoints and floor bounds;
all six pose derivatives for ordinary weighted and standardized Huber costs;
and exact 1/256 matrix scaling for a known 4 m plane while keeping raw RMS and
support unchanged. Independent NumPy matches are checked against the original
equations during diagnosis, then used to reconstruct candidate equations and
fused models from original PNGs without numeric reference-pose reads.

A weak-plane control reconstructs the weighted failure witness, rejects both
unweighted-matrix substitution and falsely attenuated raw RMS, and checks atomic
accepted clocks and expiry. A 12-frame ray-cast scene adds fixed Gaussian
inverse-depth noise, seed 1701 and standard deviation 0.001 per metre. Both
candidates and the original meet the preregistered 10 mm / 0.01 rad control
gates and accept all 11 updates. Its maximum errors are:

| Synthetic mode | Position (mm) | Rotation (mrad) |
|---|---:|---:|
| Original | 3.410 | 4.768 |
| Relative variance | 3.894 | 1.668 |
| Standardized Huber | 3.705 | 2.339 |

All control equations are independently reconstructed. Injecting the assumed
noise law checks conditional behavior; it does not independently establish that
law for a physical camera. Position error slightly worsens in both controls,
despite their smaller rotation errors.

The [source-bound evidence packet](../assets/projective-range-v1/summary.json)
retains the diagnostic, both designs, original sources, complete ledgers and
failed decisions. Earlier source/evidence archives remain intact. CI runs the
synthetic, raw and transport checks without downloading recorded datasets.

All 424 Rust tests (53 optional-native and 371 core), Clippy, formatting and 49
core scenario/replays pass. The 245 core outputs remain unchanged. All 1,440
historical depth, affine RGB-D and motion/time-compensated sensor outcomes
reproduce exactly, without numeric reference-pose reads. Independent checks of
the six candidate recordings reconstruct 19,838 fitted/returned normal systems
and 1,074 model generations. These integrity checks pass while both adoption
decisions and all complete recorded accuracy protocols fail.

## Interface and reproduction

Select `TrackerConfig.residual` or JSON `Configure.residual` explicitly as
`depth_variance_plane` or `depth_whitened_plane`. Both require
depth/point/forward/central-difference policies without photometric tracking.
Unsupported combinations reject before configuration acknowledgment. The default
and successful report schema retain their original behavior. RGB timing, camera
calibration, root transforms, vehicle driving and Cargo dependencies are unchanged.

With the existing pinned original data and NumPy/Pillow environment:

```bash
source scripts/env.sh
cargo test --all-targets --locked --manifest-path integrations/projective-depth/Cargo.toml
cargo build --release --bins --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-projective-range.py \
  --output artifacts/range-diagnosis
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --residual depth_whitened_plane --output artifacts/range-analytic
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-range.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --residual depth_whitened_plane --output artifacts/range-controls
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --residual depth_whitened_plane \
  --case tum-fr1-room-temporal --case tum-fr1-desk2-independent \
  --case tum-fr1-rpy-heldout --output artifacts/range-recorded
# Expected exit 1: complete physical accuracy fails.
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --analytic-reports artifacts/range-analytic --residual depth_whitened_plane \
  --recorded-reports artifacts/range-recorded \
  --case tum-fr1-room-temporal --case tum-fr1-desk2-independent \
  --case tum-fr1-rpy-heldout --output artifacts/range-raw
```

Use fresh paths and replace the mode with `depth_variance_plane` to repeat the
first candidate. The original frozen diagnostic source is retained because the
maintained independent dispatcher subsequently gained both experimental modes.
