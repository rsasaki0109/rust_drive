# Measured RGB constraints for fused-depth tracking

This page preserves the original RGB-D comparison. The maintained `keyframe`
mode now includes [exposure gain/offset elimination](projective-rgbd-exposure.md),
scoring 84/179 room and 124/179 desk2. It still fails default adoption. The
original implementation and complete 80/75 outcomes below remain archived.

The maintained Rust fused-depth tracker now accepts an explicit, optional
**keyframe photometric mode**. It combines measured image intensities with the
existing fused depth surface. A textured-plane control recovers motion that
depth alone cannot observe. **The mode is not adopted as the default:** room
improves, but desk2 accuracy regresses. All results remain offline research.

## Complete outcomes

Both recordings retain the original 180 acquisitions, first optical-camera root,
K=525 calibration, 0.1 m / 0.1 rad gates and all 179 updates in the denominator.
Numeric reference poses enter scoring only after operational outcomes are saved.

| Tracker | Accepted, room / desk2 | Accurate, room / desk2 | Maximum position, room / desk2 (m) | Maximum rotation, room / desk2 (rad) |
|---|---|---|---|---|
| Original depth | 179 / 179 | 64 / 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| First RGB-D: repeated image rejects the bundle | 172 / 160 | 80 / 21 | 0.274393 / 0.801365 | 0.112944 / 0.329311 |
| Maintained RGB-D: independent image freshness | 179 / 179 | 80 / 75 | 0.275569 / 0.306636 | 0.109385 / 0.201632 |

No run expires or resets. The corrected mode restores new-depth availability
when RGB acquisitions repeat, improves room's count and maximum errors, but
still fails the unchanged adoption rule because desk2 is worse. The default
reproduces all 360 original outcomes exactly. These are already viewed indoor
development recordings, without held-out or automotive qualification.

The [source-bound packet](../assets/projective-rgbd-v1/summary.json) and
[validation receipt](../assets/projective-rgbd-v1/validation.json) preserve the
first failure, corrected sources, complete pre-label ledgers and scored results.
The first strict-image attempt has no complete raw-image equation audit; the
maintained mode receives full independent reconstruction.

## Measurements and model ownership

`Photometric::Keyframe` requires an associated `ImageObservation` with a
grayscale byte per depth pixel and its own acquisition timestamp. RGB PNGs in
the viewed trials are converted by `(77*R + 150*G + 29*B) >> 8`. Image dimensions
match the camera; values are bytes. RGB/depth association must be within 0.02 s.
Depth clocks remain strictly increasing and govern accepted-pose expiry.

RGB clocks may repeat while depth advances: the two recordings have 173 and
162 unique RGB acquisitions for 180 depth frames each. A repeated image supplies
**no intensity constraint**. A newly measured depth can still update pose after
the original geometric support, conditioning and residual checks pass.
Regressing, missing, malformed or unassociated RGB rejects the configured bundle
without updating the accepted pose. A duplicate depth cannot refresh tracking,
and new RGB cannot hide accepted-depth expiry.

The reference color is a raw measured image at a depth-reference change; it is
not averaged into a synthetic image. If a reference changes without fresh RGB,
the color anchor is cleared. Geometry remains usable, and intensity resumes
only after a later reference change has a fresh associated image. This avoids
assigning an old image to a new camera view. Pose labels never enter this logic.

The existing sampler and grayscale pyramid are shared with the older separate
multi-reference experiment. The new mode uses the maintained fused-depth model,
one measured color reference and the original fine sampling stride of 2.
It is separate from the older raw-keyframe multi-reference tracker.

## Joint fit and limitations

Each photometric term uses a geometrically admitted source point and bilinear
reference intensity, with explicit right/down derivative conventions at pixel
knots. It contributes only if RGB/depth phase differs from the reference phase
by at most 0.005 s, the image is fresh, gradient magnitude is at least 1 gray
level/pixel, absolute intensity residual is below 30 gray levels, and at least
200 photometric samples survive. Tukey weights use that same 30-level cutoff.

Weights reuse the previously defined 0.005 m geometric and 10-gray intensity
scales: the photometric H/g contribution is multiplied by `0.005² / 100`
relative to the original geometric system. Separate H/g, counts, residuals and
usage flags are reported; the solve and scaled condition use their combined
system. All original depth gates, iteration caps, keyframe thresholds, root
composition, atomic failures and 0.2 s expiry remain. No dependencies, Cargo pins,
vehicle localization or controls change. Other experimental fitting policies
and XYZ geometry are rejected when combined with this mode.

These weights and phase bounds are provisional. Phase qualification does not
compensate camera motion between RGB and depth, rolling shutter, exposure change
or dynamic objects. Small geometric or image RMS does not establish root
accuracy. The first diagnostic identifies accumulating rotation error in the
viewed desk2 interval; it does not prove a unique cause. Adding measured texture
does not resolve the complete physical protocol.

## Validation and reproduction

The optional crate passes **35 tests**, including all six image Jacobian axes,
textured-plane motion that geometry rejects as unobservable, and repeated-image
freshness. The original four ray-cast motion controls pass their 4 mm / 0.004 rad
gates. A separate ten-frame lifecycle control checks reference clearing and
resumption, independently reconstructs its systems, rejects five input faults
and verifies latched expiry.

The NumPy raw auditor reconstructs the maintained mode's depth and image systems
from hash-qualified PNGs, checks separate and combined contributions, image
freshness/phase/reference ownership and every prescribed depth model update.
This includes 7,995 reported geometric systems, 7,383 image systems and 358
prescribed model generations. Reference changes without fresh RGB clear the
color anchor three times in room and five times in desk2; their subsequent
image-free reference intervals are included in the audit.
Altered image gradients, freshness metadata and joint H/g rescaling are rejected.
The original attempt and current sources remain versioned rather than changing
historical hashes. Core checks retain 371 Rust tests, 49 scenario replays and
245 deterministic output hashes. CI runs synthetic RGB-D, lifecycle and raw
analytic checks without downloading recordings.

Use the [original input/environment instructions](projective-depth-rust.md#reproduction)
and fresh output paths:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --photometric keyframe --output artifacts/fused-rgbd-recorded
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --photometric keyframe --output artifacts/fused-rgbd-analytic
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-rgbd-lifecycle.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/fused-rgbd-lifecycle
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --photometric keyframe --analytic-reports artifacts/fused-rgbd-analytic \
  --recorded-reports artifacts/fused-rgbd-recorded --output artifacts/fused-rgbd-raw
```

The physical evaluator saves the complete accuracy failure and exits 1. Apply
the [unchanged adoption checker](native-depth-regression.md) with these candidate
and raw-audit folders and the canonical depth baseline. The mode remains outside
the depth-only idle-tick supervisor. Subjective maturity stays about 20% versus
mature driving OSS; neither synthetic observability nor room's improvement is a
qualified sustained-localization result.
