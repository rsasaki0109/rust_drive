# Local-plane normals for retained depth tracking

This optional NumPy experiment estimates measured surface normals from a 5×5
pixel plane. It changes only the normal frontend of the
[symmetric Huber control](symmetric-keyframe-depth.md). Original axial depths,
vertices, depth decimation, intrinsics, residual derivatives, Huber weights,
pose solver and marginalization are preserved. The Rust driving pipeline and
its fused-depth estimator remain unchanged.

The preceding [image-based diagnostic](robust-keyframe-depth.md#residual-and-normal-disagreement-diagnostic)
found normal disagreements above 15 degrees in roughly 52%/57% of the room/desk2
candidate occurrences at saved control poses. That mixes shape, noise,
association and pose error; it motivates this comparison without identifying
noise as the cause.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Terminal loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| Symmetric Huber control | 179 / 72 | 179 / 74 | Neither |
| Local-plane normals, same Huber factors | 142 / 77 | 120 / 75 | Both |

Both complete physical-accuracy protocols fail. For accepted updates, room's
maximum position/rotation errors are **0.293017 m / 0.143939 rad**; desk2's
are **0.187061 m / 0.205988 rad**. The symmetric control's corresponding maxima
were **0.172677 m / 0.130111 rad** and **0.239761 m / 0.232270 rad**. Slightly
larger accurate-update counts do not compensate for terminal tracking loss.

The complete tracking protocol regresses. Room rejects three fits by the
condition gate and three by the increment gate, followed by 31 expired
acquisitions. Desk2 rejects five fits by the increment gate and one for absent
current measured support, followed by 53 expired acquisitions. These fitting
rejections occur at the coarse level, before accepted fine-level results.
Loss is terminal and no resets shorten either 179-update denominator.

The [failure diagnostic](../assets/patch-keyframe-depth-v1/failure-diagnostic.json)
retains acquisition identities, rejection stages and the first expired age.
Any reported last-admitted condition belongs to the preceding successful
iteration, not the rejected system. A rejection reason identifies the guard
that failed; it does not establish a physical cause or calibrated uncertainty.

The [matched comparison](../assets/patch-keyframe-depth-v1/comparison-with-parent.json)
uses the same input inventories, manifests and camera as the symmetric
control. Maximum physical errors cover accepted poses only; early loss
truncates that population, so smaller maxima cannot establish improvement
over a complete trajectory. The Rust default and subjective maturity estimate
of **about 20%** remain unchanged.

All accepted fits reach the five-iteration finest-level cap. The
[convergence diagnostic](../assets/patch-keyframe-depth-v1/convergence.json)
finds median maximum-node proposed translation steps of about 0.187 mm in
room and 0.242 mm in desk2 at the returned poses. These local quantities do
not establish convergence or physical accuracy.

## Fixed normal estimator

The [design](../assets/patch-keyframe-depth-v1/design.json) is fixed before
recorded fitting. At every pyramid level, the center and eligible neighbors
must have `0.3 < Z < 5` metres. A neighbor is included only when
`abs(Z_neighbor - Z_center) < 0.05 + 0.01 * Z_center` metres. At least 13 of the
25 samples, including the center, are required. The two-pixel image border
has no normal. Missing centers are never filled.

The operational implementation accumulates first and second moments of the
eligible 3D offsets from the center. It forms the population covariance and
uses its smallest eigenvector. A patch requires middle eigenvalue above
`1e-8 m²` and smallest/middle eigenvalue ratio at most `0.1`. Orient the unit
normal so that its dot product with the optical center vertex is nonnegative.
Optical coordinates are x right, y down and z forward. These support, spread
and planarity thresholds are engineering choices rather than calibrated noise
or uncertainty. A patch crossing a depth change below the neighbor gate can
still mix surfaces.

Only normals change. Center vertices are neither smoothed nor projected onto
the fitted plane. Normal validity and direction change which correspondences
pass the existing selection; this comparison includes that support change.
The preceding Tukey weighting experiment is not combined with this change.

At most five poses and 24 scalar variables are optimized, with at most 18
retained prior coordinates. Iteration caps remain 10/7/5 at levels 4/2/1,
scaled condition at most 1e6, increments at most 0.10 m / 0.15 rad and
returned-pose unweighted RMS at most 0.03 m. Every current acquisition requires
a supported measured edge. Accepted displacement is at most 0.50 m; accepted
pose age above 0.20 s latches loss. There are no pose resets or idle watchdog.

## Independent verification

The checker builds each patch's eligible points separately, subtracts their
centroid and computes an SVD with zero rows for excluded samples. It does not
call the operational moment/eigenvector routine. Only the frozen depth
decimation, vertices and intrinsics frontend is shared. Center-depth/vertex
and camera equality to the control are checked exactly.

Six fixtures across three levels give 18 normal-layer comparisons: a tilted
plane, a noisy plane, a 0.4 m depth step, missing centers, a sparse line and
a rough checkerboard. Support masks agree exactly and normal components agree
within 2e-12. Separate controls exercise the 12/13-sample boundary and a rank-one
covariance even with 25 eligible points. Both sides of the large depth step
retain correct same-side planar normals.

For the single fixed tilted-plane fixture with Gaussian axial noise sigma
2 mm and seed 421016, 18,096 common valid pixels have median/p95 angular errors
of **0.102041 / 0.219099 rad** with the control normals and
**0.021103 / 0.045123 rad** with patch normals. This is a known-plane synthetic
result, not a measured normal-noise calibration or a trajectory guarantee.

The private raw/state auditor reconstructs normal fields from original PNGs
using the SVD implementation, and rebuilds the symmetric factor Jacobian by
matrix products. It checks all reported counts, RMS, H/g, pose steps, factor
ownership, Schur results and atomic commits. Shared central differences test
all twelve source/target pose axes. Both fifteen-frame synthetic motion
branches pass 4 mm / 4 mrad bounds, with **452** iteration/returned-pose
equations checked and nine retained marginal generations. Maximum position
errors are about 0.054 mm and 0.055 mm. All nine inherited report corruptions
are rejected; late elimination failure and clean recovery remain atomic.

Recorded sensor fitting finishes for both cases before either raw/state audit;
both audits finish before numeric physical-reference scoring. RGB and numeric
ground truth do not enter the operational estimator. Each case retains all
179 original updates, a fixed identity initialization outside the denominator,
root gates of 0.10 m / 0.10 rad and reference brackets at most 20 ms. These are
already viewed indoor development recordings, not held-out environments.

The recorded audit verifies **5,895** systems: **5,434** iteration systems,
**262** returned-pose systems and **199** elimination-input systems, plus all
199 Schur-result priors. Room consumes 411 unique raw pair factors across
109 marginal generations; desk2 consumes 430 across 90. All twelve fitting
rejections and all 84 post-expiry acquisitions are independently reproduced.

The core check passes **371** Rust tests and **49** scenario executions with
full sensor-log replays. All **245** deterministic output files match the
preceding trial, as do **257** preceding evidence files. The
[source comparison](../assets/patch-keyframe-depth-v1/previous-rust-source-parity.json)
also verifies 16 pinned sources including the preceding Rust estimator.
CI adds the optional patch-normal analytic and syntax checks.

## Reproduction

Use the pinned optional environment in `scripts/requirements-visual.txt` and
the existing hash-pinned TUM inputs:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-patch-keyframes.py \
  --output artifacts/patch-keyframes-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-patch-keyframes.py \
  --output artifacts/patch-keyframes-recorded
# Expected exit 1: full physical accuracy and continuous tracking fail.
```

Output directories must be fresh. Completed summaries distinguish a physical
quality failure (exit 1) from an integrity exception. Original sources, source
hashes, sensor-only ledgers and compact score fields remain separate from
preceding trials. Images and reference files are not redistributed.
[TUM attribution and license](tum-source-provenance.md).

The [packet manifest](../assets/patch-keyframe-depth-v1/files.json) records
file sizes and SHA-256 digests. Each compressed sensor-only ledger is stored
once. Merge compact `scores.json` fields into matching ledger rows, then
serialize `{summary, frames}` with two-space JSON indentation and a final
newline to reconstruct the original scored report byte for byte. The
[validation receipt](../assets/patch-keyframe-depth-v1/validation.json) records
both reconstructed digests separately from the physical quality result.

Patch normals do not address depth/calibration bias, one-way nearest-pixel
association, pose initialization or frozen marginal-prior approximations.
Runtime bounds are structural work limits rather than measured real-time
performance. Vehicle integration, independent environments, long-duration
mapping and calibrated uncertainty remain unverified.
