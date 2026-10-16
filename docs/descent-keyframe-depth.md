# Frozen-correspondence depth update control

This optional NumPy tracker extends the
[reciprocal tracker](reciprocal-keyframe-depth.md) with a bounded check on each
proposed pose update. It evaluates a nonlinear Huber objective over a fixed
set of measured pixel pairs, including the transported marginal prior, and
halves the update when the acceptance inequality fails. The Rust driving
pipeline and its default estimator remain unchanged.

The preceding [continuation diagnostic](depth-convergence.md) found no
converged fits among ten saved fits after ten additional updates. Its raw RMS
uses changing correspondence populations and omits prior cost. The new control
tests a different, local property: acceptance of each update on the same
population used for its linearization. That property does not establish a
common objective across re-associations or physical trajectory accuracy.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Terminal loss |
|---|---:|---:|---|
| Preceding Rust fused depth | 179 / 64 | 179 / 161 | Neither |
| Reciprocal control | 176 / 88 | 179 / 75 | Neither |
| Frozen-correspondence update control | 176 / 88 | 179 / 75 | Neither |

All **7,427** tentative local updates across both recordings' fit attempts use
the full multiplier **1** and strictly decrease their frozen-population
objective. **No recorded update needs backtracking.** Room accounts for 3,541
updates and desk2 for 3,886. After removing only the new receipts, every one of the 180
acquisition states, reported equations, proposals, failures and priors in each
case is exactly identical to the reciprocal control. Independent scoring also
reproduces its physical results. The
[state comparison](../assets/descent-keyframe-depth-v1/state-parity.json) and
[step statistics](../assets/descent-keyframe-depth-v1/line-search-statistics.json)
retain both checks.

Room's maximum position/rotation errors remain **0.280169 m / 0.197894 rad**;
desk2 remains **0.211982 m / 0.233135 rad**. The three room condition rejections
at indices 252–254 remain in its denominator; desk2 accepts all updates.
Both complete physical-accuracy protocols fail, and the recorded command
returns **exit 1** for that expected quality failure. Neither track expires
or resets. The matched-timestamp comparison retains all 176 common room and
179 common desk2 updates, with unchanged accurate counts.

This adds an executable update check and records its acceptance evidence. It
does not improve recorded accuracy or resolve the preceding convergence
failure. Among admissible proposals, frozen-pair objective overshoot is not observed. The
experiment does not distinguish correspondence switching, calibration/model
error or prior approximation as the cause of drift. The Rust default and
subjective maturity estimate of **about 20%** remain unchanged.

## Fixed update rule

Retain the original 10/7/5 iteration caps, measured reciprocal association,
5×5 PCA normals, symmetric residual and rotating-normal derivative. Form the
original raw-plus-prior H/g and apply every original support, scaled-condition
and full-increment guard before attempting any shorter step. At least 200
reciprocal samples are required per admitted raw pair.

Freeze the original target/source pixel IDs and measured points/normals for
that iteration. For candidate poses, transform those same measurements and
recompute the symmetric normalized-normal residual. Sum the Huber losses with
cutoff **0.01 m**; do not divide by a changing sample count. Add the unchanged
prior's cost `0.5 * delta.T * H_prior * delta + g_prior.T * delta`, using its
original nonlinear left-retraction coordinates and anchors. The stored prior
uses a zero additive constant; this convention also fixes the numerical
allowance below. These engineering weights are not a calibrated likelihood.

Try step multipliers **1, 1/2, 1/4, …, 1/128**, in that order. Each candidate
uses direct left translation and Rodrigues rotation of the scaled full
proposal. Accept the first candidate satisfying:

```text
candidate_cost <= before_cost + 0.0001 * alpha * (gradient · proposal)
                              + 1e-12 * max(1, abs(before_cost))
```

The small final term permits numerical roundoff; strict decrease is reported
separately. Record the pixel-pair hash, before cost, directional derivative,
all attempted costs/limits and accepted multiplier. The trace's `delta` remains
the full unscaled proposal. Convergence still requires every node's full
proposal to be strictly below **10 micrometres / 10 microradians**, so choosing
a small multiplier cannot manufacture convergence. If all eight candidates
fail, reject the entire fit without committing any tracker state.

Re-associate for the next iteration. The returned-pose equation also uses
fresh correspondences, preserving every original final support/condition/RMS,
displacement, Schur and clock/expiry guard. The first optical pose stays fixed;
the current acquisition has no incoming prior. A failed search at a coarse
level is not a permitted coarse skip. The five-node, 24-pose-coordinate and
18-prior-coordinate limits remain unchanged.

## Independent verification

The operational objective copies the original reciprocal pixel selection and
evaluates residuals in the common root frame. The verifier independently
rebuilds SVD normals, flattened source IDs, relative camera matrices and return
projection. It evaluates the objective in the target camera frame with a
separate Huber branch and quaternion-based prior coordinates. It uses spectral
solves and its separate Rodrigues map to check the full proposal, each
candidate, the first passing multiplier and the resulting poses. Shared depth
decimation, vertices, intrinsics and previously validated oracle code remain
explicit dependencies.

Twelve finite-difference axes agree with the complete raw-plus-prior gradient.
An engineered strong-prior fixture over rendered measured pixels rejects
three candidate lengths before accepting **1/8**. A separate small-gradient
fixture independently reproduces exhaustion of all eight lengths. These
priors isolate update bookkeeping and do not represent calibrated sensor
uncertainty; the deliberately large test direction is not claimed as a normal
Gauss–Newton solve.

Six corrupted search reports are rejected: wrong step length, objective,
pixel population, missing candidate, directional derivative and applied pose.
The inherited nine matrix/state corruption controls also pass. Both inherited
fifteen-frame analytic branches satisfy 4 mm / 4 mrad bounds, checking **452**
iteration/returned-pose equations and nine retained marginal generations.
Missing support, planar degeneracy, malformed inputs, terminal expiry and late
elimination failure atomicity remain tested. Source comparisons verify that
the tracker changes only its fitter and the objective import.

## Recorded evaluation protocol

The [design](../assets/descent-keyframe-depth-v1/design.json) and archive freeze
**33** source/context members before fitting both already viewed recordings.
Both cases use all original 180 acquisitions with identity initialization
outside the **179-update denominator**, no resets, unchanged 0.10 m / 0.10 rad
physical gates and reference interpolation brackets at most 20 ms.

Sensor processing finishes for both recordings before either independent
audit. Both private-module audits must complete before numeric reference
scoring. Original images and inventories are hash-verified; RGB and physical
reference poses never enter fitting. The independent audit checks every raw
system, pixel-population receipt, candidate cost, state transition and Schur
output. Rejections remain in each full denominator. Matching accepted
timestamps are reported separately from that denominator.

The recorded audit verifies **8,103** reported systems: **7,427** applied
tentative iterations, **47** skipped coarse systems, **355** returned-pose
systems and **274** elimination inputs. It also checks all 274 Schur outputs
and every candidate objective/step receipt. Room retains 451 unique consumed
raw factors over 135 generations; desk2 retains 503 over 139. The same three
fine-level room condition rejections and 47 coarse skips are independently
reproduced. A tentative step can belong to a fit that is subsequently rejected;
these iteration counts are not counts of committed acquisitions.

The mandatory core check passes **371** Rust tests and **49** scenarios with
complete sensor-log replays. All **245** deterministic outputs, **335** previous
evidence files and 16 pinned preceding native sources retain their hashes.
CI includes the new analytic and syntax checks. The
[validation receipt](../assets/descent-keyframe-depth-v1/validation.json) and
[packet manifest](../assets/descent-keyframe-depth-v1/files.json) preserve
source hashes, math/state verification, unchanged physical scores and exact
reconstruction of the scored reports.

## Reproduction

Use the pinned optional NumPy/Pillow environment and existing original TUM
inputs. Output directories must be fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-descent-keyframes.py \
  --output artifacts/descent-keyframes-analytic-replay
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-descent-keyframes.py \
  --output artifacts/descent-keyframes-recorded-replay
```

The analytic command returns zero for integrity success. The recorded command
returns zero only if both complete physical-accuracy protocols pass; otherwise
its comparison records exit 1. Mathematical integrity is reported separately.
The packet preserves compressed sensor-only ledgers and compact physical
scores. Merging the compact fields into matching ledger rows and serializing
`{summary, frames}` with two-space indentation and a final newline reconstructs
the original scored reports byte for byte.

Original images and physical reference files are not redistributed;
[TUM attribution and terms](tum-source-provenance.md). Frozen-population descent
does not establish convergence under pixel reassociation, confidence
calibration, new environments, real-time performance or vehicle integration.
