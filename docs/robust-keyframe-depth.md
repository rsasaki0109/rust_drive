# Redescending residual influence in the retained-information tracker

This optional NumPy experiment keeps the symmetric measured-normal residual
and changes its weights to a Tukey biweight. Pair admission requires a minimum
weight mass, and admitted pairs retain zero-weight samples in their unweighted
RMS. The Rust estimator and driving pipeline are unchanged.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| Symmetric Huber control | 179 / 72 | 179 / 74 | Neither |
| Tukey with minimum weight mass | 179 / 71 | 179 / 74 | Neither |

The result is a tradeoff. Room's maximum position/rotation errors increase
from **0.172677 m / 0.130111 rad** to **0.262596 m / 0.137605 rad**.
Desk2's maxima decrease from **0.239761 m / 0.232270 rad** to
**0.198242 m / 0.198650 rad**, but its accurate-update count stays 74/179.
Both complete physical-accuracy protocols still fail; neither branch rejects
an update or loses tracking. The Rust default and subjective maturity estimate
of **about 20%** remain unchanged.

The [matched comparison](../assets/robust-keyframe-depth-v1/comparison-with-parent.json)
retains both exact summaries. Input manifests, camera interpretation, source
hashes and physical accuracy gates match the symmetric control. The changed
weights and pair-admission rule remain explicit experimental choices.

All accepted updates still reach the finest-level five-iteration cap.
The [convergence record](../assets/robust-keyframe-depth-v1/convergence.json)
reports median maximum-node proposed translation steps of approximately
0.070 mm in room and 0.106 mm in desk2 at the returned pose. These local
quantities do not establish physical accuracy or full convergence.

## Residual and normal-disagreement diagnostic

The [diagnostic](../assets/robust-keyframe-depth-v1/residual-diagnostic.json)
reconstructs all finest-level admitted pairs at the saved final poses of the
[symmetric control](symmetric-keyframe-depth.md). It checks source-image hashes
and reproduces every original edge count and RMS before computing statistics.
It fits no poses and reads no numeric ground truth.

| Symmetric control, saved poses | Room | Desk2 |
|---|---:|---:|
| Geometric candidates across admitted pairs | 5,608,725 | 6,224,028 |
| Absolute residual at least 3 cm | 106,235 (1.89%) | 57,732 (0.93%) |
| Measured normal disagreement above 15 degrees | 51.85% | 57.18% |
| Original pairs | 1,346 | 1,359 |
| Pairs passing hypothetical Tukey weight-mass admission | 1,328 | 1,357 |
| Updates without a hypothetical supported current edge | 0/179 | 0/179 |

Large residuals are not ground-truth outlier labels. Normal differences combine
surface shape, measurement noise, correspondence error and pose error; these
figures do not estimate calibrated normal noise. Hypothetical admission at the
control's saved poses does not predict the result of a refitted trajectory.
The cutoffs below were fixed before this diagnostic and new recorded fitting.
Counts are candidate occurrences across overlapping pairs, rather than
independent sensor observations.

## Fixed weights and admission rules

The [design](../assets/robust-keyframe-depth-v1/design.json) retains the original
nearest-pixel candidate selection and the symmetric residual with its exact
fixed-correspondence normal-rotation derivative. Tukey weights are:

```text
c(scale) = 0.03 * scale metres
w(r) = (1 - (r/c)^2)^2   when abs(r) < c
w(r) = 0                otherwise
```

Cutoffs are 0.12/0.06/0.03 m at pyramid scales 4/2/1. The broad coarse cutoff
allows initial alignment before the finer levels restrict residual influence.
These are fixed engineering cutoffs, without calibrated noise interpretation.
The symmetric residual and derivative are imported from the preserved control.
Weights are frozen during each Gauss–Newton linearization.

A pair is admitted only when the **sum of weights is at least 200**. This is
dimensionless weight mass, not a confidence value or statistical effective
sample size. Merely having 200 positive weights is insufficient. For an admitted
pair, `count` remains the number of **all** geometric candidates, and squared
residual totals include every candidate, including zero-weight tails. H/g use
the weighted rows. A denied pair supplies no equation, count or RMS contribution.
Therefore removing a whole pair can change the global RMS population; the
admission rules are part of this comparison.

The new current acquisition must still supply a measured edge. Old information
cannot supply fabricated fresh weight mass. Finest-level weights are frozen
with each incident-only Schur linearization; prior transport and factor
ownership remain unchanged. The retained engine differs from its original
source by only the pair-function import, which the checker verifies.

The active window remains at most five nodes, with 24 scalar pose variables
and an 18-coordinate prior. Iteration caps remain 10/7/5, scaled condition
at most 1e6, increments at most 0.10 m / 0.15 rad, returned-pose unweighted RMS
at most 0.03 m and accepted displacement at most 0.50 m. Accepted-pose age
above 0.20 s latches loss. There are no resets or idle-tick watchdog.

## Independent verification

The separate raw oracle reconstructs the correspondences and normalized-normal
derivative using homogeneous camera matrices, skew matrices and tensor products.
It computes Tukey weights from a separate literal cutoff table, checks weight
mass and reconstructs admitted counts, unweighted RMS and weighted H/g. The
operational weight/sampling functions are not used by that oracle. Both share
the preceding depth-pyramid frontend.

Known weights are checked at zero, half the cutoff, the cutoff and beyond it
at every scale. In a one-dimensional location fixture, 800 small symmetric
inlier offsets and 200 large offsets have known location -0.004 m. Tukey
recovers it; Huber shifts to -0.0065 m. This is an influence control, not a
six-axis motion or measured-trajectory result.

Separate plane fixtures verify zero-weight rejection, rejection when there
are at least 200 positive samples but insufficient weight mass, and preservation
of coarse support. An admitted mixed-depth pair keeps **0.046970 m** raw RMS
despite its zero-weight tail. Restoring the old tail influence fails the
independent H comparison. Shared central differences cover all twelve source
and target pose axes, and endpoint/frame-change controls remain in place.

Both fifteen-frame ray-cast branches pass their 4 mm / 4 mrad bounds. The checker
verifies **261** iteration/returned-pose systems and nine retained marginal
generations, with maximum position errors approximately 0.032 mm for discard
and 0.035 mm for retain. All nine inherited report corruptions are rejected.
Late elimination failure remains atomic, and clean recovery matches an
execution omitting the failed frame.

Recorded estimation finishes for both cases before raw/state audits; both
audits finish before numeric reference scoring. Source hashes are checked
before image decoding. Neither RGB nor numeric physical reference poses enter
the fitter. All original 179 updates stay in each denominator, with one fixed
identity initialization outside it. Root gates remain 0.10 m / 0.10 rad and
reference interpolation brackets at most 20 ms. Both recordings were already
viewed; this is not held-out or new-environment validation.

The recorded audit verifies **7,888** systems: **7,259** iteration systems,
**358** returned-pose systems and **271** elimination-input systems, plus all
271 Schur-result priors. Room consumes 564 unique raw pair factors across 135
marginal generations; desk2 consumes 595 across 136. The
[validation receipt](../assets/robust-keyframe-depth-v1/validation.json)
separates implementation integrity from the failed full physical outcome.

`bash scripts/check.sh` passes **371** core Rust tests and all **49** scenario
executions/replays. All **245** deterministic output files and **236** preceding
evidence files keep their previous digests. The
[source comparison](../assets/robust-keyframe-depth-v1/previous-rust-source-parity.json)
also verifies 16 preceding pinned sources, including the Rust estimator.
CI adds the new optional analytic and syntax checks.

## Reproduction and remaining limits

Use the pinned optional NumPy/Pillow environment from
`scripts/requirements-visual.txt` and the existing hash-pinned TUM inputs:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-residuals.py \
  --output artifacts/depth-residual-diagnostic
OPENBLAS_NUM_THREADS=1 python scripts/check-robust-keyframes.py \
  --output artifacts/robust-keyframes-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-robust-keyframes.py \
  --output artifacts/robust-keyframes-recorded
# Expected exit 1: complete physical accuracy still fails.
```

Directories must be fresh. Completed scores distinguish physical accuracy
failure from an audit exception. First sources and ledgers remain separate
from the previous trials. Source images and motion-capture files are not
redistributed with the evidence.
[TUM source attribution and license](tum-source-provenance.md).

The [packet manifest](../assets/robust-keyframe-depth-v1/files.json) records
file sizes and SHA-256 digests. Each compressed sensor-only ledger is stored
once. Merging compact `scores.json` frame fields into its ledger rows and
serializing `{summary, frames}` with two-space JSON indentation and a final
newline reconstructs the original scored report byte for byte. Both original
report digests are verified.

Tukey's redescending loss is nonconvex and can discard useful measurements.
One-way association, noisy normals, depth/calibration bias and frozen-prior
approximations remain. The weight mass, condition test and residual thresholds
do not establish calibrated confidence. Longer-duration operation, independent
physical environments, real-time performance and vehicle integration remain
unverified.
