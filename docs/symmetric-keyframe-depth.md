# Symmetric measured-normal residuals in the retained-information tracker

The optional NumPy comparison uses both measured surface normals in each
point-to-plane residual, including their rotation derivatives. It keeps the
preceding information-retention engine, nearest-pixel search and work limits.
The Rust driving pipeline and fused-depth default are unchanged.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| Original retained-information control | 179 / 70 | 179 / 74 | Neither |
| Symmetric measured-normal residual | 179 / 72 | 179 / 74 | Neither |

The symmetric residual improves room's accurate-update count by two and
preserves desk2's count. Maximum position/rotation errors shrink from
**0.188117 m / 0.145158 rad** to **0.172677 m / 0.130111 rad** in room,
and from **0.301526 m / 0.284178 rad** to **0.239761 m / 0.232270 rad**
in desk2. Both complete physical-accuracy protocols still fail. Neither branch
loses tracking or rejects an update. These are limited improvements on viewed
data; the Rust fused-depth default and subjective maturity estimate of
**about 20%** remain unchanged.

The [matched comparison](../assets/symmetric-keyframe-depth-v1/comparison-with-parent.json)
retains both methods' exact summaries. Input manifests, source-image hashes,
camera interpretation and all admission bounds match the retained control.
The earlier subpixel trial remains a separate failed comparison.

Both symmetric branches still reach the finest-level five-iteration cap on
all updates. The [convergence record](../assets/symmetric-keyframe-depth-v1/convergence.json)
reports median maximum-node proposed translation steps of approximately
0.074 mm in room and 0.095 mm in desk2 at the returned pose. These remain
local diagnostics rather than a guarantee of convergence or physical accuracy.

## Prior influence before another raw-factor change

The preceding [subpixel trial](subpixel-keyframe-depth.md) failed to improve
the viewed room/desk2 accuracy. The new
[prior diagnostic](../assets/symmetric-keyframe-depth-v1/prior-diagnostic.json)
first examines the archived original retained-information control, without
decoding images or fitting poses. It verifies the ledger digests and prior
chain, reconstructs the prior with the independent quaternion transport, and
subtracts it from the recorded final combined matrix.

| Original retained control | Room | Desk2 |
|---|---:|---:|
| New acquisition's direct prior block | Exactly zero | Exactly zero |
| Median prior trace fraction in combined diagonal scaling | 27.51% | 27.69% |
| Raw matrix SPD and condition <=1e6 in that scaling | 126/179 | 119/179 |
| Median largest prior-anchor translation coordinate | 0.675 mm | 0.709 mm |
| Maximum largest prior-anchor translation coordinate | 56.094 mm | 6.828 mm |

The prior acts on retained poses, which influence the new pose through raw
cross blocks. Raw-only systems fail the stated scaled condition test on
53/179 room and 60/179 desk2 updates; this does not classify every failure as
mathematical rank loss. Trace fractions depend on scaling and are not
confidence percentages. Opposing raw/prior gradients are expected near a
combined optimum and do not establish an erroneous prior. These diagnostics
do not identify the cause of drift or justify weakening old information.
The next experiment keeps retention fixed and tests a different raw residual.

## Fixed correspondences, symmetric residual

The [design](../assets/symmetric-keyframe-depth-v1/design.json) is fixed before
new recorded execution. For matched points `qs`, `qt` and their unit measured
normals `ns`, `nt`, all expressed in the first optical-camera frame:

```text
L = norm(ns + nt)
n = (ns + nt)/L
diff = qs - qt
r_m = n dot diff
tangent = (diff - r_m*n)/L
J_source = [n, qs cross n + ns cross tangent]
J_target = -J_source
```

The normal-rotation term differentiates the measured source normal as well
as its point. Differentiating the target point and target normal yields the
opposite target Jacobian; a common rigid frame change leaves the residual
unchanged. The existing normal agreement gate `ns dot nt > 0.8` bounds `L`
away from zero. Robust Huber weights retain their 0.01 m transition and are
frozen during each Gauss–Newton linearization.

The residual is symmetric for a fixed point pair. Correspondence search still
runs from the newer source view to a rounded pixel in the older target view;
this is not bidirectional matching. Source strides, projective rounding,
normal/depth validity, 0.12/0.08 m point-distance gates and the 200-sample
per-pair minimum are unchanged. At identical poses, the two kernels select
the same samples; independently fitted trajectories can select different
samples as their poses diverge.

`symmetric_keyframe_depth.py` is an exact copy of the original retained engine
with only the pair-function import replaced. The source relationship is
checked. The first camera is identity; at most five active nodes, 24 scalar
pose variables and an 18-coordinate prior use iteration caps 10/7/5. Scaled
condition must be at most 1e6; increments stay within 0.10 m / 0.15 rad.
Returned-pose RMS remains at most 0.03 m, accepted displacement at most 0.50 m,
and accepted-pose age at most 0.20 s before terminal loss. There are no resets.
The signed residual has changed, so its numerical RMS is not a direct
physical-accuracy comparison with the preceding residual.

Eviction retains the same incident-only Schur prior, anchor transport,
unique factor identities and accepted-state transaction. Active factors
between retained nodes are excluded from elimination. The optional batch
tracker has no idle-tick watchdog, calibrated uncertainty or vehicle fusion.

## Independent verification

The separate raw oracle projects with homogeneous camera matrices and
differentiates query points and the normalized sum of normals with skew
matrices and tensor products. It does not call the operational residual or
factor. Both share the preceding depth-pyramid frontend. The preceding
quaternion transport, spectral solve, QR elimination and state audit are
privately bound to this new raw oracle and engine.

Central differences independently check all **12** source/target pose axes
at nonzero poses. Endpoint swapping reverses the residual sign, and a common
rigid frame change preserves its value. Omitting the normal-rotation term
produces a detectable derivative error. Original and new fixed-pose sample
counts agree at all three tested scales: 770, 973 and 1,081.

Both fifteen-frame ray-cast sphere/plane branches pass 4 mm / 4 mrad accuracy
bounds. **261** iteration/returned-pose equations are checked, with maximum
position errors about **0.032 mm** for discard and **0.035 mm** for retain.
The retained branch exercises nine marginal generations. All nine inherited
report corruptions are rejected, along with absent new depth, singular
elimination, planar ambiguity and malformed clocks/depth. An injected late
elimination failure preserves accepted poses, time and the prior; subsequent
recovery matches the execution omitting the failed acquisition.

New recorded estimation runs finish before raw/state audits, and both audits
finish before numeric reference scoring. Inputs are hash-pinned measured axial
depth and acquisition clocks; RGB and numeric ground truth do not enter the
fitter. Both recordings retain all original 179 updates in the accuracy
denominator, with one identity initialization outside it, 0.10 m / 0.10 rad
root gates and reference brackets at most 20 ms. These are already viewed
recordings; this is not new-environment or held-out validation.

The recorded audit verifies **7,916** systems: **7,285** iteration systems,
**358** returned-pose systems and **273** elimination-input systems, plus all
273 Schur-result priors. Room consumes 595 unique raw pair factors across
135 marginal generations; desk2 consumes 598 across 138 generations.
The [validation receipt](../assets/symmetric-keyframe-depth-v1/validation.json)
separates this implementation integrity from the failed complete accuracy
outcome. First sensor-only ledgers, scored reports and sources are retained.

`bash scripts/check.sh` passes **371** core Rust tests and all **49** scenario
executions/replays. All **245** deterministic output files retain their previous
digests, as do **215** preceding evidence files. A
[source comparison](../assets/symmetric-keyframe-depth-v1/previous-rust-source-parity.json)
also verifies 16 preceding pinned sources, including the Rust estimator.
CI adds only the optional analytic factor check and syntax checks.

## Reproduction and limits

Use the optional pinned NumPy/Pillow environment from
`scripts/requirements-visual.txt` and the existing TUM source inputs:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-prior.py \
  --output artifacts/depth-prior-diagnostic
OPENBLAS_NUM_THREADS=1 python scripts/check-symmetric-keyframes.py \
  --output artifacts/symmetric-keyframes-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-symmetric-keyframes.py \
  --output artifacts/symmetric-keyframes-recorded
# Expected exit 1: complete physical accuracy still fails.
```

Directories must be fresh. Completed scores distinguish physical failure
from an audit exception. First sources and recorded ledgers remain separate
from previous variants. Source images and motion-capture files are not
redistributed with the evidence.
[TUM source attribution and license](tum-source-provenance.md).

The [packet manifest](../assets/symmetric-keyframe-depth-v1/files.json) contains
file sizes and SHA-256 digests. Each compressed sensor-only ledger is stored
once. Merging its compact `scores.json` frame fields into the ledger rows and
serializing `{summary, frames}` with two-space JSON indentation and a final
newline reconstructs the original scored report byte for byte. Both original
report digests are verified.

Symmetry of the fixed-pair residual does not remove one-way association,
sensor bias, incorrect calibration, normal-estimation errors or frozen-prior
approximations. Longer-duration operation, new physical environments,
confidence calibration, real-time performance and vehicle integration remain
unverified.
