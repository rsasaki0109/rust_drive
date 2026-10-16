# Continuous depth factors in the retained-information experiment

The optional NumPy comparison replaces the preceding tracker’s nearest-pixel
raw factor with continuous inverse-depth interpolation. The active window,
first fixed origin, marginalization, pose updates and acceptance bounds stay
the same. This experiment does not feed the Rust driving pipeline.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| Preceding retained-information control | 179 / 70 | 179 / 74 | Neither |
| New subpixel factor with information retention | 179 / 69 | 179 / 74 | Neither |

The new raw factor preserves update availability but does not improve these
root-accuracy counts. Maximum errors increase to **0.296214 m / 0.151706 rad**
in room and **0.370140 m / 0.375612 rad** in desk2. The preceding retained
control's maximum position errors were 0.188117 m and 0.301526 m respectively.
Both complete physical-accuracy protocols still fail. The original Rust
fused-depth default and subjective maturity estimate of **about 20%** remain
unchanged. The failed first comparison is preserved in its own evidence packet.

The [comparison receipt](../assets/subpixel-keyframe-depth-v1/comparison-with-parent.json)
retains both methods' exact summaries. Input manifests, camera interpretation,
source-image hashes and admission bounds match the preceding retained control.

Both new branches also reach the five-iteration finest-level cap on all 179
updates. Their final systems propose median maximum-node translation increments
of approximately 0.130 mm in room and 0.182 mm in desk2; the
[saved convergence diagnostic](../assets/subpixel-keyframe-depth-v1/convergence.json)
retains the full percentiles. This trial establishes no convergence improvement.

## Why test another raw factor

The [preceding retained-information trial](marginalized-keyframe-depth.md)
accepted every update but scored only 70/179 room and 74/179 desk2 updates
within the fixed first-camera accuracy gates. Its saved execution shows that
all 179 accepted updates in each recording reached the five-iteration cap at
the finest pyramid level. The final combined system still proposes median
maximum-node translation increments of approximately 0.088 mm in room and
0.100 mm in desk2. Small local residuals and steps do not establish accurate
accumulated motion.

The [diagnostic](../assets/subpixel-keyframe-depth-v1/preceding-convergence.json)
is computed from the preserved traces, without rerunning or changing them.
It motivates testing a continuous raw factor; it does not establish that
pixel rounding caused the recorded drift. Both successful and inaccurate
updates reached that iteration cap.

The [design](../assets/subpixel-keyframe-depth-v1/design.json) was fixed before
new recorded execution. The operational engine is a copy of
`marginalized_keyframe_depth.py` with **only its pair-function import changed**.
The analytic check verifies that exact source relationship. The original
engine, sources and evidence remain reproducible.

## Measurement and linearization

The source is measured axial depth in the optical camera frame: x right,
y down, z forward, metres. The unchanged depth pyramid supplies scales
4/2/1 with source strides 1/2/4. For a source point transformed into target
camera coordinates `q=(x,y,z)`, project it to continuous coordinates:

```text
u = fx*x/z + cx
v = fy*y/z + cy
rho = bilinear(1 / measured_target_depth, u, v)
F = rho - 1/z
dF/dq = rho_u * du/dq + rho_v * dv/dq + [0,0,1/z²]
s = norm(dF/dq)
r_m = F/s
J_query = (dF/dq)/s
```

The factor freezes `s` and robust weights for each linearization. The derivative
includes the projective image-gradient terms, and is exact for `F/s` with that
normalizer held fixed. It is **not** the full derivative of a residual whose
normalizer changes with the candidate pose. This is an engineering conversion
to a local metre-scale residual, without calibrated noise or covariance.

At integer pixel knots, values within 1e-10 pixels are canonicalized and the
right-sided cell derivative is used. Bilinear values remain continuous across
valid adjacent cells. This continuity applies to `rho` and `F`: the cell
gradient and the normalizer recomputed for the next linearization can jump
at knots. The reweighted objective has no general smoothness guarantee.
All four corners must have valid measured normals and
depths strictly between 0.3 and 5 m, with depth spread at most
`0.05 + 0.01*minimum_depth`. Invalid cells are rejected.

The interpolated depth is backprojected on the continuous query ray. Its
distance from the query must remain below 0.12 m at scale 4 or 0.08 m at
scales 2/1. The normalized gradient normal must agree with the transformed
source normal by dot product greater than 0.8. Each pair needs at least 200
samples; Huber weights retain their 0.01 m transition.

For root-frame normal `n` and transformed source point `q_root`, the source
pose Jacobian is `[n, q_root cross n]`; the target Jacobian has the opposite
sign. The first camera is fixed. Up to five active poses and 24 scalar
variables use the unchanged 10/7/5 iteration caps, scaled SPD condition bound
1e6, 0.10 m / 0.15 rad increments, 0.03 m final raw residual bound and
0.50 m accepted displacement bound. No accepted pose older than 0.20 s is
refreshed after loss, and there are no resets.

Only raw factors incident to evicted nodes enter the derived Schur prior.
Retained-retained factors remain active. The same unique pair ownership,
stored-anchor transport, bounded prior and atomic commit rules apply.
The normalization and correspondence linearizations of consumed factors are
frozen with that prior.

This kernel also changes support through four-corner validity, interpolated
target points and gradient-derived normals. The comparison cannot isolate
pixel rounding from those other changes inside the raw factor.

## Independent verification

The new raw oracle uses homogeneous camera matrices, polynomial bilinear
coefficients and a quotient-rule projection derivative. It does not call the
operational sampler or factor. Three explicit scalar four-corner sums verify
interpolation, and central differences check all three query axes with frozen
normalizers. Separate controls check continuity and the right derivative at
an integer knot, missing depths, invalid corner normals, discontinuities and
points behind the camera. Omitting projective gradients produces a detectable
Jacobian error.

The unchanged prior auditor is bound privately to this separate raw oracle
and the new engine. It retains independent quaternion coordinate transport,
spectral normal-system solving, QR elimination and factor-ownership checks.
Both fifteen-frame synthetic branches pass their 4 mm / 4 mrad motion bounds:
**408** iteration/returned-pose equations are checked, with maximum position
errors approximately **0.235 mm** for discard and **0.251 mm** for retain.
The retained branch exercises nine marginal generations. All nine inherited
report corruptions are rejected. Late elimination failure leaves accepted
poses, clocks and the prior unchanged, and clean recovery matches an execution
omitting the failed frame.

Both recorded runs finish before raw/state audits, and all audits finish
before numeric motion-capture scoring. Input hashes are verified before image
decoding; neither RGB pixels nor physical reference poses enter estimation.
All original 179 updates remain in each accuracy denominator, with identity
initialization outside it and 0.10 m / 0.10 rad root gates. Reference brackets
remain at most 20 ms. These are already viewed recordings, without a new
environment or held-out validation claim.

The recorded audit checks **8,336** systems: **7,705** iteration systems,
**358** returned-pose systems and **273** elimination-input systems, plus all
273 Schur-result priors. The branches consume 586/597 unique raw pair factors
over 136/137 marginal generations. Neither branch rejects an update or expires.
These integrity successes do not establish physical pose accuracy.
The [validation receipt](../assets/subpixel-keyframe-depth-v1/validation.json)
records those outcomes separately.

`bash scripts/check.sh` passes **371** core Rust tests and all **49** scenario
executions and replays. All **245** deterministic output files keep their
preceding digests, and **194** preceding evidence files remain unchanged.
The [source comparison](../assets/subpixel-keyframe-depth-v1/previous-rust-source-parity.json)
also verifies 16 preceding pinned sources, including the Rust estimator.
CI adds the new optional analytic check; the driving implementation is unchanged.

## Reproduction and limits

Use the pinned optional NumPy/Pillow environment from
`scripts/requirements-visual.txt` and the existing hash-pinned TUM inputs:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-subpixel-keyframes.py \
  --output artifacts/subpixel-keyframes-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-subpixel-keyframes.py \
  --output artifacts/subpixel-keyframes-recorded
# Expected exit 1: complete physical accuracy still fails.
```

Output directories must be fresh. A completed `comparison.json` distinguishes
physical accuracy failure from an integrity exception. The preceding
retained-information control is preserved in its original packet. The new
source archive and first execution are retained separately; source images
and motion-capture data are not redistributed with the evidence.
[TUM source attribution and license](tum-source-provenance.md).

The [packet manifest](../assets/subpixel-keyframe-depth-v1/files.json) records
file sizes and SHA-256 digests. Each compressed sensor-only ledger is stored
once. Merging the fields from its compact `scores.json` into the corresponding
frame rows, then serializing `{summary, frames}` with two-space JSON indentation
and a final newline, recovers the original scored report byte for byte. Both
original report digests were checked.

The optimizer and independent oracle share the preceding depth-pyramid
frontend. Interpolation cannot repair incorrect calibration, sensor bias,
unmodeled motion or frozen-prior errors. Confidence calibration, independent
environments, idle-tick supervision, real-time operation and vehicle
integration remain outside this experiment.
