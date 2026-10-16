# Retaining linearized depth observations after keyframe eviction

The optional NumPy research tracker now retains a bounded quadratic factor
when a measured depth view leaves its active window. A separate control
discards that information. The first camera stays fixed, and neither branch
replaces the Rust fused-depth estimator or supplies vehicle controls.

The preceding [joint-keyframe trial](joint-keyframe-depth.md) discarded factors
incident to removed views. It scored 72/179 accurate room updates and 51/179
desk2 updates, with loss in both. This new comparison tests information
retention with a separately frozen method. It does not assume that missing
marginalization explains all of those accuracy failures.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| New matched discard control | 92 / 72 | 95 / 51 | Both |
| Retain derived marginal factors | 179 / 70 | 179 / 74 | Neither |

Retention prevents the tested accepted-pose expiry and supplies estimates for
every selected update. It does **not** establish accurate tracking: retained
maximum errors are **0.188117 m / 0.145158 rad** in room and
**0.301526 m / 0.284178 rad** in desk2. The desk2 discard control reaches
1.706278 m. All four complete physical-accuracy protocols fail. The Rust
fused-depth baseline remains the default; the subjective maturity estimate
stays **about 20%**.

The discard branch reproduces the preceding joint trial's 92/95 accepted
updates, 72/51 accurate updates and terminal loss. The added final SPD check
and Cholesky implementation are shared by both new branches. These are
matched viewed-data comparisons, without fresh-environment validation.

## Fixed protocol and implementation

The [design](../assets/marginalized-keyframe-depth-v1/design.json) is fixed
before recorded execution. Both `discard` and `retain` use the same raw-depth
fitter, window selection and admission bounds. The window holds the first
view, newest two selected motion keys, last accepted view if distinct, and
current view, at most five nodes. The first pose is exactly identity;
at most 24 scalar pose variables are fitted. A retained prior covers at most
three non-first reference poses, or 18 scalar coordinates.

All selected acquisitions, original indices 100–279, remain in each recording's
179-update denominator. There is one initialization outside that denominator,
no resets, and terminal accepted-pose expiry at 0.20 s. Inputs are measured
depth PNGs, depth timestamps and the preceding K525 camera interpretation.
RGB pixels and numeric motion-capture labels do not enter estimation. All four
operational branches and raw/state audits finish before numeric reference
scoring. These recordings were already viewed; no held-out claim is made.

The source/target point-to-plane factors, 200-match minimum per pair, Huber
0.01 m weights, source strides 1/2/4 at scales 4/2/1 and iteration caps 10/7/5
remain those of the previous experiment. The combined raw/prior system must
be diagonally scaled SPD with condition at most 1e6. Each applied node increment
is bounded to 0.10 m / 0.15 rad. Final admission checks the returned-pose system
again, requires raw geometric RMS at most 0.03 m and displacement from the
previous accepted current pose at most 0.50 m. The final SPD check is shared
by both new branches. A new frame must provide a real raw factor; prior terms
do not supply fabricated current measurement counts.

Primary physical scoring remains 0.10 m / 0.10 rad in the first optical-camera
frame, with reference interpolation brackets at most 20 ms. Camera coordinates
are x right, y down, z forward, metres, with proper right-handed rotations.

## Which information is retained

When a node is evicted, elimination uses only the preceding marginal factor
and finest-scale currently supported raw factors incident to removed nodes.
Raw factors between retained nodes stay active and are excluded from elimination.
Each consumed older/newer acquisition pair has a unique identity. A consumed
pair cannot also be an active raw factor. This ownership rule prevents
duplicating measurements between the retained quadratic and the raw window.

For removed block `e` and retained block `k`, scaled Cholesky solves produce:

```text
H_prior = H_kk - H_ke solve(H_ee, H_ek)
g_prior = g_k  - H_ke solve(H_ee, g_e)
```

No matrix inverse, diagonal damping or rank filling is used. An unobservable
removed block is rejected. The symmetric Schur result is checked for PSD
within a declared relative 1e-10 arithmetic tolerance, without clipping its
eigenvalues. At most 200 acquisitions and 2,000 consumed pair identities are
allowed. A marginal factor is a linearized approximation of actual supported
observations, without a calibrated covariance interpretation.

The quadratic's reference poses are stored at elimination. For current pose
`(R,t)` and stored anchor `(R0,t0)`, coordinates follow the optimizer's direct
left-translation retraction:

```text
w = Log(R R0^T)
v = t - (R R0^T) t0
delta = [v, w]
J = [[I, -skew(v)], [0, inverse_left_SO3_Jacobian(w)]]
H_current = J^T H_prior J
g_current = J^T (H_prior delta + g_prior)
```

The rotation chart rejects angles at or above pi minus 1e-6. The transported
matrix is a Gauss–Newton approximation. Old marginalized factors remain
frozen at their elimination linearization; active raw correspondences are
recomputed. This is bounded pose-only adjustment, without global smoothing,
landmark optimization, relocalization or loop-closure discovery.

## Failure atomicity and independent checks

Fitting, returned-pose admission and marginalization must all succeed before
reference poses, selected keys, the prior and accepted acquisition clock are
committed. Rejections may advance the observed clock but cannot refresh the
accepted clock or modify the prior. Expired tracking cannot restart itself.
This batch tracker has no idle-tick watchdog.

Five synthetic least-squares tests compare Schur elimination with projection
into a separately computed QR nullspace and verify the minimized quadratic
up to its irrelevant additive constant. Quaternion-derived coordinates and
Jacobians, plus central finite differences, independently check transport
at rotation magnitudes 1e-9, 1e-4, 0.3 and 1.2 rad. The operational skew-vector
log and analytic inverse Jacobian are not used by that quaternion checker.

Two fifteen-frame ray-cast sphere/plane sequences check **286** reported normal
systems. The retained branch exercises **nine** eliminations, including motion
key eviction. Both recover known poses within 4 mm / 4 mrad, with maximum
position errors about 0.14 mm. The optimizer and checker share the preceding
depth-pyramid frontend; raw correspondence equations and incidence-matrix
assembly are separate. Increment verification uses a separate skew-square
Rodrigues implementation.

Nine corruptions include coherent H/g scaling, changed Schur gradients,
duplicated history, changed anchors/origin, incorrect increments, and adding
retained-retained raw factors to the marginalized quadratic. The checker
rejects them. Unobservable planes, absent new depth, singular elimination,
malformed depth types and invalid clocks are also rejected. Injecting an
elimination failure after successful fitting preserves accepted state; the
next clean frame reproduces the pose and prior of an execution omitting the
failed frame.

The recorded audit reconstructs reported combined systems, Schur inputs and
outputs, factor ownership, reference revisions, accepted clocks and rejection
guards from the original source pixels and recorded prior generations.
It checks transported information with independent quaternion mathematics,
normal updates by a spectral solve, and elimination by QR. Prior matrix
comparisons use frozen tolerances of H atol 1e-6, g atol 1e-8 and rtol 1e-8;
discrete counts/identities are exact. The [validation receipt](../assets/marginalized-keyframe-depth-v1/validation.json)
records the resulting integrity and physical-accuracy outcomes separately.

The recorded audit checks **12,449** reported systems: **11,632** iteration
systems, **545** returned-pose systems and **272** elimination-input systems.
It separately checks all 272 Schur-result priors. The retained branches consume
594 and 598 unique raw pair factors, with 135 and 137 marginal generations.
All discrete identities/counts agree, and all thirteen control fit rejections
are independently reproduced. The retained branches have no fit rejections;
the control branches reject another 158 updates after expiry without fitting.
First sensor-only ledgers and scored failures are preserved.

`bash scripts/check.sh` passes 371 core Rust tests and all 49 scenario
executions/replays. All 245 deterministic outputs retain their preceding
digests. Previous estimator sources, Cargo pins and evidence packets are
unchanged. CI adds only the optional analytic marginalization check.

## Reproduction and remaining limits

Use the existing optional environment with
`scripts/requirements-visual.txt` (NumPy/Pillow):

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-marginalized-keyframes.py \
  --output artifacts/marginalized-keyframes-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-marginalized-keyframes.py \
  --output artifacts/marginalized-keyframes-recorded
# Expected exit 1: complete physical accuracy still fails.
```

The recorded command requires the already hash-pinned TUM inputs described in
the [preceding diagnostic](projective-depth-diagnostics.md). Output directories
must be fresh. A completed `comparison.json` contains the physical outcome;
integrity exceptions stop before completed scores. Source archives, sensor-only
ledgers and compact scores retain the first execution. Raw source images and
motion-capture files are not redistributed in the evidence packet.

The [packet manifest](../assets/marginalized-keyframe-depth-v1/files.json)
records file sizes and SHA-256 digests. Each `first/<case>/scores.json` stores
labels separately from its compressed sensor-only ledger. Merging its frame
fields into that ledger and serializing `{summary, frames}` with two-space JSON
indentation and a final newline recovers the original scored report byte for
byte; all four recorded report digests were checked. The
[control comparison](../assets/marginalized-keyframe-depth-v1/control-parity.json)
also verifies that discard decisions, accepted clocks and trajectories reproduce
the preceding joint-keyframe control within declared numerical tolerances.

Information retention does not repair incorrect correspondences, depth bias,
camera calibration, unmodeled motion or frozen-linearization errors. The
engineering weights and PSD/SPD tests are not calibrated confidence. Independent
environments, vehicle integration and real-time operation remain unverified.
